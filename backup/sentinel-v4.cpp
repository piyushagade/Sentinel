#include "platform.h"

#if GB_PLATFORM_IS_SENTINEL_V4

  /*
    The secondary uC listens for "breaths" from primary uC.
    The breaths can either be implemented as:
      1. A change in the state of pin A1 on the primary.
      2. An I2C message to turn the timer on the secondary on/off

    If the timer runs out, the secondary resets the primary uC.
    The timer interval can be set by sending an I2C message from primary to secondary. (Experimental) It may also be set using the voltage level of 
    pin A1 (analog output on the primary) and read on pin PB1 (analog input on the secondary)

    I am first going to try I2C method. Using analog pins is a fallback.

    TODO:
    -----------------------------------------------
      1. Test 1 MHz operation (done, ensure to burn bootloader after setting CPU clock frequency to 1 MHz)
      2. Test if 1 ms on ATTINY85 is actually 1 ms (done at 8 MHz and 1 MHz)
      3. Send ACK to primary (done)
      4. Send 2 byte response (done)
      5. Adjustable PB1 pin mode

    Future design upgrades:
    -----------------------------------------------
      1. Add LED on pin ANA_IN
      2. Add decoupling capacitor between VCC and GND
      3. Ability to orogramatically reboot ATTINY85 using the RST pin

    First flash instructions:
    -----------------------------------------------
      1. Select 1 MHz CPU frequency
      2. Burn bootloader
      3. 'Upload using programmer'
  */

  #include <TinyWireS.h>
  #include <EEPROM.h>
  #include <avr/wdt.h>
  #define Wire TinyWireS

  // Sentinel firmware meta data
  #define FIRMWAREVERSION 1.3
  #define FIRMWAREDATE 0223

  // Define addresses and pins
  #define I2CADDRESS 0x8
  #define MOSFET_ON PB3
  #define MOSFET_OFF PB4
  #define ANA_IN PB1
  #define SDA PB0
  #define SCL PB2

  // Define error codes
  #define SUCCESS 0
  #define ERROR 1
  #define REBOOTWARNING 2
  #define PINGRESPONSE 3
  #define CONFIGLOCKEDERROR 4

  // State variables
  bool SNTL_ENABLED = false;
  bool REBOOTPENDING = false;
  bool PRIMARY_OFF = false;
  bool I2C_ENABLED = false;
  bool SENTINENCE_ENABLED = false;;
  bool CONFIGUNLOCKED = false;
  bool PB1_MODE = INPUT;

  // Default sentinence interval is 2 minutes
  int INTERVALBASE = 15;
  int INTERVALMULTIPLIER = 8;

  // Time trackers (unit: seconds)
  int sentinence_start_timestamp = millis() / 1000;

  // Utility macros
  #define adc_disable() (ADCSRA &= ~(1<<ADEN))

  // Debug
  bool DIAGNOSTIC = true;
  bool flag = false;

  /*
    Change WDT state
  */
  void wdt(String state) {
    
    // Enable WDT
    if (state == "on") {
      wdt_enable(WDTO_4S);
    }

    // Disable WDT
    else if (state == "off") {
      wdt_disable();
    }

    // Reset WDT
    else if (state == "reset") {
      wdt_reset();
    }
  }

  /*
    Set sentinence state
  */
  void setsentinence (String state) {
    if (state == "on" && !SENTINENCE_ENABLED) {
      SENTINENCE_ENABLED = true;
      sentinence_start_timestamp = millis() / 1000;

      // Send ack to primary
      sendresponse(SUCCESS);

      // Lock Sentinel configuration
      CONFIGUNLOCKED = false;
      
      flag = false;
    }
    else if (state == "off" && SENTINENCE_ENABLED) {
      SENTINENCE_ENABLED = false;
      sentinence_start_timestamp = millis() / 1000;

      // Lock Sentinel configuration
      CONFIGUNLOCKED = false;

      // Send ack to primary
      sendresponse(SUCCESS);
    }
    else if (state == "heartbeat" && SENTINENCE_ENABLED) {
      sentinence_start_timestamp = millis() / 1000;

      // Send ack to primary
      sendresponse(SUCCESS);

      // Lock Sentinel configuration
      CONFIGUNLOCKED = false;
    }
  }

  /*
    Reboot the primary microcontroller
    Inverted logic due to P-channel MOSFETs
  */

  void reboot () {
    
    // Send pre-warning to primary
    sendresponse(REBOOTWARNING);
    
    // Wait for 10 seconds
    delay(10000);

    // Turn off the primary
    trigger("off");

    // A 500 ms delay
    tws_delay(500);
    
    // Turn on the primary
    trigger("on");
  }


  /*
    Trigger On/Off MOSFETs
  */
  void trigger (String state) {

    if (state == "on") {
      PRIMARY_OFF = false;
      
      digitalWrite(MOSFET_ON, !LOW);
      digitalWrite(MOSFET_ON, !HIGH);
      tws_delay(100);
      digitalWrite(MOSFET_ON, !LOW);
      
      // Enable I2C and set up ISR
      tws_delay(1500);
      Wire.begin(I2CADDRESS);
      I2C_ENABLED = true; tws_delay(10);
      Wire.onReceive(i2clistener);
    }

    else if (state == "off") {
      PRIMARY_OFF = true;

      digitalWrite(MOSFET_OFF, !LOW);
      digitalWrite(MOSFET_OFF, !HIGH);
      tws_delay(100);
      digitalWrite(MOSFET_OFF, !LOW);

      // Disable I2C
      pinMode(SDA, INPUT); digitalWrite(SDA, LOW);
      pinMode(SCL, INPUT); digitalWrite(SCL, LOW);
      I2C_ENABLED = false; tws_delay(500);
    }
  }

  /*
    Blink bits
  */
  void blinkbits (byte b) {
      for (int i = 0; i < 8; i++) {
        if (bitRead(b, i) == 1) {
          analogWrite(ANA_IN, 1023);
          delay(1000);
        } else {
          analogWrite(ANA_IN, 768);
          delay(500);
        }
      }
      digitalWrite(ANA_IN, LOW);
  }

  /*
    Blink function
  */
  void blinkmode (String mode, int times) {
    
    // Set pin as output
    pinMode(ANA_IN, OUTPUT);

    int on, off;

    if (mode == "slow") {
      tws_delay(100);
      on = 1000;
      off = 1000;
    }

    else if (mode == "fast") {
      on = 500;
      off = 500;
    }

    else if (mode == "beacon") {
      on = 200;
      off = 500;
    }

    else {
      on = 200;
      off = 2000;
    }

    tws_delay(100);
    int counter = 0;
    while (counter++ < times) {
      digitalWrite(ANA_IN, HIGH); tws_delay(on); digitalWrite(ANA_IN, LOW); tws_delay(off);
    }
  }

  /*
    Send two byte response to primary
  */
  void sendresponse (uint16_t response) {

    // Turn WDT on
    wdt("on");

    // Send low byte
    Wire.write ((byte) response); delay(50);
    
    // Send high byte
    Wire.write ((byte) (response >>= 8)); delay(50);
    
    // Turn WDT off
    wdt("off");
  }

  /*
    Read incoming I2C commands
  */
  void i2clistener(int byteCount) {

    // Do no accept new commands if a reboot is pending
    if (REBOOTPENDING) return;

    int elapsed = 0;
    while (elapsed++ <= 2000 * 1000 && !Wire.available()) delayMicroseconds(10);
    
    String state = "";
    int selector = 0;
    while (Wire.available()) {
      
      // Convert received byte to decimal (range is 0 to 127)
      byte b = Wire.read();
      char s[4];
      itoa(b, s, 10);
      int x = atoi(s);

      // Config and Power control
      if (x == 0)  state = "ping";
      if (x == 1)  state = "configunlock";
      if (x == 2)  state = "configlock";
      if (x == 3)  state = "fwversion";
      if (x == 4)  state = "fwdate";
      if (x == 5)  state = "off";
      if (x == 6)  state = "reboot";
      if (x == 7)  state = "toggle-pb1-mode";

      // Sentenence control
      else if (x == 10) state = "sentinence";
      else if (x == 11) state = "unsentinence";
      else if (x == 12) state = "heartbeat";

      // Interval base interval control (10 options)
      else if (x >= 20 && x <= 29) { state = "setintervalbase"; selector = x - 20; }

      // Interval duration control (20 levels)
      else if (x >= 30 && x <= 49) { state = "setintervalmultiplier"; selector = x - 30; }
      
      // TinyWire library needs this to detect the end of an incoming message
      TinyWireS_stop_check();
    }
    
    // Respond to ping
    if (state == "ping") {
      sendresponse(PINGRESPONSE);
    }
    
    // Temporarily unlock config
    if (state == "configunlock") {
      CONFIGUNLOCKED = true;
      sendresponse(SUCCESS);
    }
    
    
    // Lock config
    if (state == "configlock") {
      CONFIGUNLOCKED = false;
      sendresponse(SUCCESS);
    }
    
    // Send firmware version info; Needs the message to be divided by 100 on the receiving end
    if (state == "fwversion") {
      sendresponse(FIRMWAREVERSION * 100);
    }
    
    // Send firmware date info
    if (state == "fwdate") {
      sendresponse(FIRMWAREDATE);
    }
    
    // Reboot the primary uC
    if (state == "reboot") {
      reboot();
    }
    
    // Toggle the PB1 pin mode
    if (state == "toggle-pb1-mode") {
      if (PB1_MODE == INPUT) {
        pinMode(ANA_IN, OUTPUT);
        PB1_MODE = OUTPUT;
      }
      else if (PB1_MODE == OUTPUT) {
        pinMode(ANA_IN, INPUT);
        PB1_MODE = INPUT;
      }
    }
    
    // Sentinence (monitoring on)
    else if (state == "sentinence") {
      setsentinence("on");
    }

    // Unsentinence (monitoring off)
    else if (state == "unsentinence") {
      setsentinence("off");
    }

    // Heartbeat (kick the dog)
    else if (state == "heartbeat") {
      setsentinence("heartbeat");
    }

    // Set sentinence interval's multiplier
    else if (state == "setintervalmultiplier") {
      
      if (!CONFIGUNLOCKED) sendresponse(CONFIGLOCKEDERROR);

      /*
        Set the interval multiplier.
        The  range for the multiplier is 1 to 20.
      */

      INTERVALMULTIPLIER = (uint8_t) selector + 1;
      sendresponse(INTERVALMULTIPLIER);
    }

    // Set sentinence interval's base
    else if (state == "setintervalbase") {

      if (!CONFIGUNLOCKED) sendresponse(CONFIGLOCKEDERROR);
      
      /*
        Set the base interval.
        The range for selector index is 0 to 6.

        The options correspond to:
        1 sec, 5 sec, 10 sec, 15 sec, 30 sec, 1 min, 1 min 30 seconds
      */
      uint8_t options[] = { 1, 5, 10, 15, 30, 60, 90 };

      // Check if the selector is out of range (the length of the options array).
      if (selector < sizeof (options) / sizeof (options[0])) {
        INTERVALBASE = options[selector];
        sendresponse(INTERVALBASE);
      }
      else {
        sendresponse(ERROR);
      }
    }

    // Clear/reset state
    state = "";
  }

  void setup() {
      
    // Set pinModes
    pinMode(MOSFET_ON, OUTPUT);
    pinMode(MOSFET_OFF, OUTPUT);
    pinMode(ANA_IN, INPUT);
    
    // De-trigger MOSFETs
    digitalWrite(MOSFET_ON, !LOW);
    digitalWrite(MOSFET_OFF, !LOW);

    blinkmode("fast", 5);

    // Turn on primary by default in 1.5 seconds
    tws_delay(1500); trigger("on");
  }

  void loop() {

    // TinyWire library needs this to detect end of an incoming message
    TinyWireS_stop_check();

    // Check if the reboot condition is met. The bytes need to be converted to integers before comparisions
    REBOOTPENDING = int(millis() / 1000) - int(sentinence_start_timestamp) > int(INTERVALBASE) * int(INTERVALMULTIPLIER);

    if (DIAGNOSTIC && SENTINENCE_ENABLED) {

      // Send right after the Sentinel is enabled
      if (!flag && !(millis() / 1000 - sentinence_start_timestamp > INTERVALBASE * INTERVALMULTIPLIER)) {

        sendresponse(9999);
        sendresponse(int(INTERVALBASE) * int(INTERVALMULTIPLIER));
        sendresponse(millis() / 1000 - sentinence_start_timestamp > INTERVALBASE * INTERVALMULTIPLIER);
        sendresponse(int(millis() / 1000) - int(sentinence_start_timestamp) > int(INTERVALBASE) * int(INTERVALMULTIPLIER));

        flag = true;
      }

      // Send if the reboot condition is "met"
      if (millis() / 1000 - sentinence_start_timestamp > INTERVALBASE * INTERVALMULTIPLIER) {
        
        sendresponse(1234);
        sendresponse(int(INTERVALBASE) * int(INTERVALMULTIPLIER));
        sendresponse(millis() / 1000 - sentinence_start_timestamp > INTERVALBASE * INTERVALMULTIPLIER);
        sendresponse(int(millis() / 1000) - int(sentinence_start_timestamp) > int(INTERVALBASE) * int(INTERVALMULTIPLIER));
      }
    }

    // Check if a heartbeat has not been received in the last INTERVALBASE * INTERVALMULTIPLIER microseconds
    if (SENTINENCE_ENABLED && REBOOTPENDING) {

      // Turn off sentinence
      SENTINENCE_ENABLED = false;
      sentinence_start_timestamp = millis() / 1000;

      // Notify using the LED
      blinkmode("fast", 3);

      // Lock Sentinel configuration
      CONFIGUNLOCKED = false;
      
      // Reboot the primary
      reboot();
      
      // Reset state variable
      REBOOTPENDING = false;
    }

    // 1 ms delay
    delay(1);
    
  }
#endif