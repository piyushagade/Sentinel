#include "platform.h"

#if GB_PLATFORM_IS_SENTINEL_V71
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
        1. Test 1 MHz operation (done)
        2. Test if 1 ms on ATTINY85 is actually 1 ms (done, at 8 MHz and 1 MHz)
        3. Send ACK to primary (done)
        4. Send 2 byte response (done)
        5. Adjustable PB1 pin mode
        6. Add sleep mode to save power (done)

    Future design upgrades:
    -----------------------------------------------
        1. Add LED on pin ANA_IN (done)
        2. Add decoupling capacitor between VCC and GND (done)
        3. Ability to programatically reboot ATTINY85 using the RST pin

    First flash instructions:
    -----------------------------------------------
        1. Select 1 MHz CPU frequency
        2. Burn bootloader
        3. 'Upload using programmer'

    Legendary advices:
    -----------------------------------------------
        1. The dynamic memory needs to be below 45% full. If not, the I2C communication does'nt work.
    
    EEPROM map:
    -----------------------------------------------
    The program is designed to store 2 bytes of data. Hence, all location indices should be an even number. 
    Indices are double that of the corresponding IDs.

        ID   |  Name                         |  Description
    -----------------------------------------------------------------------------------------
        0       |  EEPROM initialization flag   |  If 1, the EEPROM has values stored in it
        1       |  I2C address                  |  Stores the I2C address of the device
        2       |  SNTL Interval base           |  Stores the base interval value
        3       |  SNTL Interval multiplier     |  Stores the interval multiplier value
        4       |  PSM Interval base            |  Stores the base interval value
        5       |  PSM Interval multiplier      |  Stores the interval multiplier value
        6       |  Primary's fault counter      |  Stores the count of reboots of primary uC
        7       |  Secondary's fault counter    |  Stores the count of reboots of the device
        8       |  PSM state flag               |  Is the device waking up from PSM sleep?
        9       |  Time tracker value           |  The value of sentinence_start_timestamp

    */

    #include <TinyWireS.h>
    #include <EEPROM.h>
    #include <avr/wdt.h>
    #include <avr/sleep.h>
    #include <avr/interrupt.h>
    #include <avr/power.h>

    #define Wire TinyWireS

    struct LOCATIONS {
    uint16_t init = 0x00;
    uint16_t address = 0x01;
    uint16_t sentinence_base = 0x02;
    uint16_t sentinence_multiplier = 0x03;
    uint16_t psm_base = 0x04;
    uint16_t psm_multiplier = 0x05;
    uint16_t primary_fault = 0x06;
    uint16_t secondary_fault = 0x07;
    uint16_t psm_state_flag = 0x08;
    uint16_t time_tracker = 0x09;
    uint16_t beacon_base = 0x02;
    uint16_t beacon_multiplier = 0x03;
    uint16_t beacon_enabled = 0x0A;
    uint16_t send_ack = 0x0B;
    uint16_t beacon_mode = 0x0C;
    } location;

    // Utility functions
    #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
    #define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

    // Sentinel firmware meta data
    #define FIRMWARE_MAJOR_VERSION 2
    #define FIRMWARE_MINOR_VERSION 10
    #define FIRMWARE_MONTH 8
    #define FIRMWARE_DATE 4

    // Define addresses and pins
    #define MOSFET_ON PB3
    #define MOSFET_OFF PB4
    #define BEACON PB1
    #define SDA PB0
    #define SCL PB2

    // Define error codes
    #define SUCCESS 0
    #define ERROR 1
    #define INVALID_ACTION 2
    #define PING_RESPONSE 3
    #define CONFIG_LOCKED_ERROR 4
    #define INVALID_OPTION 5
    #define INVALID_ACTION_SEN_ON 6
    #define INVALID_ACTION_SEN_OFF 7

    // State variables
    bool USE_PSM = false;
    bool SEND_ACK = true;
    bool SNTL_ENABLED = false;
    bool BEACON_ENABLED = true;
    bool BEACON_TRIGGER_PENDING = false;
    bool PRIMARY_REBOOT_PENDING = false;
    bool PRIMARY_OFF = false;
    bool I2C_ENABLED = false;
    bool SENTINENCE_ENABLED = false;;
    bool CONFIG_UNLOCKED = false;
    bool PSM_WAKE_FLAG = false;
    bool PB1_MODE = INPUT;
    uint8_t INTERVAL_SET_MODE = 0;

    uint8_t BEACON_MODE = 1;
    uint8_t SNTL_INTERVAL_BASE = 15;
    uint8_t SNTL_INTERVAL_MULTIPLIER = 8;
    uint8_t PSM_INTERVAL_BASE = 15;
    uint8_t PSM_INTERVAL_MULTIPLIER = 8;
    uint8_t BCN_INTERVAL_BASE = 5;
    uint8_t BCN_INTERVAL_MULTIPLIER = 1;

    // Sentinel I2C address (0 - 127)
    uint8_t I2C_ADDRESS = 9;

    // Time trackers (unit: seconds)
    uint16_t sentinence_start_timestamp = millis() / 1000;

    // Function declarations
    void trigger(int);
    void blinkmode (int, int);
    void wdt(int);
    void sendresponse (uint16_t);
    void setsentinence (int);
    void reboot (int);
    void shutdown ();
    void i2clistener(int);
    typedef void (*callback_t_func)();
    void shield(int, callback_t_func);

    /*
    System sleep
    The system wakes up when watchdog is timed out.
    */
    void sleep(int duration) {

    /*
        https://www.gammon.com.au/forum/?id=11497, Sketch H
        https://github.com/combs/Arduino-Watchdog-Handler
        https://www.hackster.io/Itverkx/sleep-at-tiny-371f04
    */

    // Write PSM state flag
    memwrite(location.psm_state_flag, 1);

    // Write time tracker value
    memwrite(location.time_tracker, millis() - sentinence_start_timestamp);
    
    // Switch ADC off
    cbi(ADCSRA, ADEN);
    
    // Added: Clear various "reset" flags
    MCUSR = 0;
    
    // // Added: Allow changes, disable reset
    // WDTCR = bit (WDCE) | bit (WDE);

    // // set interrupt mode and an interval 
    // WDTCR = bit (WDIE) | bit (WDP3) | bit (WDP0);

    // Turn on WDT; Set sleep duration to 2 second                
    wdtduration(duration);              

    // Added: Pat the dog 
    wdt_reset();   
    
    // Sleep mode is set here
    set_sleep_mode(SLEEP_MODE_PWR_DOWN); 
    sleep_enable();
    
    // Enable the Interrupts so the wdt can wake the device up
    sei();                               

    // System sleeps here
    sleep_cpu();     

    // Added: enable interrupts, Needed?
    cli();                   

    // System continues execution here when watchdog timed out 
    sleep_disable();         

    // Turn ADC back on            
    sbi(ADCSRA, ADEN);                    
    }

    /*
    Change WDT state
    */
    void wdtduration(int duration) {
    if (duration >= WDTO_15MS) wdt_enable(duration);
    else if (duration <= -1) wdt(0x02);
    }
    void wdt(int state) {
    
    // Enable WDT
    if (state == 0x01) {
        wdt_enable(WDTO_4S);
    }

    // Disable WDT
    else if (state == 0x02) {
        MCUSR = 0x0;
        wdt_disable();
    }
    
    // Initialize WDT
    else if (state == 0x03) {

    }

    // Reset WDT
    else if (state == 0x04) {
        wdt_reset();
    }
    }

    /*
    Send two response to the primary
    The function sends two bytes of data
    */
    void sendresponse (uint16_t response) {

    if (!SEND_ACK) return;
    
    // // Turn WDT on
    // wdt(0x01);

    // Send low byte
    Wire.write ((byte) response); delay(50);
    
    // Send high byte
    Wire.write ((byte) (response >>= 8)); delay(50);
    
    // // Turn WDT off
    // wdt(0x02);
    }

    /*
    Send two response to the primary
    The function sends two bytes of data
    */
    void sendresponseforced (uint16_t response) {

    // // Turn WDT on
    // wdt(0x01);

    // Send low byte
    Wire.write ((byte) response); delay(50);
    
    // Send high byte
    Wire.write ((byte) (response >>= 8)); delay(50);
    
    // // Turn WDT off
    // wdt(0x02);
    }

    /*
    Set sentinel's state
    */
    void setsentinence (int state) {
    if (state == 0x01 && !SENTINENCE_ENABLED) {
    // if (state == 0x01) {
        SENTINENCE_ENABLED = true;
        sentinence_start_timestamp = millis() / 1000;

        // Send ack to primary
        sendresponse(SUCCESS);

        // Lock Sentinel configuration
        CONFIG_UNLOCKED = false;
    }
    else if (state == 0x01 && SENTINENCE_ENABLED) {
        sendresponse(INVALID_ACTION_SEN_ON);
    }

    else if (state == 0x02 && SENTINENCE_ENABLED) {
    // else if (state == 0x02) {
        SENTINENCE_ENABLED = false;
        sentinence_start_timestamp = millis() / 1000;

        // Lock Sentinel configuration
        CONFIG_UNLOCKED = false;

        // Send ack to primary
        sendresponse(SUCCESS);
    }
    else if (state == 0x02 && !SENTINENCE_ENABLED) {
        sendresponse(INVALID_ACTION_SEN_OFF);
    }

    else if (state == 0x03 && SENTINENCE_ENABLED) {
        sentinence_start_timestamp = millis() / 1000;

        // Send ack to primary
        sendresponse(SUCCESS);

        // Lock Sentinel configuration
        CONFIG_UNLOCKED = false;
    }

    // Send ack to primary
    else sendresponse(INVALID_ACTION);
    }

    /*
    Reboot the primary microcontroller
    Inverted logic due to P-channel MOSFETs
    */

    void reboot (int device) {

    // Primary
    if (device == 0x02) {
        
        // Wait for 1 seconds
        tws_delay(1000);

        // Turn off the primary
        trigger(0x02);

        // A 250 ms delay
        tws_delay(1000);
        
        // Turn on the primary
        trigger(0x01);
    }

    // Secondary
    else if (device == 0x01) {
        
        // Force reset
        wdt_enable(WDTO_15MS);
        while (true) delay(1);
    }
    }

    /*
    Turn off the primary microcontroller
    Inverted logic due to P-channel MOSFETs
    */

    void shutdown () {

    // Turn off sentinence
    SENTINENCE_ENABLED = false;

    // Turn off the primary
    trigger(0x02);
    }

    /*
    Set I2C state and listener
    */

    void i2cstate(int state) {

    // Enable I2C and set up ISR
    if (state == 0x01) {
        tws_delay(100);
        Wire.begin(I2C_ADDRESS);
        I2C_ENABLED = true; tws_delay(10);
        Wire.onReceive(i2clistener);
    }

    // Disable I2C
    else if (state == 0x02) {
        pinMode(SDA, INPUT); digitalWrite(SDA, LOW);
        pinMode(SCL, INPUT); digitalWrite(SCL, LOW);
        I2C_ENABLED = false; tws_delay(100);
    }
    }

    /*
    Trigger On/Off MOSFETs
    */
    void trigger (int state) {

    if (state == 0x01) {
        PRIMARY_OFF = false;
        
        digitalWrite(MOSFET_ON, !LOW);

        digitalWrite(MOSFET_ON, !HIGH);
        tws_delay(50);
        digitalWrite(MOSFET_ON, !LOW);
        
        // Enable I2C and set up ISR
        i2cstate(0x01);
    }

    else if (state == 0x02) {
        PRIMARY_OFF = true;

        digitalWrite(MOSFET_OFF, !LOW);

        digitalWrite(MOSFET_OFF, !HIGH);
        tws_delay(50);
        digitalWrite(MOSFET_OFF, !LOW);

        // Disable I2C
        i2cstate(0x02);
    }
    }

    /*
    Blink function
    */
    void blinkmode (int mode, int times) {
    
    // Set pin as output
    pinMode(BEACON, OUTPUT);

    int on, off;

    // Slow
    if (mode == 0x01) {
        tws_delay(100);
        on = 1000;
        off = 1000;
    }

    // Fast
    else if (mode == 0x02) {
        on = 500;
        off = 500;
    }

    // Blink/beacon
    else if (mode == 0x03) {
        on = 100;
        off = 250;
    }

    // Long
    else if (mode == 0x04) {
        on = 3000;
        off = 1000;
    }

    else {
        on = 200;
        off = 2000;
    }

    tws_delay(100);
    int counter = 0;
    while (counter++ < times) {
        digitalWrite(BEACON, !HIGH); tws_delay(on); 
        
        digitalWrite(BEACON, !LOW); if (times > 1) tws_delay(off);
    }
    }

    /*
    Format EEPROM on first run
    */
    void memformat () {
    
    blinkmode(0x04, 1);
    
    // Write init flag
    memwrite(location.init, 0x0F);

    // Write I2C address
    memwrite(location.address, I2C_ADDRESS);

    // Write base interval
    memwrite(location.sentinence_base, SNTL_INTERVAL_BASE);

    // Write interval multiplier
    memwrite(location.sentinence_multiplier, SNTL_INTERVAL_MULTIPLIER);

    // Write base PSM interval
    memwrite(location.psm_base, PSM_INTERVAL_BASE);

    // Write interval PSM multiplier
    memwrite(location.psm_multiplier, PSM_INTERVAL_MULTIPLIER);

    // Write primary's fault counter
    memwrite(location.primary_fault, 0);

    // Write secondary's fault counter
    memwrite(location.secondary_fault, 0);

    // Write PSM state flag
    memwrite(location.psm_state_flag, 0);

    // Write time tracker value
    memwrite(location.time_tracker, 0);
    
    // Beacon enabled flag
    memwrite(location.beacon_enabled, BEACON_ENABLED);
    
    // Send acknowledgements flag
    memwrite(location.send_ack, SEND_ACK);
    
    // Beacon mode
    memwrite(location.beacon_mode, BEACON_MODE);

    }

    /*
    Get data from EEPROM and assign state variables
    */
    void memfetch () {
    
    // Base interval
    SNTL_INTERVAL_BASE = memread(location.sentinence_base);

    // Interval multiplier
    SNTL_INTERVAL_MULTIPLIER = memread(location.sentinence_multiplier);

    // Base PSM interval
    PSM_INTERVAL_BASE = memread(location.psm_base);

    // Interval PSM multiplier
    PSM_INTERVAL_MULTIPLIER = memread(location.psm_multiplier);

    // PSM state flag
    PSM_WAKE_FLAG = memread(location.psm_state_flag);

    // Beacon flag
    BEACON_ENABLED = memread(location.beacon_enabled);

    // Beacon flag
    SEND_ACK = memread(location.send_ack);

    // Beacon mode
    BEACON_MODE = memread(location.beacon_mode);
    }

    /*
    Write two bytes to EEPROM
    */
    void memwrite (uint8_t location, uint16_t data) {

    byte low = data & 0xFF;
    byte high = (data >> 8) & 0xFF;

    // Write low byte
    EEPROM.write(location * 2, low);

    // Write high byte
    EEPROM.write(location * 2 + 1, high);
    }

    /*
    Read two bytes from EEPROM
    */
    uint16_t memread (uint8_t location) {

    // Write low byte
    byte low = EEPROM.read(location * 2);

    // Write high byte
    byte high = EEPROM.read(location * 2 + 1);

    return (high << 8) | low;
    }

    /*
    Watchdog Interrupt Service
    This is executed when the watchdog timed out
    */
    // ISR(WDT_vect) {
    ISR(TIMER1_COMPA_vect) {
    blinkmode(0x01, 3);
    }

    /*
    Read incoming I2C commands
    */
    void i2clistener(int byteCount) {

    // Do no accept new commands if a reboot is pending
    if (PRIMARY_REBOOT_PENDING) return;

    // // Do no accept new commands if a beacon is pending
    // if (BEACON_TRIGGER_PENDING) return;

    int elapsed = 0;
    while (elapsed++ <= 2000 * 1000 && !Wire.available()) delayMicroseconds(10);
    
    int selector = 0;
    while (Wire.available()) {
        
        // Convert received byte to decimal (range is 0 to 127)
        byte b = Wire.read();
        char s[4];
        itoa(b, s, 10);
        int x = atoi(s);

        // Ping
        if (x == 0) { 
        sendresponseforced(PING_RESPONSE);
        };

        // Unlock configuration
        if (x == 1) { 
        CONFIG_UNLOCKED = true;
        sendresponseforced(SUCCESS);
        }
        
        // Lock configuration
        if (x == 2) {
        CONFIG_UNLOCKED = false;
        sendresponseforced(SUCCESS);
        }      

        // Firmware version
        if (x == 3) {
            sendresponseforced(FIRMWARE_MAJOR_VERSION * 100 + FIRMWARE_MINOR_VERSION);
        }

        // Firmware month and date
        if (x == 4) {
            sendresponseforced(FIRMWARE_MONTH + "-" + FIRMWARE_DATE);
        }    

        // Power off the primary indefinitely (will restart when secondary self reboots from WDT in an hour)
        if (x == 5) {
            
        }

        // Reboot the primary
        if (x == 6) {
            sendresponse(SUCCESS);
            
            // Lock Sentinel configuration
            CONFIG_UNLOCKED = false;

            // Reboot the secondary
            reboot(0x02);

            delay(500);

            // Reboot the secondary
            reboot(0x01);
        }

        // Reset fault counters
        if (x == 7) {
            memwrite(location.primary_fault, 0);
            memwrite(location.secondary_fault, 0);
            sendresponse(SUCCESS);
        }

        // Primary's fault counter
        if (x == 8) {
            sendresponseforced(SUCCESS);
            // sendresponseforced(memread(location.primary_fault));
        }        

        // Secondary's fault counter
        if (x == 9) {
            sendresponseforced(SUCCESS);
            // sendresponseforced(memread(location.secondary_fault));
        }    

        // Put secondary to sleep
        if (x == 10) {

        }

        // Set interval setting mode to sentinence interval
        if (x == 11) {
            if (!CONFIG_UNLOCKED) sendresponse(CONFIG_LOCKED_ERROR);
            else sendresponse(SUCCESS);
            INTERVAL_SET_MODE = 0;
        }

        // Set interval setting mode to power saving interval
        if (x == 12) {
            if (!CONFIG_UNLOCKED) sendresponse(CONFIG_LOCKED_ERROR);
            else sendresponse(SUCCESS);
            INTERVAL_SET_MODE = 1;
        }

        // Reboot the secondary microcontroller
        if (x == 13) {
            sendresponse(SUCCESS);  delay(50);
            reboot(0x01);
            PRIMARY_REBOOT_PENDING = false;
        };

        // Set interval setting mode to beacon interval
        if (x == 14) {
            if (!CONFIG_UNLOCKED) sendresponse(CONFIG_LOCKED_ERROR);
            else sendresponse(SUCCESS);
            INTERVAL_SET_MODE = 2;
        }

        // Turn on periodic beacon
        if (x == 15) {
            if (BEACON_ENABLED) {
            sendresponse(SUCCESS);
            return;
            }

            sendresponse(SUCCESS);
            BEACON_ENABLED = true;
            memwrite(location.beacon_enabled, BEACON_ENABLED);
        }

        // Turn off periodic beacon
        if (x == 16) {
            if (!BEACON_ENABLED) {
            sendresponse(SUCCESS);
            return;
            }

            sendresponse(SUCCESS);
            BEACON_ENABLED = false;
            memwrite(location.beacon_enabled, BEACON_ENABLED);
        }
        
        // Trigger beacon
        if (x == 17) {

            sendresponse(SUCCESS);

            digitalWrite(BEACON, !HIGH); 
            delay(100); 
            digitalWrite(BEACON, !LOW); 
            delay(50);
            
            digitalWrite(BEACON, !HIGH); 
            delay(100); 
            digitalWrite(BEACON, !LOW); 
            delay(50);
            
        }

        // Turn off the primary indefinitely 
        // TODO: Put secondary on PSM
        if (x == 18) {
            
            sendresponse(SUCCESS);
            
            // Turn off the primary
            shutdown();
        }
        
        // Enable sending acknowledgements 
        if (x == 19) {
            if (SEND_ACK) {
            sendresponse(SUCCESS);
            return;
            }
            
            SEND_ACK = true;
            memwrite(location.send_ack, SEND_ACK);
        }
        
        // Disable sending acknowledgements 
        if (x == 20) {
            if (!SEND_ACK) {
            sendresponse(SUCCESS);
            return;
            }
            
            SEND_ACK = false;
            memwrite(location.send_ack, SEND_ACK);
        }
        
        // Format EEPROM
        if (x == 21) {
            sendresponse(SUCCESS);
            memformat();
        }
        
        // Diagnostics test
        if (x == 22) {

            // Send acknowledgment
            sendresponseforced(SUCCESS);

            // Trigger OFF MOSFET
            trigger(0x02);
            
            tws_delay(500);

            // Trigger ON MOSFET
            trigger(0x01);

            // Test beacon/LED
            blinkmode(0x03, 4);
        }

        /*
            Sentinence control
        */
        
        // Sentinence on
        else if (x == 30) {
            setsentinence(0x01);
        }

        // Sentinence off
        else if (x == 31) {
            setsentinence(0x02);
        } 

        // Heartbeat (kick the dog)
        else if (x == 32) {
            setsentinence(0x03);
        }   

        // Interval base interval control (10 options)
        else if (x >= 40 && x <= 49) {
            selector = x - 40; 

            if (!CONFIG_UNLOCKED) sendresponse(CONFIG_LOCKED_ERROR);
        
            /*
            Set the base interval.
            The range for selector index is 0 to 6.

            The options correspond to:
            1 sec, 5 sec, 10 sec, 15 sec, 30 sec, 1 min, 1 min 30 seconds
            */
            uint8_t options[] = { 1, 5, 10, 15, 30, 60, 90 };

            // Check if the selector is out of range (the length of the options array).
            if (selector < sizeof (options) / sizeof (options[0])) {
            
                // Set sentinence interval base
                if (INTERVAL_SET_MODE == 0) {
                    sendresponse(SUCCESS);
                    SNTL_INTERVAL_BASE = options[selector];
                    memwrite(location.sentinence_base, SNTL_INTERVAL_BASE);
                    // sendresponse(memread(location.sentinence_base));
                }
                
                // Set power-saving interval base
                else if (INTERVAL_SET_MODE == 1) {
                    sendresponse(SUCCESS);
                    PSM_INTERVAL_BASE = options[selector];
                    memwrite(location.psm_base, PSM_INTERVAL_BASE);
                    // sendresponse(memread(location.psm_base));
                }
                
                // Set beacon interval base
                else if (INTERVAL_SET_MODE == 2) {
                    sendresponse(SUCCESS);
                    BCN_INTERVAL_BASE = options[selector];
                }
            }
            else sendresponse(ERROR);
        }

        // Interval duration control (20 levels)
        else if (x >= 50 && x <= 69) { 
            selector = x - 50;

            if (!CONFIG_UNLOCKED) sendresponse(CONFIG_LOCKED_ERROR);
            else sendresponse(SUCCESS);

            /*
            Set the interval multiplier.
            The  range for the multiplier is 1 to 20.
            */

            // Set sentinence interval multiplier
            if (INTERVAL_SET_MODE == 0) {
                sendresponse(SUCCESS);
                SNTL_INTERVAL_MULTIPLIER = (uint8_t) selector + 1;
                memwrite(location.sentinence_multiplier, SNTL_INTERVAL_MULTIPLIER);
                // sendresponse(memread(location.sentinence_multiplier));
            }
            
            // Set power-saving interval multiplier
            else if (INTERVAL_SET_MODE == 1) {
                sendresponse(SUCCESS);
                PSM_INTERVAL_MULTIPLIER = (uint8_t) selector + 1;
                memwrite(location.psm_multiplier, PSM_INTERVAL_MULTIPLIER);
                // sendresponse(memread(location.psm_multiplier));
            }
            
            // Set beacon interval multiplier
            else if (INTERVAL_SET_MODE == 2) {
                sendresponse(SUCCESS);
                BCN_INTERVAL_MULTIPLIER = (uint8_t) selector + 1;
            }
        }

        // Read EEPROM contents
        else if (x >= 70 && x <= 99) { 
            selector = x - 70;

            // Compute location index
            uint16_t MEMLOC = selector;

            // Read data from EEPROM
            uint16_t data = memread(MEMLOC);

            // Send response
            sendresponseforced(data);
        }

        /*
        Set beacon mode
        0 - Periodic beacon based on BCN time values
        1 - Sentinence indicator
        */
        else if (x >= 100 && x <= 103) { 
            sendresponse(SUCCESS);
            selector = x - 100;
            BEACON_MODE = selector;
        }
        else {
        sendresponse(INVALID_OPTION);
        }
        
        // TinyWire library needs this to detect the end of an incoming message
        TinyWireS_stop_check();
    }
    }

    /*
    WDT shield
    */
    void shield(int duration, callback_t_func function) {

    wdtduration(duration);

    // Call the function/scope/block
    function();

    // Disable WDT
    wdt(0x02);
    }

    int last_i2c_turnon_timestamp = 0;
    int last_beacon_timestamp = 0;

    int led_state = LOW;

    void loop() {
    
    shield(WDTO_4S, []() { 

        /*
        Ensure I2C is working by resetting I2C every 30 minutes
        This ocassionally causes I2C communication failures.
        */
        if (millis() - last_i2c_turnon_timestamp >= 30 * 60 * 1000) {
        i2cstate(0x01);
        last_i2c_turnon_timestamp = millis();
        }

        // TinyWire library needs this to detect end of an incoming message
        TinyWireS_stop_check();


        // Check if the reboot condition is met. The bytes need to be converted to integers before comparisions
        PRIMARY_REBOOT_PENDING = SENTINENCE_ENABLED && uint16_t(millis() / 1000) - uint16_t(sentinence_start_timestamp) > uint16_t(SNTL_INTERVAL_BASE) * uint16_t(SNTL_INTERVAL_MULTIPLIER);
        
        if (BEACON_MODE == 0) {
        digitalWrite(BEACON, !SENTINENCE_ENABLED);
        }
        else if (BEACON_MODE == 1) {
        BEACON_TRIGGER_PENDING = BEACON_ENABLED && (uint16_t(millis() / 1000) - uint16_t(last_beacon_timestamp) >= uint16_t(BCN_INTERVAL_BASE) * uint16_t(BCN_INTERVAL_MULTIPLIER));
        }
        else if (BEACON_MODE == 2) {
        BEACON_TRIGGER_PENDING = uint16_t(millis() / 1000) - uint16_t(last_beacon_timestamp) >= uint16_t() * uint16_t(1);
        }

        // Audio chirp
        else if (BEACON_MODE == 3) {
        BEACON_TRIGGER_PENDING = uint16_t(millis() / 1000) - uint16_t(last_beacon_timestamp) >= uint16_t(BCN_INTERVAL_BASE) * uint16_t(BCN_INTERVAL_MULTIPLIER);
        }

        // Trigger audible beacon/LED if due/requested
        if (BEACON_MODE == 1 && BEACON_TRIGGER_PENDING) {
        last_beacon_timestamp = millis() / 1000;

        // Reset variable
        BEACON_TRIGGER_PENDING = false;

        digitalWrite(BEACON, !HIGH); 
        delay(80); 
        digitalWrite(BEACON, !LOW); 
        delay(50);
        
        digitalWrite(BEACON, !HIGH); 
        delay(40); 
        digitalWrite(BEACON, !LOW); 
        }

        if (BEACON_MODE == 2 && BEACON_TRIGGER_PENDING) {
        last_beacon_timestamp = millis() / 1000;

        // Reset variable
        BEACON_TRIGGER_PENDING = false;

        led_state = !led_state;
        digitalWrite(BEACON, led_state); 
        }

        if (BEACON_MODE == 3 && BEACON_TRIGGER_PENDING) {
        last_beacon_timestamp = millis() / 1000;

        // Reset variable
        BEACON_TRIGGER_PENDING = false;
        
        digitalWrite(BEACON, !HIGH); 
        delay(150); 
        digitalWrite(BEACON, !LOW); 
        }

        // Check if a reboot is pending on the primary microcontroller
        if (PRIMARY_REBOOT_PENDING) {

        // Turn off sentinence
        SENTINENCE_ENABLED = false;
        sentinence_start_timestamp = millis() / 1000;

        // Lock Sentinel configuration
        CONFIG_UNLOCKED = false;

        // Increment the primary fault counter
        uint8_t faults = memread(location.primary_fault) + 1;
        memwrite(location.primary_fault, faults);
        
        // Reboot the primary
        reboot(0x02);
        
        // Reset state variable
        PRIMARY_REBOOT_PENDING = false;
        
        // // Reboot the secondary
        // reboot(0x01);
        }

        // Put sentinel to sleep (current drops from 1 mA to 0 mA)
        if (USE_PSM) sleep(WDTO_2S);
    }); 
    
    // 10 ms delay (Required; do not change)
    delay(10);
    
    }

    void setup() {

    shield(WDTO_4S, []() { 
    
        // If the device is waking up from PSM sleep
        if (USE_PSM && memread(location.psm_state_flag) == 1) PSM_WAKE_FLAG = true;
        
        if (USE_PSM && PSM_WAKE_FLAG) {

        // Restore time tracker variables
        sentinence_start_timestamp = memread(location.time_tracker);
        
        // Reset variables
        memwrite(location.psm_state_flag, 0);
        memwrite(location.time_tracker, 0);
        }

        // Format EEPROM if first run
        if (memread(location.init) != 0x0F) memformat();

        // Fetch data from EEPROM
        memfetch();

        // Set pinModes
        pinMode(MOSFET_ON, OUTPUT);
        pinMode(MOSFET_OFF, OUTPUT);
        pinMode(BEACON, INPUT);

        // Set beacon pin to HIGH (connected to P-MOSFET)
        digitalWrite(BEACON, !LOW);
        
        // De-trigger MOSFETs (connected to P-MOSFETss)
        digitalWrite(MOSFET_ON, !LOW);
        digitalWrite(MOSFET_OFF, !LOW);

        // If the device is turning on from a cold state
        if (!USE_PSM || (USE_PSM && !PSM_WAKE_FLAG)) {
        
        // Turn on primary by default
        // blinkmode(0x03, 2);
        trigger(0x01);
        // blinkmode(0x03, 3);
        }

        // If the device is waking up from a PSM sleep state
        else if (USE_PSM && PSM_WAKE_FLAG) {
        
        // blinkmode(0x03, 2);
        i2cstate(0x01);
        tws_delay(1000);
        blinkmode(0x03, 2);
        }

        // Check if the microcontroller was reset by the Watchdog Timer
        if (MCUSR & (1 << WDRF)) {
        
        // Clear the Watchdog Reset Flag
        MCUSR &= ~(1 << WDRF);
        
        blinkmode(0x03, 5);
        
        // // Increment the fault counter
        // uint8_t faults = memread(location.secondary_fault) + 1;
        // memwrite(location.secondary_fault, faults);
        }
        else if (MCUSR & (1 << PORF)) {
        
        // Clear the Watchdog Reset Flag
        MCUSR &= ~(1 << WDRF);

        blinkmode(0x03, 2);
        }
        // else if (MCUSR & (1 << BORF)) {
        
        //   // Clear the Watchdog Reset Flag
        //   MCUSR &= ~(1 << WDRF);

        //   blinkmode(0x03, 6);
        // }

    });

    // // Enable WDT
    // wdtduration(WDTO_2S);
    // delay(4000);
    }
#endif