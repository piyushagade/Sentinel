#include "platform.h"

#if GB_PLATFORM_IS_SENTINEL_V7
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
        3. Ability to programatically rb ATTINY85 using the RST pin

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

    // Utility functions
    #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
    #define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

    // Sentinel firmware meta data
    #define FIRMWAREVERSION 2.10
    #define FIRMWAREDATE 0x0804

    // Define addresses and pins
    #define MOSFET_ON PB3
    #define MOSFET_OFF PB4
    #define BEACON PB1
    #define SDA PB0
    #define SCL PB2

    // Define error codes
    #define SUCCESS 0
    #define ERROR 1
    #define INVALIDACTION 2
    #define PINGRESPONSE 3
    #define CONFIGLOCKEDERROR 4

    // State variables
    bool USE_PSM = false;
    bool SNTL_ENABLED = false;
    bool PRIMARYREBOOTPENDING = false;
    bool PRIMARY_OFF = false;
    bool I2C_ENABLED = false;
    bool SENTINENCE_ENABLED = false;
    ;
    bool CONFIGUNLOCKED = false;
    bool PSM_WAKE_FLAG = false;
    bool PB1_MODE = INPUT;
    uint8_t INTERVAL_SET_MODE = 0;

    // Default sntl interval is 2 minutes
    uint8_t SNTL_INTERVAL_BASE = 15;
    uint8_t SNTL_INTERVAL_MULTIPLIER = 8;
    uint8_t PSM_INTERVAL_BASE = 15;
    uint8_t PSM_INTERVAL_MULTIPLIER = 8;
    uint8_t BCN_INTERVAL_BASE = 5;
    uint8_t BCN_INTERVAL_MULTIPLIER = 2;

    // Sentinel I2C address
    uint8_t I2CADDRESS = 9;

    // Time trackers (unit: seconds)
    uint16_t sentinence_start_timestamp = millis() / 1000;

    // Debug
    bool DIAGNOSTICS = false;
    bool flag = false;

    // Function declarations
    void trigger(int);
    void blinkbits(byte);
    void wdt(int);
    void sendresponse(uint16_t);
    void setsentinence(int);
    void rb(int);
    void blinkmode(int, int);
    void i2clistener(int);

    /*
    System sleep
    The system wakes up when watchdog is timed out.
    */
    void sleep(int duration)
    {

        /*
        https://www.gammon.com.au/forum/?id=11497, Sketch H
        https://github.com/combs/Arduino-Watchdog-Handler
        https://www.hackster.io/Itverkx/sleep-at-tiny-371f04
        */

        // Write PSM state flag
        memwrite(8, 1);

        // Write time tracker value
        memwrite(9, millis() - sentinence_start_timestamp);

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
    void wdtduration(int duration)
    {
        if (duration >= WDTO_15MS)
            wdt_enable(duration);
        else if (duration <= -1)
            wdt(0x02);
    }
    void wdt(int state)
    {

        // Enable WDT
        if (state == 0x01)
        {
            wdt_enable(WDTO_4S);
        }

        // Disable WDT
        else if (state == 0x02)
        {
            MCUSR = 0x0;
            wdt_disable();
        }

        // Initialize WDT
        else if (state == 0x03)
        {
            MCUSR = 0x0;
            wdt_disable();
        }

        // Reset WDT
        else if (state == 0x04)
        {
            wdt_reset();
        }
    }

    /*
    Send two byte response to primary
    */
    void sendresponse(uint16_t response)
    {

        // Turn WDT on
        wdt(0x01);

        // Send low byte
        Wire.write((byte)response);
        delay(50);

        // Send high byte
        Wire.write((byte)(response >>= 8));
        delay(50);

        // Turn WDT off
        wdt(0x02);
    }

    /*
    Set sntl state
    */
    void setsentinence(int state)
    {
        if (state == 0x01 && !SENTINENCE_ENABLED)
        {
            SENTINENCE_ENABLED = true;
            sentinence_start_timestamp = millis() / 1000;

            // Send ack to primary
            sendresponse(SUCCESS);

            // Lock Sentinel configuration
            CONFIGUNLOCKED = false;

            flag = false;
        }
        else if (state == 0x02 && SENTINENCE_ENABLED)
        {
            SENTINENCE_ENABLED = false;
            sentinence_start_timestamp = millis() / 1000;

            // Lock Sentinel configuration
            CONFIGUNLOCKED = false;

            // Send ack to primary
            sendresponse(SUCCESS);
        }
        else if (state == 0x03 && SENTINENCE_ENABLED)
        {
            sentinence_start_timestamp = millis() / 1000;

            // Send ack to primary
            sendresponse(SUCCESS);

            // Lock Sentinel configuration
            CONFIGUNLOCKED = false;
        }

        // Send ack to primary
        else
            sendresponse(INVALIDACTION);
    }

    /*
    Reboot the primary microcontroller
    Inverted logic due to P-channel MOSFETs
    */

    void rb(int device)
    {

        // Primary
        if (device == 0x02)
        {

            // Wait for 1 seconds
            delay(1000);

            // Turn off the primary
            trigger(0x02);

            // A 500 ms delay
            tws_delay(500);

            // Turn on the primary
            trigger(0x01);
        }

        // Secondary
        else if (device == 0x01)
        {

            // Force reset
            wdt_enable(WDTO_15MS);
            while (true)
                delay(1);
        }
    }

    /*
    Set I2C state and listener
    */

    void i2cstate(int state)
    {

        // Enable I2C and set up ISR
        if (state == 0x01)
        {
            tws_delay(1500);
            Wire.begin(I2CADDRESS);
            I2C_ENABLED = true;
            tws_delay(10);
            Wire.onReceive(i2clistener);
        }

        // Disable I2C
        else if (state == 0x02)
        {
            pinMode(SDA, INPUT);
            digitalWrite(SDA, LOW);
            pinMode(SCL, INPUT);
            digitalWrite(SCL, LOW);
            I2C_ENABLED = false;
            tws_delay(100);
        }
    }

    /*
    Trigger On/Off MOSFETs
    */
    void trigger(int state)
    {

        if (state == 0x01)
        {
            PRIMARY_OFF = false;

            digitalWrite(MOSFET_ON, !LOW);

            digitalWrite(MOSFET_ON, !HIGH);
            tws_delay(100);
            digitalWrite(MOSFET_ON, !LOW);

            // Enable I2C and set up ISR
            i2cstate(0x01);
        }

        else if (state == 0x02)
        {
            PRIMARY_OFF = true;

            digitalWrite(MOSFET_OFF, !LOW);

            digitalWrite(MOSFET_OFF, !HIGH);
            tws_delay(100);
            digitalWrite(MOSFET_OFF, !LOW);

            // Disable I2C
            i2cstate(0x02);
        }
    }

    /*
    Blink bits
    */
    void blinkbits(byte b)
    {
        for (int i = 0; i < 8; i++)
        {
            if (bitRead(b, i) == 1)
            {
                analogWrite(BEACON, 1023);
                delay(1000);
            }
            else
            {
                analogWrite(BEACON, 768);
                delay(500);
            }
        }
        digitalWrite(BEACON, LOW);
    }

    /*
    Blink function
    */
    void blinkmode(int mode, int times)
    {

        // Set pin as output
        pinMode(BEACON, OUTPUT);

        int on, off;

        // Slow
        if (mode == 0x01)
        {
            tws_delay(100);
            on = 1000;
            off = 1000;
        }

        // Fast
        else if (mode == 0x02)
        {
            on = 500;
            off = 500;
        }

        // Blink
        else if (mode == 0x03)
        {
            on = 100;
            off = 250;
        }

        // Long
        else if (mode == 0x04)
        {
            on = 3000;
            off = 1000;
        }

        else
        {
            on = 200;
            off = 2000;
        }

        tws_delay(100);
        int counter = 0;
        while (counter++ < times)
        {
            digitalWrite(BEACON, !HIGH);
            tws_delay(on);
            digitalWrite(BEACON, !LOW);
            tws_delay(off);
        }
    }

    /*
    Initialize EEPROM
    */
    void meminit()
    {

        // Write init flag
        EEPROM.write(0, 0x1);

        // Write I2C address
        EEPROM.write(1, I2CADDRESS);

        // Write base interval
        EEPROM.write(2, SNTL_INTERVAL_BASE);

        // Write interval multiplier
        EEPROM.write(3, SNTL_INTERVAL_MULTIPLIER);

        // Write base PSM interval
        EEPROM.write(4, PSM_INTERVAL_BASE);

        // Write interval PSM multiplier
        EEPROM.write(5, PSM_INTERVAL_MULTIPLIER);

        // Write primary's fault counter
        EEPROM.write(6, 0);

        // Write secondary's fault counter
        EEPROM.write(7, 0);

        // Write PSM state flag
        EEPROM.write(8, 0);

        // Write time tracker value
        EEPROM.write(9, 0);
    }

    /*
    Write two bytes to EEPROM
    */
    void memwrite(uint8_t location, uint16_t data)
    {

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
    uint16_t memread(uint8_t location)
    {

        // Write low byte
        byte low = EEPROM.read(location * 2);

        // Write high byte
        byte high = EEPROM.read(location * 2 + 1);

        return (high << 8) | low;
    }

    /*
    Diagnostics; Send values to the primary
    */
    void diagnostics()
    {

        if (!SENTINENCE_ENABLED)
            return;

        // Send right after the Sentinel is enabled
        if (!flag && !PRIMARYREBOOTPENDING)
        {

            sendresponse(9999);
            sendresponse(int(millis() / 1000) - int(sentinence_start_timestamp) > int(SNTL_INTERVAL_BASE) * int(SNTL_INTERVAL_MULTIPLIER));

            flag = true;
        }

        // Send if it's time for reboot
        if (PRIMARYREBOOTPENDING)
        {

            sendresponse(7777);
            sendresponse(int(millis() / 1000) - int(sentinence_start_timestamp) > int(SNTL_INTERVAL_BASE) * int(SNTL_INTERVAL_MULTIPLIER));
        }
    }

    /*
    Watchdog Interrupt Service
    This is executed when the watchdog timed out
    */
    ISR(WDT_vect)
    {
        wdt_disable();
    }

    /*
    Read incoming I2C commands
    */
    void i2clistener(int byteCount)
    {

        // Do no accept new commands if a rb is pending
        if (PRIMARYREBOOTPENDING)
            return;

        int elapsed = 0;
        while (elapsed++ <= 2000 * 1000 && !Wire.available())
            delayMicroseconds(10);

        int selector = 0;
        while (Wire.available())
        {

            // Convert received byte to decimal (range is 0 to 127)
            byte b = Wire.read();
            char s[4];
            itoa(b, s, 10);
            int x = atoi(s);

            // Ping
            if (x == 0)
            {
                sendresponse(PINGRESPONSE);
            };

            // Unlock configuration
            if (x == 1)
            {
                CONFIGUNLOCKED = true;
                sendresponse(SUCCESS);
            }

            // Lock configuration
            if (x == 2)
            {
                CONFIGUNLOCKED = false;
                sendresponse(SUCCESS);
            }

            // Firmware version
            if (x == 3)
            {
                sendresponse(FIRMWAREVERSION * 100);
            }

            // Firmware date
            if (x == 4)
            {
                sendresponse(FIRMWAREDATE);
            }

            // Power off the primary indefinitely (will restart when secondary self reboots from WDT in an hour)
            if (x == 5)
            {
            }

            // Reboot the primary
            if (x == 6)
            {
                PRIMARYREBOOTPENDING = true;
                rb(0x02);
                PRIMARYREBOOTPENDING = false;
            }

            // Reset fault counters
            if (x == 7)
            {
                memwrite(6, 0);
                memwrite(7, 0);
                sendresponse(SUCCESS);
            }

            // Primary's fault counter
            if (x == 8)
            {
                sendresponse(memread(6));
            }

            // Secondary's fault counter
            if (x == 9)
            {
                sendresponse(memread(7));
            }

            // Put secondary to sleep
            if (x == 10)
            {
            }

            // Set interval setting mode to sentinence interval
            if (x == 11)
            {
                if (!CONFIGUNLOCKED)
                    sendresponse(CONFIGLOCKEDERROR);
                INTERVAL_SET_MODE = 0;
            }

            // Set interval setting mode to power saving interval
            if (x == 12)
            {
                if (!CONFIGUNLOCKED)
                    sendresponse(CONFIGLOCKEDERROR);
                INTERVAL_SET_MODE = 1;
            }

            // Reboot the secondary microcontroller
            if (x == 13)
            {
                rb(0x01);
                PRIMARYREBOOTPENDING = false;
            };

            // Set interval setting mode to beacon interval
            if (x == 14)
            {
                if (!CONFIGUNLOCKED)
                    sendresponse(CONFIGLOCKEDERROR);
                INTERVAL_SET_MODE = 2;
            }

            // Sentenence control

            // Sentinence on
            else if (x == 30)
            {
                setsentinence(0x01);
            }

            // Sentinence off
            else if (x == 31)
            {
                setsentinence(0x02);
            }

            // Heartbeat (kick the dog)
            else if (x == 32)
            {
                setsentinence(0x03);
            }

            // Interval base interval control (10 options)
            else if (x >= 40 && x <= 49)
            {
                selector = x - 40;

                if (!CONFIGUNLOCKED)
                    sendresponse(CONFIGLOCKEDERROR);

                /*
                Set the base interval.
                The range for selector index is 0 to 6.

                The options correspond to:
                1 sec, 5 sec, 10 sec, 15 sec, 30 sec, 1 min, 1 min 30 seconds
                */
                uint8_t options[] = {1, 5, 10, 15, 30, 60, 90};

                // Check if the selector is out of range (the length of the options array).
                if (selector < sizeof(options) / sizeof(options[0]))
                {

                    // Set sentinence interval base
                    if (INTERVAL_SET_MODE == 0)
                    {
                        SNTL_INTERVAL_BASE = options[selector];
                        memwrite(2, SNTL_INTERVAL_BASE);
                        sendresponse(memread(2));
                    }

                    // Set power-saving interval base
                    else if (INTERVAL_SET_MODE == 1)
                    {
                        PSM_INTERVAL_BASE = options[selector];
                        memwrite(4, PSM_INTERVAL_BASE);
                        sendresponse(memread(4));
                    }

                    // Set beacon interval base
                    else if (INTERVAL_SET_MODE == 2)
                    {
                        BCN_INTERVAL_BASE = options[selector];
                        sendresponse(SUCCESS);
                    }
                }
                else
                    sendresponse(ERROR);
            }

            // Interval duration control (20 levels)
            else if (x >= 50 && x <= 69)
            {
                selector = x - 50;

                if (!CONFIGUNLOCKED)
                    sendresponse(CONFIGLOCKEDERROR);

                /*
                Set the interval multiplier.
                The  range for the multiplier is 1 to 20.
                */

                // Set sentinence interval multiplier
                if (INTERVAL_SET_MODE == 0)
                {
                    SNTL_INTERVAL_MULTIPLIER = (uint8_t)selector + 1;
                    memwrite(3, SNTL_INTERVAL_MULTIPLIER);
                    sendresponse(memread(3));
                }

                // Set power-saving interval multiplier
                else if (INTERVAL_SET_MODE == 1)
                {
                    PSM_INTERVAL_MULTIPLIER = (uint8_t)selector + 1;
                    memwrite(5, PSM_INTERVAL_MULTIPLIER);
                    sendresponse(memread(5));
                }

                // Set beacon interval multiplier
                else if (INTERVAL_SET_MODE == 2)
                {
                    BCN_INTERVAL_MULTIPLIER = (uint8_t)selector + 1;
                    sendresponse(SUCCESS);
                }
            }

            // TinyWire library needs this to detect the end of an incoming message
            TinyWireS_stop_check();
        }
    }

    int last_i2c_turnon_timestamp = 0;
    int last_beacon_timestamp = 0;

    void loop()
    {

        // Ensure I2C is working
        if (millis() - last_i2c_turnon_timestamp >= 60000)
        {
            i2cstate(0x01);
            last_i2c_turnon_timestamp = millis();
        }

        // Audible beacon
        // if (uint16_t(millis()) - uint16_t(last_beacon_timestamp) >= uint16_t(BCN_INTERVAL_BASE) * uint16_t(BCN_INTERVAL_MULTIPLIER) * 1000) {
        if (uint16_t(millis() / 1000) - uint16_t(last_beacon_timestamp) >= 10)
        {
            last_beacon_timestamp = millis() / 1000;
            digitalWrite(BEACON, !HIGH);
            delay(100);
            digitalWrite(BEACON, !LOW);
            delay(50);

            digitalWrite(BEACON, !HIGH);
            delay(100);
            digitalWrite(BEACON, !LOW);
            delay(50);
        }

        // TinyWire library needs this to detect end of an incoming message
        TinyWireS_stop_check();

        // Check if the reboot condition is met. The bytes need to be converted to integers before comparisions
        PRIMARYREBOOTPENDING = SENTINENCE_ENABLED && uint16_t(millis() / 1000) - uint16_t(sentinence_start_timestamp) > uint16_t(SNTL_INTERVAL_BASE) * uint16_t(SNTL_INTERVAL_MULTIPLIER);

        // Print diagnostics variables
        if (DIAGNOSTICS)
            diagnostics();

        // Check if a beat has not been received in the last SNTL_INTERVAL_BASE * SNTL_INTERVAL_MULTIPLIER microseconds
        if (PRIMARYREBOOTPENDING)
        {

            // Turn off sentinence
            SENTINENCE_ENABLED = false;
            sentinence_start_timestamp = millis() / 1000;

            // Lock Sentinel configuration
            CONFIGUNLOCKED = false;

            // Increment the primary fault counter
            uint8_t faults = memread(6) + 1;
            memwrite(6, faults);

            // Reboot the primary
            rb(0x02);

            // Reset state variable
            PRIMARYREBOOTPENDING = false;

            // Reboot the secondary
            rb(0x01);
        }

        // Put sentinel to sleep (current drops from 1 mA to 0 mA)
        if (USE_PSM)
            sleep(WDTO_2S);

        // 10 ms delay
        delay(10);
    }

    void setup()
    {

        // If the device is waking up from PSM sleep
        if (USE_PSM && memread(8) == 1)
            PSM_WAKE_FLAG = true;

        if (USE_PSM && PSM_WAKE_FLAG)
        {

            // Restore time tracker variables
            sentinence_start_timestamp = memread(9);

            // Reset variables
            memwrite(8, 0);
            memwrite(9, 0);
        }

        // Initialize WDT
        wdt(0x03);

        // Initialize EEPROM if first run
        if (memread(0) == 0xFF)
            meminit();

        // Use the last used configuration/variables
        else if (USE_PSM)
        {

            SNTL_INTERVAL_BASE = memread(2);
            SNTL_INTERVAL_MULTIPLIER = memread(3);

            PSM_INTERVAL_BASE = memread(4);
            PSM_INTERVAL_MULTIPLIER = memread(5);
        }

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
        if (!USE_PSM || (USE_PSM && !PSM_WAKE_FLAG))
        {

            // Turn on primary by default in 1.5 seconds
            tws_delay(1500);
            rb(0x02);
            blinkmode(0x03, 3);
        }

        // If the device is waking up from a PSM sleep state
        else if (USE_PSM)
        {
            i2cstate(0x01);
            blinkmode(0x03, 2);
        }
    }
#endif