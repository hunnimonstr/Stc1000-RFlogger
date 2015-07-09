/* Beta harness for Rf receiver to listen and log data received from stc100+ devices 
 * with a 433htz transmitter wired to the reprogramming header pins.
 * This code based mainly on the WeatherSensorWH2 project by Mr Luc Small.  
 * Luc did all the hard work, this is based on his example code and code found in the arduino ide examples
 * once the weather station library has been installed
 * github.com/lucsmall/WH2-Weather-Sensor-Library-for-Arduino
 *  GNU liscence http://www.fsf.org/ use freely for personal or free distribution
*  no ideas to be used in commercial code.
 *
 *    Version 1.0 Beta version
 *
 * OVERVIEW
 * a stand alone data logger to record timestamped data records to a csv file stored on a fat32
* fat16  formatted sd card.
* Data record will contain:
* 1) Timestamp
* 2) eye friendly/text  date time
* 3) Temperature in the units your stc is operating with C in my case
* 4) the humidity data field (may contain data) with subsiquent updates of the stc1000+
* other record info metadata and metrics can be outputted too the commands are commented * out using //
* 
* INTERFACE
* 
* 5 x led interface 
* 1)yellow Reception indicator,
* 2) green record + write indicator,
* 3) red1, error 
* 4) red2,error 
* 5) red3 error  (or any colours you prefer 
 * Normal/optimum operation
 *     when turned on all leds will light and the reds wil then light in 1,2,3 order to validate lamps and wirring
 *     the yellow led will light up while rtc checks are carried out, the green led will light while card and file system checks 
 *     are carried out, then when running the yellow will indicate rf activity, and the green good record aquisition and recording 
 *     So normal operation is
 *     No Red Leds
 *     Yellow flickering with Rf reception
 *     Green Flashing Maybe too fast to notice with record wtites may add a delay.
 *
 * Error Conditions
 * red1 led lit no good record received in the last 5 minutes
 * red1 and red2 lit no good record for 15 mins
 * red1 red2 red3 lit no good record for 1 hour
 * red1 flashing with green flashing - No SDcard found or not recognising file system must be
  * fat16 or fat32 and some cards can be iffy
 * red 1 flashing with yellow flashing - RTC error 1 time not set.
 * green on red1 flashing file open failure
 *
* If an error condition is indicated plug into a puter with the arduino V1.065 ide installed and *
* open the serial monitor for debug data and info..
 * Some soldering to connect resistors inline may be necessary tho diddy breadboards and hot glue can work ;)
 * http://www.ebay.co.uk/itm/like/121599131538?limghlpsr=true&hlpv=2&ops=true&viphx=1&hlpht=true&lpid=108&chn=ps&device=c&rlsatarget=&adtype=pla&crdt=0&ff3=1&ff11=ICEP3.0.0-L&ff12=67&ff13=80&ff14=108&ff19=0
 *
 *
 * Hardware requirements
* Rf Transmitter added to the STC1000+ header pins
* 1 x rf receiver connected to digital pin #2
 * 1 X SD Card interface board
 * 1 X RTC (real time clock) with backup power battery to keep time when no aruino power
 * 1 x arduino board
 * 1 x yellow led
 * 1 x green led
 * 3 x red led (blue)
 * 5 x resistors value  1k ohm for yellow led 470k for red and green leds
 * a handful of sacraficial prototype wires
 * 2 x 20cm long single wire annenna (ive used 2 x 20cm jumper leads)
 * a box
 * a psu 5-12v dc power supply 500 ma should be fine??  will check..
 *
 * I have opted for  £4 sdcard/rtc shield via ebay similar to
 * http://www.adafruit.com/products/1141
 *
 * if you to go for the shield option just plug it into the board.
 * if you have 2 seperate h/w bits
 * https://www.arduino.cc/en/reference/wire
 * RTC - DS1307 type rtc used..
 * The Realtime clock (rtc) uses the I2C coms so needs connecting to
 * SDA      Arduino gpio A4 or dedicated pin above aref other side of the board (possible conflict with warninG led's?? (dATA LINE)
 * SCL      Arduino gpio A5 or dedicated pin (possible conflict with warninG led's??  (CLOCK SPEED)
 * +5v may be marked VCC     TO arduino +5V
 * GND                       To arduino common ground
 * https://www.arduino.cc/en/Reference/SPI
 * SD cards use the SPI interface, 4 X gpio pins + +ve5v and common ground
 * MISO     Master in slave out  connect to arduino 12
 * MOSI     Master out slave In  connect to arduino  11
 * SCK      serial clock connect to arduino 13
 * SS       slave select 10
 * (additional SPI devices can be added using the common miso mosi and sck pins needing only a distinct SS connection)
 * http://www.ladyada.net/learn/arduino/lesson3.html
 * Leds are connected +ve (longer leg) into Gpio pin sockets A1-5 (may change,,,)
 * Don't buy super brite leds really cheap low power ones are all thats needed unless you plan on sitting the logger under industrial lighting.
*  led Pin Assignment
 * yellow A2
 * green A3
 * red1 d3
 * red2 d4
 * red3 d5  connect the -Ve shorter leg to its resistor and then to common ground (solder? or mini breadboard + hot glue gun??)
 *
 * The rf Rx module needs +5v cmomon ground and a connection from Digital Pin 2 D2 to one of its 2 centre data pins.
 *
 *
 *
 */

#include <Wire.h>
#include "RTClib.h"
#include <SPI.h>
#include <SD.h>
#include <WeatherSensorWH2.h>

#define RF_IN 2
#define LED_PACKET A2
#define LED_ACTIVITY A3
#define LED_RED1 3    
#define LED_RED2 4
#define LED_RED3 5
#define REC_CHK 0
#define DISK_ERR 1
#define CLK_ERR 2
#define FILE_ERR 3
#define RST_RED 4

//global vars and function declarations
const int chipSelect = 10;                                  // Chip Select(Slave Select) SD card interface 4, 8, OR 10 ON OTHER BARDS check docs
RTC_DS1307 rtc;                                             // Real Time Clock Instance
volatile byte got_interval = 0;                             // received Rf signal flag
volatile byte interval = 0;                                 // duration of rfsignal??
volatile unsigned long old = 0, packet_count = 0;           // housekeeping vars used for record metrics
volatile unsigned long spacing, now1, average_interval;     
volatile unsigned long errlastgoodrecord=millis();          // record of last good recorord received/written set optimistically
WeatherSensorWH2 STC1000;                                   // The weather Sensore object instance or stc1000 in our case
volatile byte VERBOSE = false;                              // if true every valid received record will be logged Signal repeated 3 X per cycle

void setrtc();                                              // sets the rtc to compile time or user entered time in code
void signalstats();                                         // compiles the crucial spacing stat amongst other packet data
int logrecord();                                            // open file and write the record
void debugprint();                                          // serial output of record and metadats 
void dowarnings(int action);                                //sets led warning status
void Datetimetest();                                        //  code for date time functions display to serial
void ledtest();                                             // light leds for startup test& delay
void clocktest();                                           // validates/sets the clock is running
void sdtest();
ISR(TIMER1_COMPA_vect)// Timer1 synced with the frequency of the RF signal and sets got_signal flag and interval value.
{
  static byte count = 0;
  static byte was_hi = 0;
  if (digitalRead(RF_IN) == HIGH) {
    digitalWrite(LED_ACTIVITY, HIGH);
    count++;
    was_hi = 1;
  } else {
    digitalWrite(LED_ACTIVITY, LOW);
    if (was_hi) {
      was_hi = 0;
      interval = count;
      got_interval = 1;
      count = 0;
    }
  }
}
//****************************************************************************
void setup () {

  Serial.begin(9600);                                                  //start debug coms 
  pinMode(RF_IN, INPUT);                                               // IO PIN SETUP
  pinMode(LED_PACKET, OUTPUT);
  pinMode(LED_ACTIVITY, OUTPUT);
  pinMode(LED_RED1, OUTPUT);
  pinMode(LED_RED2, OUTPUT);
  pinMode(LED_RED3, OUTPUT);
  
  ledtest();                                                            // Test h/w. Rf failure will be evident by a quiet box. 
  clocktest();
  sdtest();
  // interupt initialising the following assignments set up timer1 ISR above
  // think it tunes a time to the frequency to listen on to determin if a start on mesage is braodcast..
  TCCR1A = 0x00;  // still reading up on this.. 00 = sets it to default, 
  TCCR1B = 0x09;  // 0x09 == 1001 == R/W and no prescaler on the timer
  TCCR1C = 0x00;  // set the C register to empty
  OCR1A = 399;    // -value set to compare match against?
  TIMSK1 = 0x02;  //  controls which interrupts the timer can trigger 02 0010 = Compare/Match
  sei();                                                                 // enable interrupts
}

void loop () {
  if (got_interval) {                                               // tests the got interval flag if true set in the interupt on rf change
    STC1000.accept(interval);                                       // Listen? process? 'accept' from the RF input passing the interval generated in ISR interupt
    if (STC1000.acquired()) {                                       // If a valid record is aquiered print it to the log file..
      digitalWrite(LED_PACKET, HIGH);                               // Flash on Green led for record processing durration to indicate activity
      signalstats();                                                // get the signal stats
      if ((VERBOSE) || (spacing > 1500)) {                          // if all records OR last record was over 1500 m/s ago ( may need tuning towards 3000)
        /* OR (||) Logic                                            Do the loggng to file
         *           (Spacng > Test value)   TRUE | FALSE
         *                  ______________________|______           
         *                  VERBOSE = TRUE  | True| True
         *                  VERBOSE = FALSE | True| False
         */
        if (logrecord()!=0){ dowarnings(FILE_ERR);}                 // Call logrecord display file error if logrecord fails
        else {                                                      // record write sucessfull.
             errlastgoodrecord = millis();                          // time last good record read = now
             dowarnings(RST_RED);                                   // reset any missed record warnings
             }                                                      //CLOSE ELSE clause
        debugprint();                                               // SERIAL PRINT THE RECORD AND METADATA
        }                                                           // close of if VERBOSE OR last record was over 1500 m/s 
    }                                                               // close of aquiered
    got_interval = 0;                                               // add delay here to prolong green record accepted/recorded signal flash 
    digitalWrite(LED_PACKET, LOW);                                  // turn off green led to indicate record received and written    
  }                                                                 // close of got interval test  may adjust timings??
  dowarnings(REC_CHK);                                              // check last good record and warn accordingly
}                                                                   // Main loop close
//***************************************************************************************************************************************
// functions

void sdtest(){
    digitalWrite(LED_ACTIVITY,HIGH);                                      // yellow led for rtc test start
   
  // ********************************************************************************
  digitalWrite(LED_PACKET, HIGH);                                       // green led on for sd checks
  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect)) {dowarnings(DISK_ERR );}                   // If card fails Fatal error do disk error
  Serial.println("card initialized.");
  //*******************************************************************************
  digitalWrite(LED_PACKET, LOW);                                        // green led off post sd checks

}

void clocktest(){
  
  #ifdef AVR                                                            //rtc on the wire/i2c bus
    Wire.begin();
  #else                                                                 // else rtc on the wire1/i2c bus
    Wire1.begin();                                                      // Shield I2C pins connect to alt I2C bus on Arduino
  #endif
    rtc.begin();
    if (! rtc.isrunning()) {                                            // If clock nor running/set setting date time
        setrtc();
    }
  if (! rtc.isrunning()) {dowarnings(CLK_ERR );}                        // clock still not up? Fatal Error do clock error
  Datetimetest();                                                       //chuck out verbose clock info via usb/serial for debugging
  digitalWrite(LED_ACTIVITY,LOW);                                       // turn off yellow clock check led
}

void dowarnings(int action){
  /*routine to handle led warnings and resets
   * 
   */
  switch (action) {
  case DISK_ERR:                                                    // Disk error
    Serial.println("Card failed, or not present");
    // Signal Card Error  Flash green led LED_PACKET:
    digitalWrite(LED_ACTIVITY, LOW);
    while (true) {                                                    // flash leds for bad or no card indefinatly
      /*
       * FATAL ERROR cant write data..
       */
      digitalWrite(LED_PACKET, HIGH);                                 // Flash Green led for error condition no/bad sd card
      digitalWrite(LED_RED1, HIGH);                                   // Flash red1 led for for error condition no/bad sd card
      delay(500);
      digitalWrite(LED_PACKET, LOW);                                  // Flash Green led ffor error condition no/bad sd card
      digitalWrite(LED_RED1, LOW);                                    // Flash red1 led for error condition no/bad sd card
      delay(500);
    }

    break;
  case CLK_ERR:                                                       // clock error FATAL!!
    while(true) {                                                     // forever
      digitalWrite(LED_ACTIVITY, HIGH);                               // Flash yellow led for error condition No RTC
      digitalWrite(LED_RED1, HIGH);                                   // Flash red1 led for for error condition No RTC
      delay(333);
      digitalWrite(LED_ACTIVITY, LOW);                                // Flash yellow led ffor error condition No RTC
      digitalWrite(LED_RED1, LOW);                                    // Flash red1 led for error condition no/bad No RTC
      delay(333);
      }
    break;
  case FILE_ERR:
           //Logging error handeling..
                digitalWrite(LED_PACKET, HIGH);                          // Green led for error condition file open error
                while (true) {                                         // flash led for bad or no card indefinatly
                  /*
                   * FAtal Error ?? or is it should it be retried???
                   */
                  digitalWrite(LED_RED1, HIGH);                       // Flash red1 led for for error condition file open error
                  delay(333);
                  digitalWrite(LED_RED1, LOW);                        // Flash red1 led for error condition file open error
                  delay(333);
                  }
   break; 
   case REC_CHK:
   
              if ( (millis() - errlastgoodrecord) > 300000) {                  //5 mins no record
                digitalWrite(LED_RED1, HIGH);
              }
              if ( (millis() - errlastgoodrecord) > 900000) {                   //15 mins no record
                digitalWrite(LED_RED2, HIGH);
              }
            
              if ( (millis() - errlastgoodrecord) > 3600000) {                  // 1 hour no record
                digitalWrite(LED_RED3, HIGH);
            
              }
              break;
  case RST_RED:
            digitalWrite(LED_RED1, LOW);                              // turn off any record error leds
            digitalWrite(LED_RED2, LOW);
            digitalWrite(LED_RED3, LOW);
            break;
  default: 
    // statements
    break;
  }
}

void setrtc(){
  /* Void function to set the rtc module to the time last compiled
   * Or by adjusting the comments to a specific time/date
   * Requires: A Ds1307 clock module initialised as rtc
   * Returns : Nothing 
   * Results: The clock rtc should be set to a valid datetime
   * Exception handleing: None
   */
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));

}


void signalstats(){
  /*Void Function to set global variables used for signal analysis
   * Requires the referenced global variables to be initialised
   * Returns: Nothing
   * Results: packet_count,average_interval, and spacing assignned current values
   * Exception handleing: None
   */

      // determin regularity of clean record reception..
      now1 = millis();
      spacing = now1 - old;
      old = now1;
      packet_count++;
      average_interval = now1 / packet_count;

}




int logrecord(){
/* Function to lofg a record returing an integer 0= ok, 1 = error
 * Requires: Clock rtc FileSystem SD  
 * Returns: 0 for success 1 for error Failure to log.
 * Results: When succesful a data record is written to file when an erro ocours no file record is written
 * exception handleing On: failure to open the file the return value 1 is assigned. 
 */
  DateTime now = rtc.now();                                         // initialise datetimevar to current time for logging may move into file open?
  File dataFile ; 
       if ( dataFile = SD.open("datalog.txt", FILE_WRITE)) {
          // Write validated data record..
          // log time
          dataFile.print(now.unixtime());                        // timestamp
          dataFile.print(",");                                 // comma data delimeter
                                                               // eye friendly date time
          if(now.day() < 10) dataFile.print("0");
          dataFile.print(now.day(), DEC);
          dataFile.print("/");
          if(now.month() < 10) dataFile.print("0");
          dataFile.print(now.month(), DEC);
          dataFile.print("/");
          dataFile.print(now.year(), DEC);
          dataFile.print(" ");
          if(now.hour() < 10) dataFile.print("0");
          dataFile.print(now.hour(), DEC);
          dataFile.print(":");
          if(now.minute() < 10) dataFile.print("0");
          dataFile.print(now.minute(), DEC);
          dataFile.print(":");
          if(now.second() < 10) dataFile.print("0");
          dataFile.print(now.second(), DEC);
          dataFile.print(",");                                  // comma data delimeter
          dataFile.print(STC1000.get_temperature_formatted());  // temperature in degrees C
          dataFile.print(",");
          dataFile.print(STC1000.get_humidity(), DEC);         // humidity If used??
          // The following statements can be uncommented out to log more data..
          // dataFile.print(",");
          //dataFile.print(STC1000.calculate_crc(), HEX);         // crc result
          // dataFile.print(",");
          // dataFile.println((STC1000.valid() ? " OK" : " BAD"));   // text indicator OK or BAD indicating stc1000 record validity
          // dataFile.print(",");
          // dataFile.print("0x");  // text prefix for hex value of sensor id
          //  dataFile.println(STC1000.get_sensor_id(), HEX);
          // dataFile.print(",");
          //  dataFile.print(STC1000.get_temperature(), DEC);    // raw temp data format in 10ths of a degree
          dataFile.println();  //eol to terminate the record.
          dataFile.close();  // close file
          return 0;
          //
          
        }// if the file isn't open, pop up an error:
        else {
          Serial.println("error opening datalog.txt");
          // add file error handling
          return 1;  // error condition to trigger dowarnings(FILE_ERR)
        } // close else

}



void debugprint(){
            /* void function to echo the record written to file to the serial port
             *  Metadata about the record from the received packet is also output 
           *  Requires Clock rtc, and WH2s object STC1000 loaded with a valid record
           *  Returns: nothing
           *  Results: the record and metadata output to the sweial monitor
           *  Exception handeling None
           */
  byte i;                                                           // debug metadata counter
  byte *packet;                                                     
  DateTime now = rtc.now(); 
 
          Serial.print(now.unixtime());                        // timestamp
          Serial.print(", ");                                 // comma data delimeter
          
          if (now.day() < 10) Serial.print("0"); 
          Serial.print(now.day(), DEC);
          Serial.print("/");
          if (now.month() < 10) Serial.print("0");
          Serial.print(now.month(), DEC);
          Serial.print("/");
          Serial.print(now.year(), DEC);
          Serial.print(" ");
          if (now.hour() < 10) Serial.print("0");
          Serial.print(now.hour(), DEC);
          Serial.print(":");
          if (now.minute() < 10) Serial.print("0");
          Serial.print(now.minute(), DEC);
          Serial.print(":");
          if (now.second() < 10) Serial.print("0");
          Serial.print(now.second(), DEC);
          Serial.print(",");                                  // comma data delimeter
          Serial.print(STC1000.get_temperature_formatted());  // temperature in degrees C
          Serial.print(",");
          Serial.println(STC1000.get_humidity(), DEC);         // humidity If used??

          
          
          // serial debug data and metadata

          
          Serial.print("Spacing: ");
          Serial.println(spacing, DEC);
          Serial.print("Packet count: ");
          Serial.println(packet_count, DEC);
          Serial.print("Average spacing: ");
          Serial.println(average_interval, DEC);
          packet = STC1000.get_packet();
          for (i = 0; i < 5; i++) {
            Serial.print("0x");
            Serial.print(packet[i], HEX);
            Serial.print("/");
            Serial.print(packet[i], DEC);
            Serial.print(" ");
          }
          Serial.print("crc: ");
          Serial.print(STC1000.calculate_crc(), HEX);
          Serial.println((STC1000.valid() ? " OK" : " BAD"));
          Serial.print("Sensor ID: 0x");
          Serial.println(STC1000.get_sensor_id(), HEX);
          Serial.print("Humidity: ");
          Serial.print(STC1000.get_humidity(), DEC);
          Serial.println("%");
          Serial.print("Temperature: ");
          Serial.print(STC1000.get_temperature_formatted());
          Serial.print(" C  [");
          Serial.print(STC1000.get_temperature(), DEC);          //*****  Raw temp data in 10ths of a degree  ie 25C = 250
          Serial.println("]");
          Serial.println("--------------");

}


void Datetimetest()
/* Void function to rest the rtc settings by ouputing current time and the date time of 7 days into the future
 *  Requires clock rtc running
 *  Returns nothing
 *  Results The time date test values are displayed in the serial monitor
 */
{ DateTime now = rtc.now();
  if (now.day() < 10) Serial.print("0");
  Serial.print(now.day(), DEC);
  Serial.print('/');
  if (now.month() < 10) Serial.print("0");
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.year(), DEC);
  Serial.print(' ');
  if (now.hour() < 10) Serial.print("0");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  if (now.minute() < 10) Serial.print("0");
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  if (now.second() < 10) Serial.print("0");
  Serial.print(now.second(), DEC);
  Serial.println();

  Serial.print(" since midnight 1/1/1970 = ");
  Serial.print(now.unixtime());
  Serial.print("s = ");
  Serial.print(now.unixtime() / 86400L);
  Serial.println("d");

  // calculate a date which is 7 days and 30 seconds into the future
  DateTime future (now.unixtime() + 7 * 86400L + 30);

  Serial.print(" now + 7d + 30s: ");
  Serial.print(future.year(), DEC);
  Serial.print('/');
  Serial.print(future.month(), DEC);
  Serial.print('/');
  Serial.print(future.day(), DEC);
  Serial.print(' ');
  Serial.print(future.hour(), DEC);
  Serial.print(':');
  Serial.print(future.minute(), DEC);
  Serial.print(':');
  Serial.print(future.second(), DEC);
  Serial.println();

  Serial.println();

}

void ledtest(){
 /* Void funtion to test leds function ok
  *  Requires: Green led on pin a3 Yellow on a2, red1 on d3 red2 on d4 and red3 on d5 as specified above
  *  Returns: nothing
  *  Results in a test of leds all should light then the reds should shut off before running in sequence
  *  for 1 pass 1 on then 2 only on then 3 only on.
  *  then all should shut off
 * 
 */
                                                     
      digitalWrite(LED_PACKET, HIGH);                                 
      digitalWrite(LED_ACTIVITY, HIGH); 
      digitalWrite(LED_RED1, HIGH);
      digitalWrite(LED_RED2, HIGH);
      digitalWrite(LED_RED3, HIGH);
      delay(1000);
      digitalWrite(LED_RED1, LOW);
      digitalWrite(LED_RED2, LOW);
      digitalWrite(LED_RED3, LOW);
      delay(500);
      
      
      digitalWrite(LED_RED1, HIGH);                                  
      delay(1000);
      digitalWrite(LED_RED1, LOW);  
      digitalWrite(LED_RED2,HIGH);
      delay(1000);
       digitalWrite(LED_RED2, LOW);  
      digitalWrite(LED_RED3, HIGH);                                    
    delay(1000); 
      digitalWrite(LED_PACKET, LOW);                                 
      digitalWrite(LED_ACTIVITY, LOW); 
      digitalWrite(LED_RED3, LOW);    
    delay(1000);
    //end of led testing



 
}

