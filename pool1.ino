#include <Wire.h>               // main library for the I2C bus
#include "ds3231.h"             // RTC library
#include <OneWire.h>            // library for the temp sensor
#include <DallasTemperature.h>  // library for the temp sensor
#include <LiquidCrystal_I2C.h>  // library for the LCD I2C display
#include <math.h>

const int pumpPin = 9;          // pin connected to the relay controlling the heating pump

// Settings for the LCD
const int lcdAddress = 0x27;    // LCD I2C serial address
const int en = 2, rw = 1, rs = 0, d4 = 4, d5 = 5, d6 = 6, d7 = 7, bl = 3; // LCD pinout
LiquidCrystal_I2C lcd(lcdAddress, en, rw, rs, d4, d5, d6,d7,bl, POSITIVE);  // Set the LCD I2C address

#define BUFF_MAX 128            // used for formatting of the RTC string 

// DS18B20 Temprature sensors stuff
const int oneWirePin1 = 8;
const int oneWirePin2 = 7;
OneWire oneWire1(oneWirePin1);                     // creating oneWire bus object on pin oneWirePin
OneWire oneWire2(oneWirePin2);  
DallasTemperature sensor1(&oneWire1);             // initialising temp sensors on the oneWire bus object
DallasTemperature sensor2(&oneWire2);
float currentTemp0;                            // used to hold temp data across
float currentTemp1;                            // used to hold temp data across

// delay without delay variables DwD
unsigned long currentMillis;

unsigned long startMillis1;
const unsigned long interval1 = 1000;  //the value is a number of milliseconds, ie 1 second

unsigned long startMillis2;
//unsigned long currentMillis2;
const unsigned long interval2 = 600000;  //the value is a number of milliseconds, ie 10 minuets
unsigned long startMillis3;
//unsigned long currentMillis3;
const unsigned long interval3 = 180000;  //the value is a number of milliseconds, ie 3 minuets

const int startNight = 19;
const int startMorning = 6;

int stat;    // device status
int ctFlag;  // temp flag.  

void setup() {
  //Serial.begin(9600);           // starting serial communication
  Wire.begin();                 // starting I2C bus
  sensor1.begin();              // starting the DS18B20 Temp sensors
  sensor2.begin();              // starting the DS18B20 Temp sensors
  DS3231_init(DS3231_INTCN);    // initialising RTC object

  lcd.begin(16,2);               // initialize the lcd 

  pinMode(pumpPin, OUTPUT);         // set pin connected to the relay to output
  digitalWrite(pumpPin, HIGH);   // make sure pump is off when system start

  startMillis1 = millis();       //initial start time for DwD
  
  //while(!Serial);               // making sure serial communication work
  //Serial.println("Here we go! ");

  lcd.home ();
  lcd.print("Here we go! ");
  delay(5000);
  lcd.clear();

  initStat();
  //Serial.print("System starting Status: ");
  //Serial.println(stat);
  
} // End of setup function

void loop() {

  struct ts t;                    // used to read from the RTC

  currentMillis = millis();       //get the current "time" for DwD
  if (currentMillis - startMillis1 >= interval1)  //test whether the period has elapsed
  {
    DS3231_get(&t);               // Read the time from the RTC module

    writeTime(&t,1);


    getTemp();
    writeTemp(currentTemp0,2);
    writeTemp(currentTemp1,3);

  lcd.setCursor ( 0, 0 );
  lcd.print("stat: ");
  lcd.print(stat);
 
  startMillis1 = currentMillis;  //IMPORTANT to save the start time state.
  }


  if (stat == 0) {                                    // check every 10 min if it's 6am
//      currentMillis2 = millis();       //get the current "time" for DwD
    if (currentMillis - startMillis2 >= interval2)  //test whether the period has elapsed
    {
      digitalWrite(pumpPin, LOW);   // turn pump off

      if (t.hour >= startMorning) {              // if it's after 6am, change status to '2'
        digitalWrite(pumpPin, HIGH);   // turn pump on
        startMillis3 = millis();       //initial start time for DwD     
        stat = 2;
      }  // end if time check
  //Serial.println("Status 0 initiated by stat == 0");    
      startMillis2 = currentMillis;  //IMPORTANT to save the start time state.
    }
  } // End if stat == 0
  else if (stat == 1) {                      // basicly a 10 min timer
    
    digitalWrite(pumpPin, LOW);   // turn pump off
 //   currentMillis2 = millis();                //get the current "time" for DwD
    if (currentMillis - startMillis2 >= interval2)  //test whether the period has elapsed
    {
        digitalWrite(pumpPin, HIGH);   // turn pump on
        startMillis3 = millis();       //initial start time for DwD     
        stat = 2;                      // change to next status
      startMillis2 = currentMillis;  //IMPORTANT to save the start time state.
 //Serial.println("Status 2 initiated by stat == 1");
    }
    
  } // End if stat == 1
  else if (stat == 2) {

    digitalWrite(pumpPin, HIGH);   // turn pump on
    
//    currentMillis3 = millis();       //get the current "time" for DwD
    if (currentMillis - startMillis3 >= interval3)  //test whether the period has elapsed
    {

 //Serial.println("1 min elpased");
 //Serial.print("Check Temp: ");
    checkTemp();

      if (ctFlag) {
        digitalWrite(pumpPin, HIGH);   // make sure heat pump on
        stat = 3;
      } else {
        startMillis2 = millis();       //initial start time for DwD
        digitalWrite(pumpPin, LOW);   // turn heat pump off
        if (t.hour >= startNight) {
          stat = 0;
 //Serial.println("Status 0 initiated by stat == 2");
        } else {
          stat = 1;
 //Serial.println("Status 1 initiated by stat == 2");
        }

      }
        
      startMillis3 = currentMillis;  //IMPORTANT to save the start time state.
    }
  } // End if stat == 2
  else if (stat == 3) {
    digitalWrite(pumpPin, HIGH);   // make sure heat pump on
    checkTemp();
    if (ctFlag == 0) { 
      digitalWrite(pumpPin, LOW);   // turn heat pump off
      startMillis3 = millis();       //initial start time for DwD     
      stat = 2;
    }
  } // End of stat ==3
  
} // End of the loop function

void writeTemp(float t, int corner) {                            // writing the current temp to the LCD
                                                               // use of corner - 0-> Top left
                                                               //                 1-> Top right
                                                               //                 2-> Bottom left
                                                               //                 3-> Bottom right
  int topBottom;
  int rightLeftA;
  int rightLeftB;

  if (corner == 0) {       // little bit formating
    topBottom = 0;
    rightLeftA = 0;
    rightLeftB = 4;
  } else if (corner == 1) {
    topBottom = 0;
    rightLeftA = 8;
    rightLeftB = 12;
  } else if (corner == 2) {
    topBottom = 1;
    rightLeftA = 0;
    rightLeftB = 4;
  } else {
    topBottom = 1;
    rightLeftA = 8;
    rightLeftB = 12;
  }  
  
  lcd.setCursor ( rightLeftA, topBottom );
  lcd.print(t);
  lcd.setCursor ( rightLeftB, topBottom );
  lcd.print((char)223);
  lcd.print("C");  
} // End of writeTemp function

void writeTime(struct ts *t, int corner){     // writting current time to the LCD.
                                              // use of corner - 0-> Top left
                                              //                 1-> Top right
                                              //                 2-> Bottom left
                                              //                 3-> Bottom right
  char buff[BUFF_MAX];            // will hold the date time string
  timeStringFormat(buff, t);

  if (corner == 0) lcd.setCursor ( 0, 0 );       // little bit formating
  else if (corner == 1) lcd.setCursor ( 8, 0 );
  else if (corner == 2) lcd.setCursor ( 0, 1 );
  else lcd.setCursor ( 8, 1 );
  
  lcd.print(buff);
} // End of writeTime

void timeStringFormat (char *buff, struct ts *t) {
//  snprintf(buff, BUFF_MAX, "%d.%02d.%02d %02d:%02d:%02d", t->year, t->mon, t->mday, t->hour, t->min, t->sec);
  snprintf(buff, BUFF_MAX, "%02d:%02d:%02d", t->hour, t->min, t->sec);  
} // End of the timeStringFormat function


void initStat(void){
  struct ts t;                  // holder for the time data
  DS3231_get(&t);               // Read the time from the RTC module

  if(t.hour > startNight ||t.hour < startMorning) {
    startMillis2 = millis();       //initial start time for DwD
    stat = 0;

 //Serial.println("Status 0 initiated by initStat");
  }
  else if (t.hour >=startMorning && t.hour <= startNight) {
            checkTemp();
            if (ctFlag) {
              startMillis3 = millis();       //initial start time for DwD     
              stat = 2;
            }
 //Serial.println("Status 2 initiated by initStat");            }
            else {
              startMillis2 = millis();       //initial start time for DwD
              stat = 1;
 
 //Serial.println("Status 1 initiated by initStat");
            }
          }

} // End of initStat

void getTemp (void){

  sensor1.requestTemperatures();
  currentTemp0 = sensor1.getTempCByIndex(0);
  currentTemp0 = round(currentTemp0 * 10.0)/10.0;
  
  sensor2.requestTemperatures();
  currentTemp1 = sensor2.getTempCByIndex(0);
  currentTemp1 = round(currentTemp1 * 10.0)/10.0;
} // End of Get temp function

void checkTemp (void){
  
  if (currentTemp0 >= currentTemp1 +1) { ctFlag = 1; }
  else { ctFlag = 0;}

}
