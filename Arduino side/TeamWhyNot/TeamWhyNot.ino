#include <Adafruit_GPS.h>
#include <SparkFun_ADXL345.h>
#include <RTClib.h>
#include <Wire.h>
#include <SFE_BMP180.h>
#include <SoftwareSerial.h>
#include <Servo.h>

//ADXL345 and rtc variable initializing
RTC_DS3231 rtc;
ADXL345 adxl = ADXL345();

int analogInput = 6;
int stat = 0;
int refresh = 1000;
float vout = 0.0; // arduino'ya giren voltaj değeri
float vin = 0.0; // bataryamızın voltaj değeri
float R1 = 100000; // R1 direnci 100k
float R2 = 10000; // R2 direnci 10k
int value = 0; // analog port okunan değer (0, 1024)


//Software serial port initializing
SoftwareSerial mySerial(7, 8);
SoftwareSerial Xbee(6,5);
Adafruit_GPS GPS(&mySerial);

//Servo variable initializing
Servo heatshield;


//PRESSURE sensor variable initailizinh
SFE_BMP180 pressure;

//Creating file variables for photos.
//There will be 10 photo in 30m(meter) period


#define ALTITUDE 1900.0
#define GPSECHO  false

boolean usingInterrupt = false;
void useInterrupt(boolean);

int i = 0;
int posit = 0;
int lastalt = 0;
int kirmiziPin = 12;
int yesilPin = 11;
int maviPin = 10;
int kirmizi2 = 3;
int yesil2 = 4;
int mavi2 = 13;
int buttonPin = 2;
int buttonState = 0;  

void setup() {
  //Xbee and Serial port communication
  Serial.begin(115200);
  Xbee.begin(9600);
  
  //GPS initalizing
  GPS.begin(9600);
  
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  
  //BMP180 initializing
  pressure.begin();
  
  pinMode(buttonPin, INPUT);
  pinMode(kirmiziPin, OUTPUT);
  pinMode(yesilPin, OUTPUT);
  pinMode(maviPin, OUTPUT);

  //Servo motor attaching port
  heatshield.attach(9);//Will be initialized
  heatshield.write(0);
  //RTC start
  rtc.begin();
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  //ADXL start
  adxl.powerOn();
  adxl.setRangeSetting(8);
  adxl.setActivityXYZ(1, 0, 0);
  adxl.setActivityThreshold(75);
  adxl.setInactivityXYZ(1, 0, 0);
  adxl.setInactivityThreshold(75);
  adxl.setTimeInactivity(10);
  adxl.setTapDetectionOnXYZ(0, 0, 1);
  adxl.setTapThreshold(50);
  adxl.setTapDuration(15);
  adxl.setDoubleTapLatency(80);
  adxl.setDoubleTapWindow(200);
  adxl.setFreeFallThreshold(7);
  adxl.setFreeFallDuration(30);
  adxl.InactivityINT(0);
  adxl.ActivityINT(0);
  adxl.FreeFallINT(0);
  adxl.doubleTapINT(0);
  adxl.singleTapINT(0);

 // put your setup code here, to run once:
  pinMode(analogInput, INPUT);
  Serial.begin(9600);

  useInterrupt(true);
  delay(1000);

  mySerial.println(PMTK_Q_RELEASE);
}


SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

uint32_t timer = millis();


void renkAyarla(int kpin, int ypin, int mpin,int kirmizi, int yesil, int mavi){
 kirmizi = 255 - kirmizi;
 yesil = 255 - yesil;
 mavi = 255 - mavi;
 analogWrite(kpin, kirmizi);
 analogWrite(ypin, yesil);
 analogWrite(mpin, mavi);
}

void loop() {

  buttonState = digitalRead(buttonPin);

  /*BATTERY SENSOR START*/
  value = analogRead(analogInput);
 
  vout = (value*5)/1024;
  vin = vout*(R1+R2) / R2;


  vout = (value*5)/1024;
  vin = vout*(R1+R2) / R2;

  
  delay(refresh);
  /* BATTERY SENSOR END*/
 
  //ADXL345
  int x,y,z;   
  adxl.readAccel(&x, &y, &z);

  if(y > 30){
    //heatshield.write(45);//Half rotate for heatshield.This will block heat shield from release 
  }

  //RTC
  DateTime now = rtc.now();
  
  //BMP180 initilaizer
  
  char status;
  double T,P,p0,a;

  //<TEAM ID>,<MISSION TIME>,<PACKET COUNT>,<ALTITUDE>, <PRESSURE>,
  //<TEMP>,<VOLTAGE>,<GPS TIME>,<GPS LATITUDE>,<GPS LONGITUDE>,<GPS
  //ALTITUDE>,<GPS SATS>,<TILT X>,<TILT Y>,<TILT Z>,<SOFTWARE STATE>
  status = pressure.startTemperature();
  if (status != 0){
    delay(status);
    status = pressure.getTemperature(T);
    if (status != 0){
      status = pressure.startPressure(3);
      if(status != 0){
        delay(status);
        status = pressure.getPressure(P,T);
        if (status != 0){
          delay(status);
          p0 = pressure.sealevel(P,ALTITUDE);
          a = pressure.altitude(P,p0);
        }
      }   
    }
  }


  //RTC Variable initializing
  int rtchour = now.hour();
  int rtcmin = now.minute();
  int rtcsec = now.second();


  //Testing from serial monitor
  //Team ID
  Xbee.print("4266,");
  //Time from RTC
  Xbee.print(now.hour(), DEC);
  Xbee.print(":");
  Xbee.print(now.minute(), DEC);
  Xbee.print(":");
  Xbee.print(now.second(), DEC);
  Xbee.print(",");
  //Packet Count
  Xbee.print(i);
  Xbee.print(",");
  //Altitude from bmp180
  Xbee.print(a,0);
  Serial.print(a,0);
  Xbee.print(",");
  //Pressure from bmp180
  Xbee.print(P,2);
  Serial.print(P,2);
  Xbee.print(",");
  //Temperature from bmp180
  Xbee.print(T,2);
  Xbee.print(",");
  Xbee.print(vin);
  Xbee.print("v");
  Xbee.print(",");
  //GPS Data

    //GPS 
  

  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }

  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
  
  if (timer > millis())  timer = millis();
  
  if (millis() - timer > 500) { 
    timer = millis(); // reset the timer
    //GPS Time
    Xbee.print(GPS.hour, DEC); Xbee.print(':');
    Xbee.print(GPS.minute, DEC); Xbee.print(':');
    Xbee.print(GPS.seconds, DEC);
    Xbee.print(",");
    if (GPS.fix) {
      //GPS Latitude
      Xbee.print(GPS.latitude, 4);
      Xbee.print(","); 
      //GPS Longtitude
      Xbee.println(GPS.longitude, 4);
      Xbee.print(","); 
      //GPS Altitude
      Xbee.print(GPS.altitude);
      Xbee.print(","); 
      //Number of GPS Satellites
      Xbee.print((int)GPS.satellites);
      Xbee.print(","); 
    }
  }
  //ADXL345 values
  Xbee.print(x);
  Xbee.print(",");
  Xbee.print(y);
  Xbee.print(",");
  Xbee.print(z);
  //New Line
  Xbee.print(",");
  Xbee.print("Active");
  Xbee.println();
  if (buttonState == HIGH) {
    // turn LED on:
    if(a < 100){
      renkAyarla(kirmiziPin, yesilPin, maviPin,255, 0, 0);
    }else{
      renkAyarla(kirmiziPin, yesilPin, maviPin,0, 255, 0);
    } 
  } else {
    renkAyarla(kirmiziPin, yesilPin, maviPin,0, 0, 255);
  }
  if (vin > 1){
    renkAyarla(kirmizi2, yesil2, mavi2,0, 255, 0);
  }else{
    renkAyarla(kirmizi2, yesil2, mavi2,255, 0, 0);
  }

  i++;
  
  delay(1000);
}


