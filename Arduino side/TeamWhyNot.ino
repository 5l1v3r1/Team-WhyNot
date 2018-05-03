#include <Adafruit_GPS.h>
#include <SparkFun_ADXL345.h>
#include <RTClib.h>
#include <Wire.h>
#include <SFE_BMP180.h>
#include <SoftwareSerial.h>
#include <SD.h>
#include <Servo.h>

//ADXL345 and rtc variable initializing
RTC_DS3231 rtc;
ADXL345 adxl = ADXL345();


//Software serial port initializing
SoftwareSerial mySerial(7, 8);
SoftwareSerial Xbee(5,6);
Adafruit_GPS GPS(&mySerial);

//Servo variable initializing
Servo heatshield;


//PRESSURE sensor variable initailizinh
SFE_BMP180 pressure;

//Creating file variables for photos.
//There will be 10 photo in 30m(meter) period
File photo1;
File photo2;
File photo3;
File photo4;
File photo5;
File photo6;
File photo7;
File photo8;
File photo9;
File photo10;


#define ALTITUDE 1900.0
#define GPSECHO  false

boolean usingInterrupt = false;
void useInterrupt(boolean);

int i = 0;
int posit = 0;
int last altitude = 0;

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

  //Servo motor attaching port
  heatshield.attach();//Will be initialized

  //Sd Card initializing
  if(!SD.begin(4)){
    while(1);
  }

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

  photo1 = SD.open("photo1.jpg", FILE_WRITE);
  photo2 = SD.open("photo2.jpg", FILE_WRITE);
  photo3 = SD.open("photo3.jpg", FILE_WRITE);
  photo4 = SD.open("photo4.jpg", FILE_WRITE);
  photo5 = SD.open("photo5.jpg", FILE_WRITE);
  photo6 = SD.open("photo6.jpg", FILE_WRITE);
  photo7 = SD.open("photo7.jpg", FILE_WRITE);
  photo8 = SD.open("photo8.jpg", FILE_WRITE);
  photo9 = SD.open("photo9.jpg", FILE_WRITE);
  photo10 = SD.open("photo10.jpg", FILE_WRITE);


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

void loop() {

  //ADXL345
  int x,y,z;   
  adxl.readAccel(&x, &y, &z);

  if((y < -30)&& (z < -30)){
    heatshield.write(45);//Half rotate for heatshield.This will block heat shield from release 
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
  Xbee.print(",");
  //Pressure from bmp180
  Xbee.print(P,2);
  Xbee.print(",");
  //Temperature from bmp180
  Xbee.print(T,2);
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
  Xbee.println();


//Taking photo between 30 m altitude
  while(GPS.altitude < 310) {
    if (GPS.altitude < 310)
      heatshield.write(90);//Degree will be changed
      photo1.print();
      break;
    if (GPS.altitude < 280)
      photo2.print();
      break;
    if (GPS.altitude < 250)
      photo3.print();
      break;
    if (GPS.altitude < 220)
      photo4.print();
      break;
    if (GPS.altitude < 190)
      photo5.print();
      break;
    if (GPS.altitude < 160)
      photo6.print();
      break;
    if (GPS.altitude < 130)
      photo7.print();
      break;
    if (GPS.altitude < 100)
      photo8.print();
      break;
    if (GPS.altitude < 70)
      photo9.print(); 
      break;
    if (GPS.altitude < 40)
      photo10.print();
      break;
    if (GPS.altitude < 30)
      photo1.close();
      photo2.close();
      photo3.close();
      photo4.close();
      photo5.close();
      photo6.close();
      photo7.close();
      photo8.close();
      photo9.close();
      photo10.close();
      break;
    if(GPS.altitude <20)
      break;

  }
  if(i % 3 == 0){
    //Controlling for descending status
    lastaltitude = GPS.altitude;
    if((lastaltitude - GPS.altitude) < 0){
      //If there is such a condition xbee will tell that
      Xbee.println("Probe is descending");

    }
  }

  i++;
  delay(1000);
}
