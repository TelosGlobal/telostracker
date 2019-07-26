
// *********************************************************
// *            TelosTracker                               *
// *                                                       *
// *     Author: Kevin Quaintance                          *
// *     Date:   7/25/2019                                 *
// *                                                       *
// *     This script works with Particle Electron cellular *
// *     2G world microcontroller (arduino based).         *
// *     It measures temp/humidity samples from 5 probes,  *
// *     vibration, and GPS location data every 10 mins    *
// *     and posts it the Particle Cloud.                  *
// *     Another backoffice process takes that data from   *
// *     the Particle Cloud and records the data onto the  *
// *     Telos blockchain.                                 *
// *                                                       *
// *********************************************************

// This #include statement was automatically added by the Particle IDE.
#include <Adafruit_GPS.h>

// This #include statement was automatically added by the Particle IDE.
// This is the temp sensor library
#include <Adafruit_DHT_Particle.h>

// Setup the 5 temp/humidity sensors
#define DHTPIN0 C4     // Onboard sensor
#define DHTPIN1 C5     // Sensor1
#define DHTPIN2 D0     // Sensor2
#define DHTPIN3 D1     // Sensor3
#define DHTPIN4 D2     // Sensor4

// Here we define the type of temp sensor used
#define DHTTYPE DHT22       // DHT 11 

// Assign sensor pins
DHT dht0(DHTPIN0, DHTTYPE);
DHT dht1(DHTPIN1, DHTTYPE);
DHT dht2(DHTPIN2, DHTTYPE);
DHT dht3(DHTPIN3, DHTTYPE);
DHT dht4(DHTPIN4, DHTTYPE);

// Setup the tilt/virbration sensor
#define TILT D7

//Initialize external RGB led
STARTUP(RGB.mirrorTo(B3, B2, B1));

//Declare Particle Cloud Variables ///////////////////////////
//These become accessible variables accessable 
//via the Particle APIs
int vibr  = 0;
int vibr0 = 0;
int temp0 = 0;
int hum0  = 0;
int temp1 = 0;
int hum1  = 0;
int temp2 = 0;
int hum2  = 0;
int temp3 = 0;
int hum3  = 0;
int temp4 = 0;
int hum4  = 0;
int fix  = 0;
int sats  = 0;
int longitude = 0;
int lon = 0;
int latitude = 0;
int lat = 0;
int altitude = -1000;
int speed = 0;
String stamp;

// Setup for interval publishing /////////////////////////////

void displayInfo(); // forward declaration
// Target is every 10 minutes(in milliseconds)
const unsigned long PUBLISH_PERIOD = 598000; //598sec to adjust for some slip
// force lastPublish to trigger immediately when loop starts
long int lastPublish = -600000;

//This invokes a routine that attempts to sync the interval to 
//trigger when the time hits 10 minute increments.  It runs
//on first loop pass and then once per day around 11:54pm UTC.
int issync = 0;

// GPS config /////////////////////////////////////////////////
#ifndef PARTICLE
//SoftwareSerial mySerial(3, 2);
#else
USARTSerial& mySerial = Serial1;
#endif

// what's the name of the hardware serial port?
//HardwareSerial mySerial Serial1;

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&mySerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences.
#define GPSECHO false

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy


////////////////////////////////////////////////////////////////
void setup() {

//  Register Cloud Variables with Particle Cloud
    Particle.variable("stamp", stamp);
    Particle.variable("vibr0", vibr0);
    Particle.variable("temp0", temp0);
    Particle.variable("hum0", hum0);
    Particle.variable("temp1", temp1);
    Particle.variable("hum1", hum1);
    Particle.variable("temp2", temp2);
    Particle.variable("hum2", hum2);
    Particle.variable("temp3", temp3);
    Particle.variable("hum3", hum3);
    Particle.variable("temp4", temp4);
    Particle.variable("hum4", hum4);
    Particle.variable("fix", fix);
    Particle.variable("sats", sats);
    Particle.variable("longitude", longitude);
    Particle.variable("lon", lon);
    Particle.variable("latitude", latitude);
    Particle.variable("lat", lat);
    Particle.variable("altitude", altitude);
    Particle.variable("speed", speed);
    
    // Force a connect to the Cloud
//    if (Particle.connected() == false) {
//       Particle.connect();
//    }
        
    // Common-anode RGB LED connected to (R), (G), (B)
    RGB.mirrorTo(B3, B2, B1, true, true);

    // We open up a serial port to monitor the sensor values
    //Serial.begin(9600); 
    //Serial.println("TelosTracker Initializing...");
    Particle.publish("TelosTracker Initializing...", Time.timeStr());
    
    // Initiate Temp sensors
    dht0.begin();
    dht1.begin();
    dht2.begin();
    dht3.begin();
    dht4.begin();
    
    // Initiate the tilt sensor
    pinMode(TILT, INPUT);

// GPS setup ///////////////////////////////////////////////////////////////////////////
    // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
    GPS.begin(9600);
    delay(1000);

    //# request a HOT RESTART, in case we were in standby mode before.
//    GPS.sendCommand("$PMTK101*32");
//    delay(1000);
    
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time

  // Set the update rate
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  // GPS.sendCommand(PGCMD_ANTENNA);

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
    useInterrupt(true);

    delay(1000);  
    
      // Ask for firmware version
//    mySerial.println(PMTK_Q_RELEASE);
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
#ifndef PARTICLE
SIGNAL(TIMER0_COMPA_vect) {
#else
void handleSysTick(void* data) {
#endif
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO && c) {
  #ifdef UDR0
    UDR0 = c;
    // writing direct to UDR0 is much much faster than Serial.print
    // but only one character can be written at a time.
  #else
    Serial.write(c);
  #endif
  }
}

void useInterrupt(boolean v) {
  #ifndef PARTICLE
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
  #else
    static HAL_InterruptCallback callback;
    static HAL_InterruptCallback previous;
    callback.handler = handleSysTick;
    HAL_Set_System_Interrupt_Handler(SysInterrupt_SysTick, &callback, &previous, nullptr);
  #endif
}

//uint32_t timer = millis();


void loop() {

  // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
//  if (timer > millis())  timer = millis();
  

//  delay(1000);

// Populate GPS variables for publishing
    if (GPS.fix > fix) {
        fix = GPS.fix;
    }
    if (GPS.satellites > sats) {
        sats = GPS.satellites;
    }    
    if (GPS.latitude > latitude) {
        latitude = GPS.latitude;
    }
    if (GPS.lat > lat) {
        lat = GPS.lat;
    }
    if (GPS.longitude > longitude) {
        longitude = GPS.longitude;
    }
    if (GPS.lon > lon) {
        lon = GPS.lon;
    }
    if (GPS.altitude > altitude) {
        altitude = GPS.altitude;
    }
    if (GPS.speed > speed) {
        speed = GPS.speed;
    }

//  Read tilt sensor
    long measurement =TP_init();
    delay(50);
    stamp = Time.timeStr();
    vibr  = static_cast<int>(measurement);
    if (vibr > vibr0) {
        vibr0 = vibr;
    }

// CHECK SENSOR0
    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 
    float h0 = dht0.getHumidity();
    // Read temperature as Celsius
    float t0 = dht0.getTempCelcius();
    temp0 = static_cast<int>(t0);
    hum0  = static_cast<int>(h0);

// CHECK SENSOR1
    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 
    float h1 = dht1.getHumidity();
    // Read temperature as Celsius
    float t1 = dht1.getTempCelcius();
    temp1 = static_cast<int>(t1);
    hum1  = static_cast<int>(h1);

// CHECK SENSOR2
    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 
    float h2 = dht2.getHumidity();
    // Read temperature as Celsius
    float t2 = dht2.getTempCelcius();
    temp2 = static_cast<int>(t2);
    hum2  = static_cast<int>(h2);
  
// CHECK SENSOR3
    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 
    float h3 = dht3.getHumidity();
    // Read temperature as Celsius
    float t3 = dht3.getTempCelcius();
    temp3 = static_cast<int>(t3);
    hum3  = static_cast<int>(h3);

// CHECK SENSOR4
    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 
    float h4 = dht4.getHumidity();
    // Read temperature as Celsius
    float t4 = dht4.getTempCelcius();
    temp4 = static_cast<int>(t4);
    hum4  = static_cast<int>(h4);



if (millis() - lastPublish >= PUBLISH_PERIOD) {
	lastPublish = millis();
    displayInfo();
}

// Let's sync time with the Cloud (via cellular) if connected at 00:55:55am UTC
if (Time.hour() == 0) {
    if (Time.minute() == 52) {
        if (Time.second() == 55) {
            if (!Particle.connected()) {
                Particle.connect();
            }
            delay(5000);
            // Request time synchronization from the Particle Device Cloud
            Particle.syncTime();
            // Wait until Electron receives time from Particle Device Cloud (or connection to Particle Device Cloud is lost)
            waitUntil(Particle.syncTimeDone);
            Particle.publish("Time Synced with Cloud.");
            //Publish daily vitals
            Particle.publishVitals(); 
            
            //  Let's trigger a resync of the publishing
            issync = 0;
            delay(2000);
        }
    }
}    

// Wait a second and do it again.
delay(1000);

}

void displayInfo()
{
    Particle.publish("Timestamp", Time.timeStr());
    Particle.publish("Vibration", String(vibr0));
    delay(2000);
    Particle.publish("Hum0", String(hum0));
    Particle.publish("Temp0", String(temp0));
    delay(2000);
    Particle.publish("Hum1", String(hum1));
    Particle.publish("Temp1", String(temp1));
    delay(2000);  
    Particle.publish("Hum2", String(hum2));
    Particle.publish("Temp2", String(temp2));
    delay(2000);  
    Particle.publish("Hum3", String(hum3));
    Particle.publish("Temp3", String(temp3));
    delay(2000); 
    Particle.publish("Hum4", String(hum4));
    Particle.publish("Temp4", String(temp4)); 
    delay(2000);
    Particle.publish("GPSfix", String(fix));
    Particle.publish("SATs", String(sats));
    delay(2000); 
    Particle.publish("Longitude", String(longitude));
    Particle.publish("Lon", String(lon));
    delay(2000); 
    Particle.publish("Latitude", String(latitude));
    Particle.publish("Lat", String(lat)); 
    delay(2000); 
    Particle.publish("Altitude", String(altitude));
    Particle.publish("Speed (Knots)", String(speed));     
	
	//Reset some variables
	fix = 0;
	sats = 0;
	vibr0 = 0;
	longitude = 0;
    lon = 0;
    latitude = 0;
    lat = 0;
    altitude = -1000;
    speed = 0;   
	
    //This section will sync up the intervals to 10 minutes on the hour  
    if (issync == 0) {  
        int now = millis();
        int min = Time.minute();
        int sec = Time.second();
        // Particle.publish("Timestamp: ", Time.timeStr());
        // Particle.publish("millis: ", String(now));
        int rnd = (ceil((min*.1)+.05));
        int trig = (rnd*600000);
        int curr = (((min*60)+sec)*1000);
        int nextint = trig - curr;
        int elapsed = PUBLISH_PERIOD - nextint;
        lastPublish = now - (elapsed + 4500); //4500millis (4.5 secs) to adjust back a bit further
        // delay(2000);
        // Particle.publish("rnd: ", String(rnd));
        // Particle.publish("trig: ", String(trig));
        // delay(2000);
        // Particle.publish("curr: ", String(curr));
        // Particle.publish("nextint: ", String(nextint));
        // delay(2000);
        // Particle.publish("elapsed: ", String(elapsed));
        // Particle.publish("new lastPub: ", String(lastPublish));
        
        // One and done.
        issync = 1;
    }
  
}    

// This will pull the vibration data from the sensor
long TP_init(){
    delay(10);
    long measurement=pulseIn (TILT, HIGH);  //wait for the pin to get HIGH and returns measurement
    return measurement;
}

// ***************ADAFRUITGPS*********************
// Based on: https://github.com/adafruit/Adafruit_GPS/blob/master/examples/GPS_HardwareSerial_Parsing/GPS_HardwareSerial_Parsing.ino

// These are random examples of how to access the GPS data

  // if millis() or timer wraps around, we'll just reset it
//  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
//  if (millis() - timer > 60000) {
//    timer = millis(); // reset the timer

//    Serial.print("\nTime: ");
//    Serial.print(GPS.hour, DEC); Serial.print(':');
//    Serial.print(GPS.minute, DEC); Serial.print(':');
//    Serial.print(GPS.seconds, DEC); Serial.print('.');
//    Serial.println(GPS.milliseconds);
//    Serial.print("Date: ");
//    Serial.print(GPS.day, DEC); Serial.print('/');
//    Serial.print(GPS.month, DEC); Serial.print("/20");
//    Serial.println(GPS.year, DEC);
//    Serial.print("Fix: "); Serial.print((int)GPS.fix);
//    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
//    if (GPS.fix) {
//      Serial.print("Location: ");
//      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
//      Serial.print(", ");
//      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
//      Serial.print("Location (in degrees, works with Google Maps): ");
//      Serial.print(GPS.latitudeDegrees, 4);
//      Serial.print(", ");
//      Serial.println(GPS.longitudeDegrees, 4);

//      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
//      Serial.print("Angle: "); Serial.println(GPS.angle);
//      Serial.print("Altitude: "); Serial.println(GPS.altitude);
//      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
//      Particle.publish("Altitude", String(GPS.altitude));
//      Particle.publish("Speed (knots)", String(GPS.speed));
//      Particle.publish("Lat", String(GPS.latitude + GPS.lat));
//      Particle.publish("Lon", String(GPS.longitude + GPS.lon));
//    }
//  }
