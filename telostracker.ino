// *********************************************************
// *            TelosTracker                               *
// *                                                       *
// *     Author: Kevin Quaintance                          *
// *     Date:   7/30/2019                                 *
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

#include <math.h>

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
bool pwr  = 0;
int vibr  = 0;
int vibr0 = 0;
int temp0 = 0;
double t = 0;
double h = 0;
double t0 = 0;
double h0 = 0;
double t1 = 0;
double h1 = 0;
double t2 = 0;
double h2 = 0;
double t3 = 0;
double h3 = 0;
double t4 = 0;
double h4 = 0;
int fix  = 0;
int sats  = 0;
double longitude = 0;
double latitude = 0;
int altitude = -1000;
int speed = 0;
String telemetry;
int ransync = 1;

//String DHT[] = {"dht0", "dht1", "dht2", "dht3", "dht4"};
//int temps[] = {t0,t1,t2,t3,t4};
//int humds[] = {h0,h1,h2,h3,h4};
//int count = arraySize(temps);


// Setup for interval publishing /////////////////////////////

void displayInfo(); // forward declaration


// Target is every 10 minutes(in milliseconds)
const unsigned long PUBLISH_PERIOD = 591000; // 9 seconds less than 10mins helps with slips 
// force lastPublish to trigger immediately when loop starts
long int lastPublish = -591000;

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

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&mySerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences.
#define GPSECHO false

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

PMIC power;

////////////////////////////////////////////////////////////////
void setup() {

//  Register Cloud Variables with Particle Cloud
    Particle.variable("pwr", pwr);    
    Particle.variable("vibr0", vibr0);
    Particle.variable("t0", t0);
    Particle.variable("h0", h0);
    Particle.variable("t1", t1);
    Particle.variable("h1", h1);
    Particle.variable("t2", t2);
    Particle.variable("h2", h2);
    Particle.variable("t3", t3);
    Particle.variable("h3", h3);
    Particle.variable("t4", t4);
    Particle.variable("h4", h4);
    Particle.variable("fix", fix);
    Particle.variable("sats", sats);
    Particle.variable("longitude", longitude);
    Particle.variable("latitude", latitude);
    Particle.variable("altitude", altitude);
    Particle.variable("speed", speed);
    Particle.variable("telemetry", telemetry);
    Particle.variable("ransync", ransync);
    
    // Common-anode RGB LED connected to (R), (G), (B)
    RGB.mirrorTo(B3, B2, B1, true, true);

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
    GPS.sendCommand("$PMTK101*32");
    delay(1000);
    
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
    useInterrupt(false);

    // Publish init state
    Particle.publish("TelosTracker Initializing...", Time.timeStr());
    delay(1000);  
    
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

void loop() {
    
// Check if unit is getting good power
  pwr = power.isPowerGood();
    
// GPS STUFF /////////////////////////////////////////////////////////////
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

// Populate GPS variables for publishing
    if (GPS.fix > fix) {
        fix = GPS.fix;
    }
    if (GPS.satellites > sats) {
        sats = GPS.satellites;
    }    
    if (GPS.latitudeDegrees != latitude) {
        latitude = GPS.latitudeDegrees;
    }
//    if (GPS.lat > lat) {
//        lat = GPS.lat;
//    }
    if (GPS.longitudeDegrees != longitude) {
        longitude = GPS.longitudeDegrees;
    }
//    if (GPS.lon > lon) {
//        lon = GPS.lon;
//    }
    if (GPS.altitude > altitude) {
        altitude = GPS.altitude;
    }
    if (GPS.speed > speed) {
        speed = GPS.speed;
    }

//  Read tilt sensor
    long measurement =TP_init();
    delay(50);
//    stamp = Time.timeStr();
    vibr  = static_cast<int>(measurement);
    if (vibr > vibr0) {
        vibr0 = vibr;
    }

    // CHECK SENSORS
    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 
    h = dht0.getHumidity();
    delay(2000);
    if (isnan(h)) {
        h0 = -999.000;
    }
    else if (h0 < h) {
        h0 = h;
    }
    else {
    }
    // Read temperature as Celsius
    t = dht0.getTempCelcius();
    if (isnan(t)) {
        t0 = -999.000;
    }
    else if (t0 < t) {
        t0 = t;
    }
    else {
    }


    
// CHECK SENSOR1
    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 
//    h1 = dht1.getHumidity();
    // Read temperature as Celsius
//    t1 = dht1.getTempCelcius();
//    delay(2000);
    h = dht1.getHumidity();
    delay(2000);
    if (isnan(h)) {
        h1 = -999.000;
    }
    else if (h1 < h) {
        h1 = h;
    }
    else {
    }
    // Read temperature as Celsius
    t = dht1.getTempCelcius();
    if (isnan(t)) {
        t1 = -999.000;
    }
    else if (t1 < t) {
        t1 = t;
    }
    else {
    }

    
// CHECK SENSOR2
    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 
//    h2 = dht2.getHumidity();
    // Read temperature as Celsius
//    t2 = dht2.getTempCelcius();
//    delay(2000);
    h = dht2.getHumidity();
    delay(2000);
    if (isnan(h)) {
        h2 = -999.000;
    }
    else if (h2 < h) {
        h2 = h;
    }
    else {
    }
    // Read temperature as Celsius
    t = dht2.getTempCelcius();
    if (isnan(t)) {
        t2 = -999.000;
    }
    else if (t2 < t) {
        t2 = t;
    }
    else {
    }

    
  
// CHECK SENSOR3
    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 
//    h3 = dht3.getHumidity();
    // Read temperature as Celsius
//    t3 = dht3.getTempCelcius();
//    delay(2000);
    h = dht3.getHumidity();
    delay(2000);
    if (isnan(h)) {
        h3 = -999.000;
    }
    else if (h3 < h) {
        h3 = h;
    }
    else {
    }
    // Read temperature as Celsius
    t = dht3.getTempCelcius();
    if (isnan(t)) {
        t3 = -999.000;
    }
    else if (t3 < t) {
        t3 = t;
    }
    else {
    }


// CHECK SENSOR4
    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 
//    h4 = dht4.getHumidity();
    // Read temperature as Celsius
//    t4 = dht4.getTempCelcius();
//    delay(2000);
    h = dht4.getHumidity();
    delay(2000);
    if (isnan(h)) {
        h4 = -999.000;
    }
    else if (h4 < h) {
        h4 = h;
    }
    else {
    }
    // Read temperature as Celsius
    t = dht4.getTempCelcius();
    if (isnan(t)) {
        t4 = -999.000;
    }
    else if (t4 < t) {
        t4 = t;
    }
    else {
    }

    
//If our interval is met, let's go publish the data
if (millis() - lastPublish >= PUBLISH_PERIOD) {
	lastPublish = millis();
    displayInfo();
}

// Let's sync time with the Cloud (via cellular) if connected at 00:55:55am UTC
if (Time.hour() == 1) {
    if (Time.minute() == 57) {
        if (Time.second() > 30 && Time.second() <59) {
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
            
            //Let's reset the GPS module for grins
            GPS.sendCommand("$PMTK101*32");
            
            //  Let's trigger a resync of the publishing
            issync = 0;
            delay(2000);
        }
    }
}    

// Wait a second and do it again.
//delay(1000);

}

// This will pull the vibration data from the sensor
long TP_init(){
    delay(10);
    long measurement=pulseIn (TILT, HIGH);  //wait for the pin to get HIGH and returns measurement
    return measurement;
}

void displayInfo()
{

    // Get time now in epoch
    int numOfSecond = Time.local();

    String telemetry = String::format(
    "{\"ts\":%i, \"fix\":%i, \"sats\":%i, \"lat\":%g, \"lng\":%g, \"atl\":%.1f, \"spd\":%.1f, "
    "\"t0\":%.1f, \"t1\":%.1f, \"t2\":%.1f, \"t3\":%.1f, \"t4\":%.1f, \"h0\":%.1f, \"h1\":%.1f,"
    "\"h2\":%.1f, \"h3\":%.1f, \"h4\":%.1f, \"vib\":%i, \"pwr\":%i}",
    numOfSecond, fix, sats, GPS.latitudeDegrees, GPS.longitudeDegrees, GPS.altitude, GPS.speed,
    t0, t1, t2, t3, t4, h0, h1, h2, h3, h4, vibr0, pwr);
    
    Particle.publish("Telemetry", String(telemetry));
    //delay(2000);
	//Particle.publish("t0", String(t0));
	//Particle.publish("t1", String(t1));
    //delay(2000);
	//Particle.publish("t2", String(t2));
	//Particle.publish("t3", String(t3));
    //delay(2000);
    //Particle.publish("t4", String(t4));
	//Particle.publish("h0", String(h0));
    //delay(2000);
    //Particle.publish("h1", String(h1));
	//Particle.publish("h2", String(h2));
    //delay(2000);
    //Particle.publish("h3", String(h3));
	//Particle.publish("h4", String(h4));

	
	//Reset some variables to ensure our data updates aren't stale
	pwr = 0;
	vibr0 = 0;
	fix = 0;
	sats =0;
	speed=0;
	altitude = -1000;
	t = 0;
	h = 0;
	t0 = 0;
	h0 = 0;
	t1 = 0;
	h1 = 0;
	t2 = 0;
	h2 = 0;
	t3 = 0;
	h3 = 0;
	t4 = 0;
	h4 = 0;
	
 
    //This section will sync up the intervals to 10 minutes on the hour  (kinda works)
    if (issync == 0) {  
        int now = millis();
        int min = Time.minute();
        int sec = Time.second();
        int rnd = (ceil((min*.1)+.05));
        int trig = (rnd*600000);
        int curr = (((min*60)+sec)*1000);
        int nextint = trig - curr;
        int elapsed = PUBLISH_PERIOD - nextint;
        lastPublish = now - (elapsed + 1500); //1500millis (1.5 secs) to adjust back a bit further
        Particle.publish("ransync", String(ransync));
        // One and done.
        issync = 1;
    }
  
}    
