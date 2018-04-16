//#include <GPS.h>
/*Elcano Module C6: Navigator.
Includes C5 Obstacle detection.

Documentation:
  NavigationSystem (TO DO: Write document, based on these comments).
  Wiring_C6Mega.xls
  Navigation sheet of Elcano_BOM  (TO DO: Reorganize Elcano BOM).

The Navigator has the job of making the best estimate of current vehicle
position, attitude, velocity and acceleration.

It uses a variety of sensors, some of which may not be present oraaa reliable.

Yes: S1: Hall Odometer.  This unit gives wheel spin speed feedback.
TO DO: Connect hardware and integrate into Kalman Filter

Future: Visual Odometry from S4 (Smart camera) as passed though C7 (Visual Data
Management). Visual Odometry could also come from an optical mouse.

Future: S3: Digital Compass

Future: S9: Inertial Navigation Unit.

Yes, from KF: Dead reckoning based on prior state estimate.
TO DO: Integrate Kalman Filter with GPS and Odometry.

Yes: Distance and bearing to landmarks / obstacles.

Yes: S7: GPS.  GPS should not the primary navigation sensor and the robot should be
able to operate indoors, in tunnels or other areas where GPS is not available
or accurate.

Sensed navigation information is passed to C3/C4, which use prexisting or SLAM
digital maps, intended path to refine the estimated position.
C3/C4 may use a particle filter.
C3/C4 may also receive lateral lane deviation from a camera (C7).

The odometry sensor is one dimensional and gives the position along the
intended path. Lane deviation is also one dimensional and gives position
normal to the intended path. Odometry, lane following and a digital map
should be sufficient for localization.  An odometer will drift. This drift
can be nulled out by landmark recognition or GPS.

An alternative to the Kalman Filter (KF) is fuzzy numbers.
The position estimate from most sensors is a pair of fuzzy numbers.
[Milan Mares, Computation Over Fuzzy Quantities, CRC Press.]
A fuzzy number can be thought of as a triangle whose center point
is the crisp position and whose limits are the tolerances to which
the position is known.

GPS provides a pyramid aligned north-south and east-west, in contrast to
odometry / lane deviation, which is a pyramid oriented in the direction of
vehicle motion. The intersection of two fuzzy sets is their minimum.
By taking the minima of all position estimate pyramids, we get a surface
describing the possible positions of the vehicle, whichcan be used by a
particle filter.

Serial lines:
0: Monitor
1: INU
2: Tx: Estimated state;
      $C6EST,<east_mm>,<north_mm>,<speed_mmPs>,<bearing>,<time_ms><positionStndDev_mm>*CKSUM
      $C6OBS,<number>,<obstacle1_range_mm>,<obstacle1_bearing>, ...*CKSUM
   Rx: Desired course
      $C4XPC,<east_mm>,<north_mm>,<speed_mmPs>,<bearing>,<time_ms>*CKSUM
      // at leat 18 characters
3: GPS
  Rx: $GPRMC,...  typically 68 characters
      $GPGSA,...
      $GPGGA,... typically 68 characters
  Tx: $PSRF103,...
*/

/*---------------------------------------------------------------------------------------*/

#define MEGA
#include <Common.h>
#include <SPI.h>
#include <SD.h>
#include <IO.h>
#include <Matrix.h>
#include <ElcanoSerial.h>
using namespace elcano;

#include <Wire.h>
#include <Adafruit_LSM303_U.h>
#include <FusionData.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303_Mag mag = Adafruit_LSM303_Mag(1366123);

long CurrentHeading = -1;

// On the Ethernet Shield, CS is pin 4. Note that even if it's not
// used as the CS pin, the hardware CS pin (10 on most Arduino boards,
// 53 on the Mega) must be left as an output or the SD library
// functions will not work.

namespace C6_Navigator
{
    void setup();
    void loop();
}

// Set the GPSRATE to the baud rate of the GPS module. Most are 4800
// but some are 38400 or other. Check the datasheet!
#define GPSRATE 9600

#define MAX_WAYPOINTS 10
/*   There are two coordinate systems.
     MDF and RNDF use latitude and longitude.
     C3 and C6 assume that the earth is flat at the scale that they deal with.
     All calculations with C6 are performed in the (east_mm, north_mm)
     coordinate system. A raw GPS reading is in latitude and longitude, but it
     is transformed to the flat earth system using the distance from the origin,
     as set by (LATITUDE_ORIGIN, LONGITUDE_ORIGIN).
     A third coordinate system has its origin at the vehicle center and is
     aligned with the vehicle. This is usually transformed into the flat
     world coordinate system.
*/


// limited to 8+3 characters
//#define FILE_NAME "GPSLog.csv"

File dataFile;
char GPSfile[BUFFSIZ] = "mmddhhmm.csv";
char ObstacleString[BUFFSIZ];
char StartTime[BUFFSIZ] = "yy,mm,dd,hh,mm,ss,xxx";
const char TimeHeader[] = "year,month,day,hour,minute,second,msec";
const char* RawKF = "Raw GPS data,,,,,,Kalman Filtered data";
const char* Header = "Latitude,Longitude,East_m,North_m,SigmaE_m,SigmaN_m,Time_s,";
const char* ObstHeader ="Left,Front,Right,Busy";

SerialData data;
ParseState ps;



//#define WHEEL_DIAMETER_MM 397

/* time (micro seconds) for a wheel revolution */
volatile long Odometer_mm = 0;
volatile long SpeedCyclometer_mmPs;
// Speed in degrees per second is independent of wheel size.
volatile long SpeedCyclometer_degPs;

// waypoint mission[MAX_WAYPOINTS];
waypoint GPS_reading;
waypoint estimated_position;
//instrument IMU;
const unsigned long LoopPeriod = 100;  // msec

PositionData oldPos, newPos;


/** 
 * This procedure accepts SerialData structure and prints all the members of the struct to a serial
 * output device. This will be only used for debugging purposes
 * 
 * Added by Varsha
 * Modified by Pooja - The method displayResults originally had every single line of 
 * code within the method commented. So the entire method was also commented by Pooja  
**/
/*void displayResults(SerialData &Results)
{
//    Serial.print("SerialData::Kind :" );
//    Serial.println(Results.kind);
//    Serial.print("SerialData::Number:");    
//    Serial.println(Results.number);    
//    Serial.print("SerialData::speed_cmPs:");
//    Serial.println(Results.speed_cmPs);
//    Serial.print("SerialData::angle_mDeg:");
//    Serial.println(Results.angle_mDeg);    // front wheels
//    Serial.print("SerialData::bearing_deg:");
//    Serial.println(Results.bearing_deg);  // compass direction
//    Serial.print("SerialData::posE_cm:");
//    Serial.println(Results.posE_cm);
//    Serial.print("SerialData::posN_cm:");
//    Serial.println(Results.posN_cm);
//    Serial.print("SerialData::probability:");
//    Serial.println(Results.probability);
//    Serial.print("SerialData::distance_travelled_cm:");
//    Serial.println(Results.distance_travelled_cm);
}*/
//---------------------------------------------------------------------------
/* Feb 23, 2016  TCF.  This instance of WheelRev is depricated.
   Current code for WheelRev is in Elcano_C2_LowLevel.
   The odometer processing is part of low level control.
   C2 will send odometer information to C6 over a serial line. */

/*---------------------------------------------------------------------------------------*/
/* Jun 20, 2016. Varsha - Get heading value from LSM303 sensor  */
long GetHeading(void)
{
    //Get a new sensor event from the magnetometer
    sensors_event_t event;
    mag.getEvent(&event);
    
//    Serial.print("X:");
//    Serial.print(event.magnetic.x);
//    Serial.print(" Y:");
//    Serial.print(event.magnetic.y);
//    Serial.print(" z:");
//    Serial.print(event.magnetic.z);
//    Serial.println("");
    
    //Calculate the current heading (angle of the vector y,x)
    //Normalize the heading
    float heading = (atan2(event.magnetic.y, event.magnetic.x) * 180) / M_PI;

    if (heading < 0)
    {
        heading = 360 + heading;
    }
    // Converting heading to x1000
    return ((long)(heading * HEADING_PRECISION));
//    return heading;
}

/*---------------------------------------------------------------------------------------*/
bool initial_position(){
  pinMode(GPS_POWER, OUTPUT);
  char* GPSString;
  char* protocol  =  "$PSRF100,1,4800,8,1,0*0E"; // NMEA
  char* disable =   "$PSRF103,02,00,00,01*26\r\n";
  char* querryGGA = "$PSRF103,00,01,00,01*25";
  bool GPS_available = false;

// prints title with ending line break
// Serial.println(" GPS parser");
// Serial.print("Acquiring GPS RMC...");
  common::checksum(protocol);
  Serial3.println(protocol);
  disable[10] = '2';
  common::checksum(disable);
  Serial3.println(disable);   // no GSA

GPS_available = estimated_position.AcquireGPGGA(200);
if(!GPS_available) {
  GPS_available = estimated_position.AcquireGPRMC(200);
      if (GPS_available) {
    estimated_position.sigma_mm = 1.0E4; // 10 m standard deviation
    Serial.println("OK");
  }
}
 /* Serial.println(TimeHeader);
  Serial.println(StartTime);
  Serial.println(RawKF);
  Serial.print(Header);
  Serial.print(Header);
  Serial.println(ObstHeader);
  */

  // Set velocity and acceleration to zero.
  estimated_position.speed_mmPs = 0;
  // Set attitude.
  
  //not sure if the compass gives the direction of the trike.....
  estimated_position.Evector_x1000 = 1000;  // to be taken from path or set by hand
  estimated_position.Nvector_x1000 = 60;
  
  // Set Odometer to 0.
  // Set lateral deviation to 0.
  // Read compass.
  // Added by Varsha - To get heading data
  CurrentHeading = GetHeading();
  
  // ReadINU.
    // Wait to get path from C4
//    while (mission[1].latitude > 90)
    {
      /* If (message from C4)
      {
        ReadState(C4);  // get initial route and speed
      }
       Read GPS, compass and IMU and update their estimates.
     */
    }
    
    // GPS_available = GPS_reading.AcquireGPGGA(300);
    // ready to roll
    // Fuse all position estimates.
    // Send vehicle state to C3 and C4.
    
    return GPS_available;
}

/*---------------------------------------------------------------------------------------*/
void setup()
{
  
      //randomSeed(analogRead(0));
//    pinMode(Rx0, INPUT);
//    pinMode(Tx0, OUTPUT);
//    pinMode(GPS_RX, INPUT);
//    pinMode(GPS_TX, OUTPUT);
//    pinMode(C4_RX, INPUT);
//    pinMode(C4_TX, OUTPUT);
//    pinMode(INU_RX, INPUT);
//    pinMode(INU_TX, OUTPUT);
//    pinMode(GPS_POWER, OUTPUT);
   
    Serial.begin(9600);
    Serial1.begin(baudrate);
    Serial.flush();
    Serial2.begin(baudrate);
    Serial3.begin(GPSRATE); // GPS
    Serial3.flush();
    digitalWrite(GPS_POWER, LOW);         // pull low to turn on!
    // make sure that the default chip select pin is set to
    // output, even if you don't use it:
    pinMode(chipSelect, OUTPUT);
    pinMode(53, OUTPUT);  // Unused CS on Mega
    pinMode(GPS_RED_LED, OUTPUT);
    pinMode(GPS_GREEN_LED, OUTPUT);
    digitalWrite(GPS_GREEN_LED, LOW);
    digitalWrite(GPS_RED_LED, LOW);
    
    /* Initializing old PositionData struct to default values
     *  Added by Varsha
     *  Modified by Pooja - SerialData and ParseState are removed as elcanoSerial.h 
     *  is no longer to be used
     */
    /*data.clear();
    ps.dt = &data;
    ps.input = &Serial2;
    ps.output = &Serial2;

    ps.capture = MsgType::drive;*/
    pinMode(16,OUTPUT);
   
    oldPos.Clear();
    oldPos.time_ms = millis();

    //Enable auto-gain
    mag.enableAutoRange(true);


  
    //Initialise the sensor
    delay(100);

  if(!mag.begin())
//  {
//    There was a problem detecting the LSM303 ... check your connections
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
//    while(1);
//  }
    mag.begin();
    bool got_initial_position = initial_position();
    while(!got_initial_position) {
      got_initial_position = initial_position();
    }  
}
/*---------------------------------------------------------------------------------------*/
void waypoint::SetTime(char *pTime, char * pDate)
{
    //GPSfile = "mmddhhmm.CSV";
    strncpy(GPSfile,   pDate+2, 2);  // month
    strncpy(GPSfile+2, pDate, 2);    // day
    strncpy(GPSfile+4, pTime,2);     // GMT hour
    strncpy(GPSfile+6, pTime+2,2);   // minute
    Serial.println(GPSfile);

    strncpy(StartTime,     pDate+4, 2);  // year
    strncpy(StartTime+3,   pDate+2, 2);  // month
    strncpy(StartTime+6,   pDate, 2);    // day
    strncpy(StartTime+9,   pTime,2);     // GMT hour
    strncpy(StartTime+12,  pTime+2,2);   // minute
    strncpy(StartTime+15,  pTime+4,2);   // second
    strncpy(StartTime+18,  pTime+7,3);   // millisecond
}


/*---------------------------------------------------------------------------------------*/
void loop()
{
//    Serial.println("Inside C6 loop");
    
    unsigned long deltaT_ms;
    unsigned long time = millis();
    unsigned long endTime = time + LoopPeriod;
    unsigned long work_time = time;
    int PerCentBusy;
    char* pData;
    char* pGPS;
    char* pObstacles;
    bool GPS_available = GPS_reading.AcquireGPGGA(300);
    //IMU.Read(GPS_reading);

    // Added by Varsha - to get heading
    CurrentHeading = GetHeading();
 
    // Fuse all position estimates with a Fuzzy Filter 
    deltaT_ms = GPS_reading.time_ms - estimated_position.time_ms;
    estimated_position.fuse(GPS_reading, deltaT_ms);
    estimated_position.time_ms = GPS_reading.time_ms;

 // clear();

   data.clear();
   Serial2.end();
   Serial2.begin(baudrate);
   ps.dt = &data;
   ps.input = &Serial2;
   ps.output = &Serial2;
   
   // Read data from C2 using Elcano_Serial
  ParseStateError r = ps.update();
  Serial.println("done");
  Serial.println(static_cast<int8_t>(r));
  Serial.println(data.speed_cmPs);
  Serial.println(data.speed_cmPs);
  //data.write(&Serial1);
  long time_ms = millis();
   if(r == ParseStateError::success) 
    {
      
      Serial2.end();
      Serial2.begin(baudrate);
          
      newPos.speed_cmPs = data.speed_cmPs;
      newPos.bearing_deg = CurrentHeading;
      newPos.time_ms = time_ms;
    }
    else {
        //Extrapolate DR to present position 
        float theta = (90 - newPos.bearing_deg) * PI / 180;
        newPos.x_Pos_mm += (time_ms - newPos.time_ms) * 1000 * (newPos.speed_cmPs /10) * cos(theta);
        newPos.y_Pos_mm += (time_ms - newPos.time_ms) * 1000 * (newPos.speed_cmPs /10) * sin(theta);
    } 
      
      //Extrapolate GPS to present position
      GPS_reading.east_mm += (time_ms - GPS_reading.time_ms) * (newPos.speed_cmPs /10) * GPS_reading.Evector_x1000;
      GPS_reading.north_mm += (time_ms - GPS_reading.time_ms) * (newPos.speed_cmPs /10) * GPS_reading.Nvector_x1000;
      
    
      ComputePositionWithDR(oldPos, newPos);
    
      // Populate PositionData struct
      PositionData gps, fuzzy_out;
    
      gps.x_Pos_mm = estimated_position.east_mm;
      gps.y_Pos_mm = estimated_position.north_mm;
  
  
      
      // Translate GPS position
      TranslateCoordinates(newPos, gps, 1);
      RotateCoordinates(gps, newPos.bearing_deg, ROTATE_CLOCKWISE);
      FindFuzzyCrossPointXY(gps, newPos.distance_mm, newPos.bearing_deg, fuzzy_out);
      RotateCoordinates(fuzzy_out, newPos.bearing_deg, ROTATE_COUNTER_CLOCKWISE);
      TranslateCoordinates(oldPos, fuzzy_out, 1);
  
      // Update old position to new position 
      
      // speed already set
      float deltaX = estimated_position.east_mm - fuzzy_out.x_Pos_mm;
      float deltaY = estimated_position.north_mm - fuzzy_out.y_Pos_mm;
      float distance = sqrt(deltaX * deltaX + deltaY * deltaY);
      if(distance != 0) {
         estimated_position.Evector_x1000 = deltaX / distance; 
         estimated_position.Nvector_x1000 = deltaY / distance;
      }
      estimated_position.east_mm = fuzzy_out.x_Pos_mm;
      estimated_position.north_mm = fuzzy_out.y_Pos_mm;
    
    }
