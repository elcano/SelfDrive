#include <Common.h>
#include <SPI.h>
#include <SD.h>
#include <IODue.h>
#include <ElcanoSerial.h>
#include <Serial_Communication.h>
#include <Wire.h>
#include <Adafruit_LSM303_U.h>
#include <FusionData.h>
#include <Adafruit_GPS.h>

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences.
#define GPSECHO  true

#define mySerial Serial3
Adafruit_GPS GPS(&mySerial);


using namespace elcano;

/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);

enum States { STOP, STRAIGHT, ENTER_TURN, LEAVE_TURN, APPROACH_GOAL, LEAVE_GOAL};
States state;

#define GPSRATE 9600
#define DESIRED_SPEED_mmPs 1390 //5mph
#define SLOW_SPEED_mmPs 833 //1-2mph
#define currentlocation  -1 //currentLocation
#define CONES 1

extern bool DataAvailable;
bool got_GPS = false;
const long turn_speed = 500;
const long MIN_TURNING_RADIUS = 2400;
long speed_mmPs = DESIRED_SPEED_mmPs;
const unsigned long LoopPeriod = 100;  // msec

int next = 1; //index to path in a list
//last is the the last index of the Path/goal
int last_index_of_path = 3; //hardcode path of the last index/dest to 3 [cur,loc1,loc2,goal]
long current_heading = -1;
//pre-defined goal/destination to get to
long goal_lat[CONES] = {47760934};
long goal_lon[CONES] = { -122189963};
long pre_desired_speed = 0;
long turn_direction_angle = 0;
long pre_turn_angle = 0;
long extractSpeed = 0; //alternative to checksum since it's not implemented

//for calculating the E and N unit vector
double delta_east, delta_north, distance;

junction Nodes[MAX_WAYPOINTS]; //Storing the loaded map

//waypoint path[MAX_WAYPOINTS];  // course route to goal/mission
waypoint path[3];

waypoint mission[CONES]; //The taget node to reach/goal
waypoint GPS_reading, estimated_position, oldPos, newPos, fuzzy_out, Start;

//origin is set to the UWB map
Origin origin(47.758949, -122.190746);

SerialData ReceiveData, SendData;
ParseState ps, ps3;

int count = 0;
/*---------------------------------------------------------------------------------------*/
/**
   All C6 Methods start here
*/
/*---------------------------------------------------------------------------------------*/
void populatePath() {
  waypoint path0, path1, path2, path3;

  //needs to get the heading for all
  path0.latitude = 47.761116;
  path0.longitude = -122.190283;
  path0.Compute_mm(origin);

  path1.latitude = 47.761076;
  path1.longitude = -122.190266;
  path1.Compute_mm(origin);

  path2.latitude = 47.761043;
  path2.longitude = -122.190218;
  path2.Compute_mm(origin);


  path3.latitude = 47.761091;
  path3.longitude = -122.190218;
  path3.Compute_mm(origin);

  path[0] = path0;
  path[1] = path1;
  path[2] = path2;
  path[3] = path3;

}
void setup_GPS() {
  //Serial 3 is used for GPS
  mySerial.begin(9600);
  GPS.begin(9600);

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time

  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
}
bool AcquireGPS(waypoint &gps_position) {
  float latitude, longitude;

  char c;
  //read atleast 10 characters everyloop speed up update time
  for (int i = 0; i < 25; i++)
    c = GPS.read();

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return false;  // we can fail to parse a sentence in which case we should just wait for another

    if (GPS.fix) {


      gps_position.latitude = GPS.latitudeDegrees;
      gps_position.longitude = GPS.longitudeDegrees;

      //      Serial.println(GPS.latitudeDegrees, 6);
      //      Serial.println(GPS.longitudeDegrees, 6);




      return true;
    }
    return false;
  }
}

//setup Elcano serial communication for recieving data from C2
//Recieving actual_speed and an arbitary angle(for the moment)
void C6_communication_with_C2() {
  //setting up receiving data for C6 elcano communication
  ps.dt = &ReceiveData;
  ps.input = &Serial2;
  ps.output = &Serial2;
  ps.capture = MsgType::drive;
  ReceiveData.clear();
}

long getHeading(void) {
  //Get a new sensor event from the magnetometer
  sensors_event_t event;
  mag.getEvent(&event);

  //Calculate the current heading (angle of the vector y,x)
  //Normalize the heading
  float heading = (atan2(event.magnetic.y, event.magnetic.x) * 180) / PIf;

  if (heading < 0)  {
    heading = 360 + heading;
  }
  return heading;
}

//Get your first initial position form the GPS
void initial_position() {

  bool GPS_available = AcquireGPS(estimated_position);

  //Get initial starting position
  while (!GPS_available)
    GPS_available = AcquireGPS(estimated_position);

  estimated_position.time_ms = millis();
  estimated_position.Compute_EandN_Vectors(getHeading()); //get position E and N vector
  estimated_position.Compute_mm(origin);  //initialize north and east coordinates for position

  //oldPos to keep track of previous position for DR
  oldPos = estimated_position;
}

void setup_C6() {
  //setting up the GPS rate
  setup_GPS();

  //Enable auto-gain
  mag.enableAutoRange(true);

  //Initialise the sensor
  delay(100);

  if (!mag.begin()) {
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
  }
  mag.begin();

  initial_position(); //getting your initial position from GPS

  Serial.print("current east: "); Serial.println(estimated_position.east_mm);
  Serial.print("current north: "); Serial.println(estimated_position.north_mm);
  Serial.println();

  //for recieving data from C2
  C6_communication_with_C2();
}

void loop_C6() {
  got_GPS = AcquireGPS(GPS_reading);
  //try to get a new GPS position
  if (got_GPS) {
    GPS_reading.Compute_mm(origin); // get north and east coordinates from origin
    GPS_reading.Compute_EandN_Vectors(getHeading()); //Get E and N vector

    Serial.print("GPS east: "); Serial.println(GPS_reading.east_mm);
    Serial.print("GPS north: "); Serial.println(GPS_reading.north_mm);
    Serial.println();

  }
  //get heading coordinates from the compass
  current_heading = getHeading();
  newPos.time_ms = millis();

  //Recieving data from C2 using Elcano_Serial
  ParseStateError r = ps.update();
  if (r == ParseStateError::success)  {
    extractSpeed = receiveData(ReceiveData.speed_mmPs);

    //elecano serial doesn't have checksum, this will prevent crazy amount of acceleration
    //in less than a second
    if (abs(extractSpeed - newPos.speed_mmPs) < 100)
      newPos.speed_mmPs = receiveData(ReceiveData.speed_mmPs); //extract data

    newPos.bearing_deg = current_heading;

    //        Serial.print("speed from c2: "); Serial.print(newPos.speed_mmPs);
    //       Serial.print(", angle from c2: "); Serial.println(ReceiveData.angle_mDeg);
  }

  // calculate position using Dead Reckoning
  ComputePositionWithDR(oldPos, newPos);

  if (got_GPS) { //got both GPS and DeadReckoning
    Serial.println("using GPS/DR");

    //to get an esitimation position average between the GPS and Dead Rekoning
    //estimated_position is updated to the current position inside this method
    FindFuzzyCrossPointXY(GPS_reading, newPos, estimated_position);
    estimated_position.time_ms = newPos.time_ms;  //update time of that positon 

    //calculating the E and N unit vectors
    //note: estimated_position is updated from above
    delta_east = estimated_position.east_mm - oldPos.east_mm;
    delta_north = estimated_position.north_mm - oldPos.north_mm;
    distance = sqrt(delta_east * delta_east + delta_north * delta_north);

    if (distance != 0) {
      estimated_position.Evector_x1000 = 1000 * (delta_east / distance);
      estimated_position.Nvector_x1000 = 1000 * (delta_north / distance);
    }
  }
  else { //Did not get a GPS reading and only got DeadReckoning
    Serial.println("using DR");
    //calculating the E and N unit vector
    delta_east = newPos.east_mm - oldPos.east_mm;
    delta_north = newPos.north_mm - newPos.north_mm;
    distance = sqrt(delta_east * delta_east + delta_north * delta_north);

    if (distance != 0) {
      estimated_position.Evector_x1000 = 1000 * (delta_east / distance);
      estimated_position.Nvector_x1000 = 1000 * (delta_north / distance);
    }

    //update new current positon
    estimated_position.east_mm = newPos.east_mm;
    estimated_position.north_mm = newPos.north_mm;
    estimated_position.time_ms = newPos.time_ms;
  }
  //update oldPos to current positon
  oldPos = estimated_position;
  
  Serial.print("new east: "); Serial.println(estimated_position.east_mm);
  Serial.print("new north: "); Serial.println(estimated_position.north_mm);
  Serial.println();
}

/*---------------------------------------------------------------------------------------*/
/**
   All the Methods for C3 starts here
*/
/*---------------------------------------------------------------------------------------*/
//setup Elcano serial communication for sending data to C2
//Sending speed_mmps that you want to go at, and turn angle for direction
void C3_communication_with_C2() {
  //Setting up for sending data form C3 to C2
  ps3.dt = &SendData;
  ps3.input = &Serial2; //connection to read from
  ps3.output = &Serial2; //connection to write to
  ps3.capture = MsgType::drive;
  SendData.clear();
}

/**
   This method computes the turning radius for the trike
   by taking in the speed
*/
long turning_radius_mm(long speed_mmPs) {
  return 2 * MIN_TURNING_RADIUS;
}

/**
   This method checks to see if we have passed the destination and missed the turn.
   Also takes in consider of the type of path shape such as rectagle, skinny, square from current location
   to the intersection when determing if we passed the destination.

   param : n = next -> index of the intersection you're approaching
   return true = past the destination , false = Have not past the destination

*/
bool test_past_destination(int n) {
  if (abs(path[n - 1].east_mm - path[n].east_mm) > abs(path[n - 1].north_mm - path[n].north_mm)) {
    if (path[n].east_mm > path[n - 1].east_mm && estimated_position.east_mm > path[n].east_mm) {
      return true;
    }
    else if (path[n - 1].east_mm < path[n].east_mm && estimated_position.east_mm < path[n].east_mm) {
      return true;
    }
  }
  else {
    if (path[n].north_mm > path[n - 1].north_mm && estimated_position.north_mm > path[n].north_mm) {
      return true;
    }
    else if (path[n - 1].north_mm < path[n].north_mm && estimated_position.north_mm < path[n].north_mm) {
      return true;
    }
  }
  return false;
}

/**
   This method test for intersection from straight to entering turn

   param : turning radius of the vehicle
   param : n = next -> index of the intersection you're approaching
   return : true -> if approached intersection to entering turn, false -> otherwise
*/
bool test_approach_intersection(long turn_radius_mm, int n) {
  if (abs(path[n - 1].east_mm - path[n].east_mm) > abs(path[n - 1].north_mm - path[n].north_mm)) {
    if (path[n].east_mm > path[n - 1].east_mm && estimated_position.east_mm > path[n].east_mm + turn_radius_mm) {
      return true;
    }
    else if (path[n - 1].east_mm < path[n].east_mm && estimated_position.east_mm < path[n].east_mm - turn_radius_mm) {
      return true;
    }
  }
  else {
    if (path[n].north_mm > path[n - 1].north_mm && estimated_position.north_mm > path[n].north_mm + turn_radius_mm) {
      return true;
    }
    else if (path[n - 1].north_mm < path[n].north_mm && estimated_position.north_mm < path[n].north_mm - turn_radius_mm) {
      return true;
    }
  }
  return false;
}

/**
   This method test for intersection from leaving turn to straight

   param : turning radius of the vehicle
   param : n = next -> index of the intersection you're approaching
   return : true -> if approached intersection to leave turn, false -> otherwise
*/
bool test_leave_intersection(long turning_radius_mm, int n) {
  if (abs(path[n - 1].east_mm - path[n].east_mm) > abs(path[n - 1].north_mm - path[n].north_mm)) {
    if (path[n].east_mm > path[n - 1].east_mm && estimated_position.east_mm > path[n - 1].east_mm + turning_radius_mm) {
      return true;
    }
    else if (path[n - 1].east_mm < path[n].east_mm && estimated_position.east_mm < path[n - 1].east_mm - turning_radius_mm) {
      return true;
    }
  }
  else {
    if (path[n].north_mm > path[n - 1].north_mm && estimated_position.north_mm > path[n - 1].north_mm + turning_radius_mm) {
      return true;
    }
    else if (path[n - 1].north_mm < path[n].north_mm && estimated_position.north_mm < path[n - 1].north_mm - turning_radius_mm) {
      return true;
    }
  }
  return false;
}

/**
   The method find and set the state of the trike and
   also set the speed of the trike depending on the state it's in

   param : turning radius
   param : n = next -> index of the intersection you're approaching
*/
void find_state(long turn_radius_mm, int n) {

  switch (state) {
    case STRAIGHT:
      if (test_approach_intersection(turn_radius_mm, n)) {
        //Needs to find the last index of path
        if (n == last_index_of_path) {
          state = APPROACH_GOAL;
        }
        else {
          state = ENTER_TURN;
        }
        break;

      case STOP:
        break;

      case ENTER_TURN:
        //setting to turning speed
        speed_mmPs = turn_speed;

        if (test_past_destination(n)) {
          state = LEAVE_TURN;
          next++;

        }
        break;

      case LEAVE_TURN:
        //setting to turning speed
        speed_mmPs = turn_speed;
        if (test_leave_intersection(turn_radius_mm, n)) {
          state = STRAIGHT;
        }
        break;

      case APPROACH_GOAL:
        if (test_past_destination(n)) {
          state = STOP;
          speed_mmPs = 0;
        }
        else if (test_approach_intersection(turn_radius_mm, n)) {
          //changing to a slowwer speed
          speed_mmPs = SLOW_SPEED_mmPs;
        }
        break;

      case LEAVE_GOAL:
        break;
      }
  }

}

/**
   This method determines the direction of turns of the vehicle
   in degrees

   param : n = next -> index of the intersection you're approaching
   return : pos = right, neg = left, 0 = straight
*/
int get_turn_direction_angle(int n) {
  if (state == ENTER_TURN || state == LEAVE_TURN) {
    float turn_direction = (estimated_position.Evector_x1000 * path[n].Evector_x1000) + (estimated_position.Nvector_x1000 * path[n].Nvector_x1000);
    int turn_direction_angle = 0;
    turn_direction /= 1000000.0;
    turn_direction = acos(turn_direction); // angle in radians
    float cross = (estimated_position.Evector_x1000 * path[n].Nvector_x1000) - (estimated_position.Nvector_x1000 * path[n].Evector_x1000);
    turn_direction_angle = turn_direction * 180 / PI; //angle is degrees
    if (cross < 0)
      turn_direction_angle = -turn_direction_angle;

    return turn_direction_angle;
  }
}

void setup_C3() {
  //Serial.println("Start up C3 setup");
  //Trike state starts Straight
  state = STRAIGHT;
  //the path is set to approach the first intersection at index 1
  next = 1;

  //Setting up for sending data from C3 to C2
  void C3_communication_with_C2();
}

void loop_C3() {

  //Getting the turning radius
  long turn_radius_mm = turning_radius_mm(speed_mmPs);

  //Determining the state of the Trike
  find_state(turn_radius_mm, next);

  //Determining the turn direction for the trike "left, right or straight"
  turn_direction_angle = get_turn_direction_angle(next);


  //Send speed and angle to C2 to diplay the Led on the test stance
  //only send data to C2 if we get new data. Avoid sending the same data
  if (pre_desired_speed != speed_mmPs || pre_turn_angle != scaleDownAngle(turn_direction_angle)) {
    SendData.clear();
    SendData.kind = MsgType::drive;
    //chheck this
    SendData.speed_mmPs = sendData(speed_mmPs);
    SendData.angle_mDeg = scaleDownAngle(turn_direction_angle);
    SendData.write(&Serial2);

    pre_desired_speed = speed_mmPs;
    pre_turn_angle = scaleDownAngle(turn_direction_angle);
  }

  //If trike has reached goal
  //  Set desured speed to 0

}
void setup() {
  //for the micro SD
  pinMode(chipSelect, OUTPUT);

  Serial.begin(9600);
  Serial2.begin(9600);

  setup_C6();
  //hard-code path for C4
  //populatePath();
  setup_C3();
}

void loop() {
  count++;
  if (count == 1000) {
    speed_mmPs = 2225; //8
    turn_direction_angle = 40;
  }
  else if (count == 2000) {
    speed_mmPs = 2800; //10
    turn_direction_angle = 0;
  }
  else if (count == 3000) {
    speed_mmPs = 1390;  //5
    turn_direction_angle = -5;
  }


  // start our timer
  //  long time_start = millis();

  loop_C6();
  loop_C3();

  //  unsigned long time_finish_loop = millis();
  //
  //  //find out our time
  //  unsigned long time_elapsed = abs(time_finish_loop - time_start);


  //    while (time_elapsed < 100) {
  //      time_finish_loop = millis();
  //      time_elapsed = abs(time_finish_loop - time_start);
  //    }

}
