#include <Common.h>
#include <SPI.h>
#include <SD.h>
#include <IODue.h>
#include <ElcanoSerial.h>
#include <Wire.h>
#include <Adafruit_LSM303_U.h>
#include <FusionData.h>
#include <Adafruit_GPS.h>

//Creating GPS object 
Adafruit_GPS GPS(&Serial3);
/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303_Mag_Unified mag(1366123);

using namespace elcano;

enum States { STOP, STRAIGHT, ENTER_TURN, LEAVE_TURN, APPROACH_GOAL, LEAVE_GOAL};
States state;

#define DESIRED_SPEED_mmPs 2235
const long default_speed_mmPs = DESIRED_SPEED_mmPs;
const long slow_speed_mmPs = 100;
const long turn_speed = 500;
const long MIN_TURNING_RADIUS = 2400;
long speed_mmPs = default_speed_mmPs;
//index to path in a list
int next = 1;

waypoint path[MAX_WAYPOINTS];  // course route to goal/mission

#define GPSRATE 9600

File dataFile;
char GPSfile[BUFFSIZ] = "mmddhhmm.csv";
char ObstacleString[BUFFSIZ];
char StartTime[BUFFSIZ] = "yy,mm,dd,hh,mm,ss,xxx";
const char TimeHeader[] = "year,month,day,hour,minute,second,msec";
const char* RawKF = "Raw GPS data,,,,,,Kalman Filtered data";
const char* Header = "Latitude,Longitude,East_m,North_m,SigmaE_m,SigmaN_m,Time_s,";
const char* ObstHeader = "Left,Front,Right,Busy";

long CurrentHeading = -1;
SerialData data, C3Results;
ParseState ps, ps3;

/* time (micro seconds) for a wheel revolution */
//volatile long Odometer_mm = 0;
//volatile long SpeedCyclometer_mmPs;
//// Speed in degrees per second is independent of wheel size.
//volatile long SpeedCyclometer_degPs;

// waypoint mission[MAX_WAYPOINTS];
waypoint GPS_reading;
waypoint estimated_position;
//instrument IMU;
const unsigned long LoopPeriod = 100;  // msec

PositionData oldPos, newPos;

//last is the the last index of the Path. Path[last] is the destination
int last_index_of_path;

#define currentlocation  -1 //currentLocation

extern bool DataAvailable;

#define CONES 1
long goal_lat[CONES] = {47760934};
long goal_lon[CONES] = { -122189963};
//long goal_lat[CONES] = {  47621881,   47621825,   47623144,   47620616,   47621881};
//long goal_lon[CONES] = {-122349894, -122352120, -122351987, -122351087, -122349894};
/*  mph   mm/s
     3    1341f
     5    2235
     8    3576
     10   4470
     15   6706
     20   8941
     25  11176
     30  13411
     45  20117
     60  26822
*/

extern int map_points =  5;//16;
junction Nodes[MAX_WAYPOINTS];
struct AStar
{
  int ParentID;
  long CostFromStart;
  long CostToGoal;
  long TotalCost;
} Open[MAX_WAYPOINTS];


waypoint Origin, Start;

waypoint mission[CONES];  // aka MDF //The target Nodes to hit

/*---------------------------------------------------------------------------------------*/
/**
   All C6 Methods start here
*/
/*---------------------------------------------------------------------------------------*/
long GetHeading(void) {
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

  if (heading < 0)  {
    heading = 360 + heading;
  }
  // Converting heading to x1000
  return ((long)(heading * HEADING_PRECISION));
  //    return heading;
}
/*---------------------------------------------------------------------------------------*/
bool initial_position() {
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



  GPS_available = estimated_position.AcquireGPGGA(300);
  if (!GPS_available) {
    GPS_available = estimated_position.AcquireGPRMC(300);
    if (GPS_available) {
      estimated_position.sigma_mm = 1.0E4; // 10 m standard deviation
      Serial.println("OK");
    }
  }
  //Checking to see the we are able to get the GPS latitude and lontitude
  Serial.print("Gps latitude: ");
  Serial.println(estimated_position.latitude);
  Serial.print("GPS longtitude: ");
  Serial.println(estimated_position.longitude);

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
  //{
  /* If (message from C4)
    {
    ReadState(C4);  // get initial route and speed
    }
    Read GPS, compass and IMU and update their estimates.
  */
  //  }

  // GPS_available = GPS_reading.AcquireGPGGA(300);
  // ready to roll
  // Fuse all position estimates.
  // Send vehicle state to C3 and C4.

  return GPS_available;
}

void setup_C6() {
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
  GPS.begin(9600);
  Serial3.begin(GPSRATE); // GPS

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  // GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time

  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(chipSelect, OUTPUT);

  Serial.println("Start C6 setup");
  
  oldPos.Clear();
  oldPos.time_ms = millis();

  //Enable auto-gain
  mag.enableAutoRange(true);

  //Initialise the sensor
  delay(100);

  if (!mag.begin()) {
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
  }

  mag.begin();
  bool got_initial_position = initial_position();
  while (!got_initial_position) {
    got_initial_position = initial_position();

  }
  Serial.println("Got the GPS Signal");

  //setting up C6 elcano communication
  data.clear();
  Serial2.end();
  Serial2.begin(baudrate);
  ps.dt = &data;
  ps.input = &Serial2;
  ps.output = &Serial2;
  ps.capture = MsgType::drive;

  Serial.println("Finish C6 setup");
}

void waypoint::SetTime(char *pTime, char * pDate) {
  //GPSfile = "mmddhhmm.CSV";
  strncpy(GPSfile,   pDate + 2, 2); // month
  strncpy(GPSfile + 2, pDate, 2);  // day
  strncpy(GPSfile + 4, pTime, 2);  // GMT hour
  strncpy(GPSfile + 6, pTime + 2, 2); // minute
  Serial.println(GPSfile);

  strncpy(StartTime,     pDate + 4, 2); // year
  strncpy(StartTime + 3,   pDate + 2, 2); // month
  strncpy(StartTime + 6,   pDate, 2);  // day
  strncpy(StartTime + 9,   pTime, 2);  // GMT hour
  strncpy(StartTime + 12,  pTime + 2, 2); // minute
  strncpy(StartTime + 15,  pTime + 4, 2); // second
  strncpy(StartTime + 18,  pTime + 7, 3); // millisecond
}

void loop_C6() {
  Serial.println("Start of C6 loop");
  unsigned long deltaT_ms;
  unsigned long time = millis();
  unsigned long endTime = time + LoopPeriod;
  unsigned long work_time = time;
  int PerCentBusy;
  char* pData;
  char* pGPS;
  char* pObstacles;

  //trying to retrieve a new estimated position form the GPS reading
  bool GPS_available = GPS_reading.AcquireGPGGA(30);

  //Use GPRMC if GPGGA failed
  if (!GPS_available) {
    GPS_available = GPS_reading.AcquireGPRMC(30);
    if (GPS_available) {
      //do i need this part below??
      GPS_reading.sigma_mm = 1.0E4; // 10 m standard deviation
      Serial.println("OK Got updated postion from GPS");
    }
  }

  CurrentHeading = GetHeading();

  // Fuse all position estimates with a Fuzzy Filter
  deltaT_ms = GPS_reading.time_ms - estimated_position.time_ms;
  estimated_position.fuse(GPS_reading, deltaT_ms);
  estimated_position.time_ms = GPS_reading.time_ms;

  // Read data from C2 using Elcano_Serial
  //Asynchronous using Elcano_Serial
  ParseStateError r = ps.update();
  long time_ms = millis();
  if (r == ParseStateError::success)  {
    Serial.println("done");
    Serial.println(static_cast<int8_t>(r));
    Serial.println(data.speed_cmPs);

    newPos.speed_cmPs = data.speed_cmPs;
    newPos.bearing_deg = CurrentHeading;
    newPos.time_ms = time_ms;
  }
  else  {
    //Extrapolate DR to present position
    float theta = (90 - newPos.bearing_deg) * PI / 180;
    newPos.x_Pos_mm += (time_ms - newPos.time_ms) * 1000 * (newPos.speed_cmPs / 10) * cos(theta);
    newPos.y_Pos_mm += (time_ms - newPos.time_ms) * 1000 * (newPos.speed_cmPs / 10) * sin(theta);
  }

  //Extrapolate GPS to present position
  GPS_reading.east_mm += (time_ms - GPS_reading.time_ms) * (newPos.speed_cmPs / 10) * GPS_reading.Evector_x1000;
  GPS_reading.north_mm += (time_ms - GPS_reading.time_ms) * (newPos.speed_cmPs / 10) * GPS_reading.Nvector_x1000;

  // Update old position to new position
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

  // speed already set
  float deltaX = estimated_position.east_mm - fuzzy_out.x_Pos_mm;
  float deltaY = estimated_position.north_mm - fuzzy_out.y_Pos_mm;
  float distance = sqrt(deltaX * deltaX + deltaY * deltaY);
  if (distance != 0) {
    estimated_position.Evector_x1000 = deltaX / distance;
    estimated_position.Nvector_x1000 = deltaY / distance;
  }
  estimated_position.east_mm = fuzzy_out.x_Pos_mm;
  estimated_position.north_mm = fuzzy_out.y_Pos_mm;

  Serial.println("End of C6 loop");
}/*---------------------------------------------------------------------------------------*/
/**
   All the Methods for C4 starts here
*/
/*---------------------------------------------------------------------------------------*/
// Fill the distances of the junctions in the MAP
void ConstructNetwork(junction *Map, int MapPoints) {
  double deltaX, deltaY;
  int destination;
  for (int i = 0; i < MapPoints; i++) {
    if ( Map[i].east_mm == INVALID)  continue;
    for (int j = 0;  j < 4; j++)  {
      destination = Map[i].destination[j];
      if (destination == END) continue;
      deltaX = Map[i].east_mm;
      deltaX -= Map[destination].east_mm;
      deltaY = Map[i].north_mm;
      deltaY -= Map[destination].north_mm;

      Map[i].Distance[j] += sqrt(deltaX * deltaX + deltaY * deltaY); //in rough scale
    }
  }
}
/*---------------------------------------------------------------------------------------*/
// Set up mission structure from cone latitude and longitude list.
void GetGoals(junction *nodes , int Goals)  {
  double deltaX, deltaY, Distance;
  for (int i = 0; i < CONES; i++) {
    mission[i].latitude = goal_lat[i];
    mission[i].longitude = goal_lon[i];
    mission[i].Compute_mm();
    mission[i].speed_mmPs = DESIRED_SPEED_mmPs;
    mission[i].index = 1 | GOAL;
    mission[i].sigma_mm = 1000;
    mission[i].time_ms = 0;

    if (i == 0) { //If CONE == 1
      mission[i].Evector_x1000 = 1000;
      mission[i].Nvector_x1000 = 0;
    }
    else  {
      deltaX = mission[i].east_mm - mission[i - 1].east_mm;
      deltaY = mission[i].north_mm - mission[i - 1].north_mm;
      Distance = sqrt(deltaX * deltaX + deltaY * deltaY);
      mission[i - 1].Evector_x1000 = (deltaX * 1000.) / Distance;
      mission[i - 1].Nvector_x1000 = (deltaY * 1000.) / Distance;
    }
    if (i == CONES - 1) {
      mission[i].Evector_x1000 = mission[i - 1].Evector_x1000;
      mission[i].Nvector_x1000 = mission[i - 1].Nvector_x1000;
      mission[i].index |= END;
    }
  }
}
/*---------------------------------------------------------------------------------------*/
// Find the distance from (east_mm, north_mm) to a road segment Nodes[i].distance[j]
// return distance in mm, and per cent of completion from i to j.
// distance is negative if (east_mm, north_mm) lies to the left of the road
// when road direction is from i to j

// Compare this routine to distance() in C3 Pilot
//k =  index into Nodes[]
//east_mm : current
long distance(int cur_node, int *k,  long cur_east_mm, long cur_north_mm, int* perCent) {
  float deltaX, deltaY, dist_mm;
  int cur , destination;
  long Eunit_x1000, Nunit_x1000;
  long closest_mm = MAX_DISTANCE;
  long Road_distance, RoadDX_mm, RoadDY_mm;

  long pc; //per cent of completion from i to j.

  *perCent = 0;
  *k = 0;
  closest_mm = MAX_DISTANCE;


  for (cur = 0; cur < 4; cur++) { // Don't make computations twice.
    destination = Nodes[cur_node].destination[cur];
    if (destination == 0 || destination < cur_node) continue;  //replace Destination with END
    // compute road unit vectors from i to cur
    RoadDX_mm = Nodes[destination].east_mm - Nodes[cur_node].east_mm;

    //      Serial.println("RoadX_mm " + String(RoadDX_mm));
    //      Serial.println("Destination " + String(destination));
    //      Serial.println("Nodes[destination].east_mm " + String(Nodes[destination].east_mm));
    //      Serial.println("-Nodes[cur_loc].east_mm " + String(-Nodes[cur_node].east_mm));
    //
    int Eunit_x1000 = RoadDX_mm  * 1000 / Nodes[cur_node].Distance[cur];

    RoadDY_mm =  Nodes[destination].north_mm - Nodes[cur_node].north_mm;
    //
    //      Serial.println("RoadY_mm " + String(RoadDY_mm));
    //      Serial.println("Nodes[destination].north_mm " + String(Nodes[destination].north_mm));
    //      Serial.println("-Nodes[cur_loc].north_mm " + String(-Nodes[cur_node].north_mm));
    //
    int Nunit_x1000 = RoadDY_mm * 1000 / Nodes[cur_node].Distance[cur];
    //
    //      // normal vector is (Nunit, -Eunit)
    //      //Answers: What would be the change in X/Y from my current Node.
    //      deltaX = cur_east_mm - Nodes[cur_node].east_mm;
    //      deltaY = cur_north_mm - Nodes[cur_node].north_mm;
    //
    //
    //      // sign of return value gives which side of road it is on.
    //      Road_distance = (-deltaY * Eunit_x1000 + deltaX * Nunit_x1000) / 1000;
    Road_distance = sqrt( (RoadDX_mm * RoadDX_mm) + (RoadDY_mm * RoadDY_mm));
    //Why do percentage computation like this?
    pc = (deltaX * Eunit_x1000 + deltaY * Nunit_x1000) / (Nodes[cur_node].Distance[cur] * 10);

    //     Serial.println("Closest_mm " + String(closest_mm) + "\t Road_distance " + String(Road_distance));
    //     Serial.println("Road Distance " + String(Road_distance));
    //     Serial.println("closest Distance " + String(closest_mm));
    if (abs(Road_distance) < abs(closest_mm) && pc >= 0 && pc <= 100) {
      closest_mm = Road_distance;
      *k = destination;
      *perCent = pc;

    }
  }
  return closest_mm;
}
/*---------------------------------------------------------------------------------------*/
//Figuring out a path to get the road network
void FindClosestRoad(waypoint *start, waypoint *road) {  //populate road with best road from start
  long closest_mm = MAX_DISTANCE;
  long dist;
  int close_index;
  int perCent;
  long done = 1;//2000;
  int i, node_successor;

  for (i = 0; i < 5/*map_points*/; i++) { // find closest road.
    dist = distance(i, &node_successor, start->east_mm, start->north_mm, &perCent); //next node to visit
    Serial.println("Start : Latitude " + String(start->latitude) + "\t Longitude " + String(start->longitude) + "\t Dist "
                   + String(dist));

    if (abs(dist) < abs(closest_mm))  {
      close_index = node_successor;
      closest_mm = dist;
      done = 1;// perCent; //Not really true amount of nodes done?
      road->index = i;
      road->sigma_mm = node_successor;
    }
  }
  if (closest_mm < MAX_DISTANCE)  {
    i = road->index; //0
    node_successor = close_index; //0
    road->east_mm =  Nodes[i].east_mm  +  done * (Nodes[node_successor].east_mm  - Nodes[i].east_mm) / 100;
    road->north_mm = Nodes[i].north_mm + done * (Nodes[node_successor].north_mm - Nodes[i].north_mm) / 100;
  }
  else  {
    for (i = 0; i < 5/*map_points*/; i++) { // find closest node
      //Serial.println("I got here");
      dist = start->distance_mm(Nodes[i].east_mm, Nodes[i].north_mm);

      if (dist < closest_mm)  {
        close_index = i;
        closest_mm = dist;
      }
    }
    road->index = road->sigma_mm = close_index;
    road->east_mm =  Nodes[close_index].east_mm;
    road->north_mm = Nodes[close_index].north_mm;
  }

  road->Evector_x1000 = 1000;
  road->Nvector_x1000 = 0;
  road->time_ms = 0;
  road->speed_mmPs = DESIRED_SPEED_mmPs;

  //Test FindClosest Road:
  Serial.println("Distance " + String(dist));
  Serial.println("Road :  East_mm " + String(road->east_mm) + "\t North_mm " + String(road->north_mm));

}

//Test ClosestRoad:
void test_closestRoad() {

  waypoint roadOrigin;
  waypoint roadDestination;

  Serial.println("First " );
  FindClosestRoad(&mission[0], &roadOrigin);

  Serial.println();

  Serial.println("Second ");
  FindClosestRoad(&mission[3], &roadDestination);
  for (int last = 0; last < 4; last++) {
    Serial.println(" mission " + String(mission[last].east_mm ) + "\t roadOrigin " + String(roadOrigin.east_mm));
    Serial.println(" mission " + String(mission[last].longitude ) + "\t roadOrigin " + String(roadOrigin.east_mm));
  }
  for (int last = 0; last < 4; last++) {
    Serial.println(" mission " + String(mission[last].east_mm ) + "\t roadOrigin " + String(roadDestination.east_mm));
    Serial.println(" mission " + String(mission[last].longitude ) + "\t roadOrigin " + String(roadDestination.north_mm));
  }
}
/*---------------------------------------------------------------------------------------*/
// start and destination are on the road network given in Nodes.
// start is in Path[1].
// Place other junction waypoints into Path.
// Returned value is next index into Path.
// start->index identifies the closest node.
// sigma_mm holds the index to the other node.
// A* is traditionally done with pushing and popping node from an Open and Closed list.
// Since we have a small number of nodes, we instead reserve a slot on Open and Closed
// for each node.

int BuildPath (int j, waypoint* start, waypoint* destination) { // Construct path backward to start.
  Serial.println("To break");
  int last = 1;
  int route[map_points];
  int i, k, node;
  long dist_mm;

  k = map_points - 1;
  route[k] = j;

  while (Open[j].ParentID != currentlocation) {
    j = route[--k] = Open[j].ParentID;
  }

  path[last] = start;
  for ( ; k < map_points; k++)  {
    node = route[k];
    path[++last].east_mm = Nodes[node].east_mm;
    path[last].north_mm  = Nodes[node].north_mm;
  }
  path[++last] = destination;
  for (k = 0; k <= last; k++) {
    if (k > 0) path[k].sigma_mm = 10; // map should be good to a cm.
    path[k].index = k;
    path[k].speed_mmPs = DESIRED_SPEED_mmPs;
    path[k].Compute_LatLon();  // this is never used
  }
  last++;
  for (j = 0; j < last - 1; j++)  {
    path[j].vectors(&path[j + 1]);
  }

  return last;
}
/*---------------------------------------------------------------------------------------*/

void test_buildPath() {

  //  BuildPath(0, Path, destination);
}
//Usa A star
int FindPath(waypoint *start, waypoint *destination)  { //While OpenSet is not empty

  Serial.println("Start East_mm " + String(start->east_mm) + "\t North " + String(start->north_mm));
  Serial.println("Start East_mm " + String(destination->east_mm) + "\t North " + String(destination->north_mm));
  long ClosedCost[map_points];
  int  i, neighbor, k;
  long NewCost, NewStartCost, NewCostToGoal;
  long NewIndex;
  long BestCost, BestID;
  bool Processed = false;

  for (i = 0; i < map_points; i++)  { // mark all nodes as empty
    Open[i].TotalCost = MAX_DISTANCE;
    ClosedCost[i] = MAX_DISTANCE;
  }

  i = start->index; // get successor nodes of start
  Open[i].CostFromStart = start->distance_mm(Nodes[i].east_mm, Nodes[i].north_mm);
  Open[i].CostToGoal = destination->distance_mm(Nodes[i].east_mm, Nodes[i].north_mm);

  Open[i].TotalCost = Open[i].CostFromStart + Open[i].CostToGoal;
  Open[i].ParentID = currentlocation;

  i = start->sigma_mm;
  Open[i].CostFromStart = start->distance_mm(Nodes[i].east_mm, Nodes[i].north_mm);
  Open[i].CostToGoal = destination->distance_mm(Nodes[i].east_mm, Nodes[i].north_mm);
  Open[i].TotalCost = Open[i].CostFromStart + Open[i].CostToGoal;

  Open[i].ParentID = currentlocation;
  while (BestCost < MAX_DISTANCE) { //While OpenSet is not empty
    BestCost = MAX_DISTANCE;
    BestID = -1;
    // pop lowest cost node from Open; i.e. find index of lowest cost item
    for (i = 0; i < 6; i++) {
      if (Open[i].TotalCost < BestCost) {
        BestID = i;
        //            Serial.println("BESTID " + String(BestID));
        BestCost = Open[i].TotalCost;
        //            Serial.println("BestCost " + String(BestCost));
      }

    }
    if (BestID < 0) {
      return INVALID;
    }
    Serial.println("BESTID " + String(BestID));
    Serial.println("DestinationINdex " + String(destination->index));
    Open[BestID].TotalCost = MAX_DISTANCE;  // Remove node from "stack".
    if (BestID == destination->index || BestID == destination->sigma_mm)  { // Done:: reached the goal!!

      return BuildPath(BestID, start, destination);   // Construct path backward to start.
    }

    i = BestID;  // get successor nodes from map

    for (neighbor = 0; neighbor < 5; neighbor++)  {
      NewIndex = Nodes[i].destination[neighbor];

      if (NewIndex == END)continue; // No success in this slot

      NewStartCost =  Open[i].CostFromStart + Nodes[i].Distance[neighbor];
      NewCostToGoal = destination->distance_mm(Nodes[NewIndex].east_mm, Nodes[NewIndex].north_mm);
      NewCost = NewStartCost + NewCostToGoal;

      if (NewCost >= ClosedCost[NewIndex]) // check if this node is already on Open or Closed.
        continue;  // Have already looked at this node

      else if (ClosedCost[NewIndex] != MAX_DISTANCE) { // looked at this node before, but at a higher cost
        ClosedCost[NewIndex] = MAX_DISTANCE;  // remove node from Closed
      }
      if (NewCost >= Open[NewIndex].TotalCost)
        continue;   // This node is a less efficient way of getting to a node on the list
      // Push successor node onto stack.

      Open[NewIndex].CostFromStart = NewStartCost;
      Open[NewIndex].CostToGoal = NewCostToGoal;
      Open[NewIndex].TotalCost = NewCost;
      Open[NewIndex].ParentID = i;
    }  // end of successor nodes


    ClosedCost[BestID] =  BestCost; // Push node onto Closed
  }

  Serial.println("Destination East_mm " + String(destination->east_mm) + "\t North " + String(destination->north_mm));

  return 0;  // failure
}

/*---------------------------------------------------------------------------------------*/
// Low level path is a straight line from start to detination.
// PathPlan makes an intermediate level path that uses as many roads as possible.
//start = currentlocation: destination = heading to;
//Find the cloeset road and call Findpath to do the A star
int PlanPath (waypoint *start, waypoint *destination) {

  //Serial.println("Start : East_mm = " + String(start->east_mm) + "\t North_mm =  " + String(start->north_mm));
  waypoint roadOrigin, roadDestination;

  int last = 0;
  path[0] = start;
  path[0].index = 0;

  FindClosestRoad( start, &roadOrigin );
  FindClosestRoad( destination, &roadDestination );

  int w = abs(start->east_mm  - roadOrigin.east_mm) + abs(start->north_mm - roadOrigin.north_mm);
  int x = abs(destination->east_mm  - roadDestination.east_mm)  + abs(destination->north_mm - roadDestination.north_mm);

  int straight_dist = 190 * abs(start->east_mm  - destination->east_mm) +  abs(start->north_mm - destination->north_mm);
  if (w + x >= straight_dist) { // don't use roads; go direct
    last = 1;
    Serial.println("In Straight");
  }
  else {  // use A* with the road network
    Serial.println("In Else");
    path[1] = roadOrigin;
    path[1].index = 1;
    //why index = 7?
    destination -> index = 7;
    last = FindPath(&roadOrigin, &roadDestination);
  }

  path[last] = destination;
  path[last - 1].vectors(&path[last]);
  path[last].Evector_x1000 = path[last - 1].Evector_x1000;
  path[last].Nvector_x1000 = path[last - 1].Nvector_x1000;
  path[last].index = last | END;

  //    Serial.println("Destination : East_mm = " + String(destination->east_mm) + "\t North_mm =  " + String(destination->north_mm));
  Serial.println();

  return last;

}
/*---------------------------------------------------------------------------------------*/
// LoadMap
// Loads the map nodes from a file.
// Takes in the name of the file to load and loads the appropriate map.
// Returns true if the map was loaded.
// Returns false if the load failed.
boolean LoadMap(char* fileName) {
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File myFile = SD.open(fileName, FILE_READ);

  // if the file opened okay, read from it:
  if (myFile) {
    Serial.println("loading nearest Map....");
    // Initialize a string buffer to read lines from the file into
    // Allocate an extra char at the end to add null terminator
    char* buffer = (char*)malloc(myFile.size() + 1);

    // index for the next character to read into buffer
    char* ptr = buffer;

    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      *ptr = myFile.read();
      ++ptr;
    }
    // Null terminate the buffer string
    *ptr = '\0';

    // Set up tokenizer for the file buffer string
    char * delimiter = " ,\n";
    char* token;
    int col = 0;
    int row = 0;

    // get the first token
    Serial.println(token);
    token = strtok(buffer, delimiter);
    Serial.println("got first token");
    Serial.println(token);

    // walk through other tokens
    while ( token != NULL ) {
      switch (col % 10) {
        case 0:  // latitude
          Nodes[row].east_mm = atol(token);

          col++;
          break;

        case 1:  // longitude
          Nodes[row].north_mm = atol(token);
          col++;
          break;

        case 2:  // filename
          if (token == "END") {
            Nodes[row].destination[0] = END;
          }
          else  {
            Nodes[row].destination[0] = atoi(token);
          }
          col++;
          break;

        case 3:  // filename
          if (token == "END") {
            Nodes[row].destination[1] = END;
          }
          else  {
            Nodes[row].destination[1] = atoi(token);
          }
          col++;
          break;

        case 4:  // filename
          if (token == "END") {
            Nodes[row].destination[2] = END;
          }
          else  {
            Nodes[row].destination[2] = atoi(token);
          }
          col++;
          break;

        case 5:  // filename
          if (token == "END") {
            Nodes[row].destination[3] = END;
          }
          else  {
            Nodes[row].destination[3] = atoi(token);
          }
          col++;
          break;

        case 6:  // filename
          Nodes[row].Distance[0] = atol(token);
          col++;
          break;

        case 7:  // filename
          Nodes[row].Distance[1] = atol(token);
          col++;
          break;

        case 8:  // filename
          Nodes[row].Distance[2] = atol(token);
          col++;
          break;

        case 9:  // filename
          Nodes[row].Distance[3] = atol(token);

          //check this method
          convertLatLonToMM(Nodes[row].east_mm, Nodes[row].north_mm);
          col++;
          row++;
          break;

        default:  // unexpected condition; print error
          Serial.print("Unexpected error happened while reading map description file.");
          Serial.print("Please verify the file is in the correct format.");
          Serial.println("Planner may not work correctly if this message appears.");
          break;
      }

      token = strtok(NULL, delimiter);
      Serial.println(token);
    }
    map_points = row;
    Serial.println("Test map");
    Serial.println(map_points);
    //To test LoadMap:
    for (int i = 0; i < map_points; i++)  {
      Serial.print("inside the loop: ");
      Serial.println(i);
      Serial.print(Nodes[i].east_mm);
      Serial.print(",");
      Serial.print(Nodes[i].north_mm);
      Serial.print(",");
      Serial.print(Nodes[i].destination[0]);
      Serial.print(",");
      Serial.print(Nodes[i].destination[1]);
      Serial.print(",");
      Serial.print(Nodes[i].destination[2]);
      Serial.print(",");
      Serial.print(Nodes[i].destination[3]);
      Serial.print(",");
      Serial.print(Nodes[i].Distance[0]);
      Serial.print(",");
      Serial.print(Nodes[i].Distance[1]);
      Serial.print(",");
      Serial.print(Nodes[i].Distance[2]);
      Serial.print(",");
      Serial.println(Nodes[i].Distance[3]);
    }

    // If file loaded, read data into Nodes[]
    free(buffer);
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    myFile.close();
    Serial.println("error opening");
    Serial.println(fileName);
    return false;
  }
  return true;
}

/*---------------------------------------------------------------------------------------*/
// SelectMap:  Return the file name of the closest origin
// Determines which map to load.
// Takes in the current location as a waypoint and a string with the name of the file that
//   contains the origins and file names of the maps.
// Determines which origin is closest to the waypoint and returns it as a junction.
// Assumes the file is in the correct format according to the description above.
void SelectMap(waypoint currentLocation, char* fileName, char* nearestMap)
{
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File myFile = SD.open(fileName, FILE_READ);

  // if the file opened okay, read from it:
  if (myFile) {
    // Initialize a string buffer to read lines from the file into
    // Allocate an extra char at the end to add null terminator
    char* buffer = (char*)malloc(myFile.size() + 1);

    //Serial.println("Printing file info");
    // index for the next character to read into buffer
    char* ptr = buffer;

    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      *ptr = myFile.read();
      ++ptr;
    }

    // Null terminate the buffer string
    *ptr = '\0';

    // Set up storage for coordinates and file names
    // Note: we malloc here because the stack is too small
    float* map_latitudes = (float*)malloc(MAX_MAPS * 4);
    float* map_longitudes = (float*)malloc(MAX_MAPS * 4);
    char** map_file_names = (char**)malloc(MAX_MAPS * sizeof(char*));

    for (int i = 0; i < MAX_MAPS; i++)  {
      // initialize using invalid values so that we can ensure valid data in allocated memory
      map_latitudes[i] = 91;
      map_longitudes[i] = 181;
      map_file_names[i] = "";
    }

    // Set up tokenizer for the file buffer string
    char *delimiter = " ,\n";
    char* token;
    int col = 0;
    int row = 0;
    char* t;


    // get the first token
    token = strtok(buffer, delimiter);

    // fill the map_latitude, map_longitude, & map_file with tokens
    while ( token != NULL ) {
      switch (col % 3)  {
        case 0:  // latitude
          map_latitudes[row] = atof(token);

          Serial.print("index: ");
          Serial.println(row);
          Serial.println(map_latitudes[row]);
          col++;
          break;

        case 1:  // longitude
          map_longitudes[row] = atof(token);
          col++;
          Serial.print("index: ");
          Serial.println(row);
          Serial.println(map_longitudes[row]);
          break;

        case 2:  // filename
          map_file_names[row] = token;
          col++;
          Serial.print("index: ");
          Serial.println(row);
          Serial.println(map_file_names[row]);
          row++;
          break;

        default:  // unexpected condition; print error
          Serial.println("Unexpected error happened while reading map description file. Please verify the file is in the correct format. Planner may not work correctly if this message appears.");
          break;
      }
      token = strtok(NULL, delimiter);
    }
    float currentLat = currentLocation.latitude / 1000000.0;
    while (currentLat > 100) {
      currentLat /= 10;
    }
    float currentLong = currentLocation.longitude / 1000000.0;
    while (abs(currentLong) > 1000) {
      currentLong /= 10;
    }
    //Serial.println(currentLat, 8);
    //Serial.println(currentLong, 8);

    int closestIndex = -1;
    long closestDistance = MAX_DISTANCE;
    for (int i = 0; i < MAX_MAPS; i++)  {
      int dist = fabs((map_latitudes[i] - currentLat)) +
                 abs(map_longitudes[i] - currentLong);
      if (dist < closestDistance) {
        closestIndex = i;
        closestDistance = dist;
      }
    }
    if (closestIndex >= 0)  {
      // Determine closest map to current location
      // Update Origin global variable
      Origin.latitude = map_latitudes[closestIndex];
      Origin.longitude = map_longitudes[closestIndex];
      for (int i = 0; i < 13; i++)
      {
        nearestMap[i] = map_file_names[closestIndex][i];
      }
    }
    else  {
      Serial.println("error determining closest map.");
    }
    // Free the memory allocated for the buffer
    free(buffer);

    free(map_latitudes);
    free(map_longitudes);
    free(map_file_names);

    // close the file:
    myFile.close();
    Serial.println("Map definitions loaded.");
  } else {
    // if the file didn't open, print an error:
    myFile.close();
    Serial.println("error opening MAP_DEFS.txt");
  }

}

/*---------------------------------------------------------------------------------------*/
void initialize_C4() {
  int last;
  Origin.Evector_x1000 = INVALID;
  Origin.Nvector_x1000 = INVALID;
  Origin.east_mm = 0;
  Origin.north_mm = 0;
  Origin.latitude = INVALID;
  Origin.longitude = INVALID;

  Start.east_mm = 0;
  Start.north_mm = 0;

  //Store the initial GPS latitude and longtitude to select the correct map
  Start.latitude = estimated_position.latitude;
  Start.longitude = estimated_position.longitude;

  Serial.print("Start latitude: ");
  Serial.println(Start.latitude);
  Serial.print("Start longitude: ");
  Serial.println(Start.longitude);

  Serial.println("Initializing SD card...");
  pinMode(SS, OUTPUT);

  if (!SD.begin(chipSelect)) {
    Serial3.println("initialization failed!");
  }
  Serial.println("initialization done.");
  char nearestMap[13] = "";

  SelectMap(Start, "MAP_DEFS.txt", nearestMap); //populates info from map_def to nearestMap
  Serial.println("SelectMap done");

  Serial.print("Nearest Map: ");
  Serial.println(nearestMap);

  //populate nearest map in junction Nodes structure
  LoadMap(nearestMap);
  Serial.println("Finished LoadingMap");

  //takes in the Nodes that contains all of the map
  ConstructNetwork(Nodes, map_points); //To fill out the rest of the nodes info
  /* Convert latitude and longitude positions to flat earth coordinates.
     Fill in waypoint structure  */
  GetGoals(Nodes, CONES);
}

//Test mission::a list of waypoints to cover
void test_mission() {

  for (int i = 0; i < map_points; i++)  {
    Serial.println("mission " + String(i) + " = " + String(mission[i].latitude) + "\t north_mm " + String(mission[i].north_mm));
  }
}

///MOCK_MAP.TXTT
void test_path() {

  for (int i = 0; i < 4; i++) {
    Serial.println("Path " + String(path[i].east_mm));
  }
}
void test_distance()  {
  int percent ;
  int destination;
  int dist = distance(0, &destination , 0 , 0, &percent);

  Serial.println("dist " + String(dist));
}
void setup_C4() {
  Serial.println("C4 setup");
  Serial.println("Start C4 setup");
  DataAvailable = true; //false

  initialize_C4();

  //why setting this to the the first junction of the map??
  Start.east_mm = Nodes[0].east_mm;//Path[last].east_mm;
  Start.north_mm = Nodes[0].north_mm;

  Serial.println("Start planning path");
  //make this a global variable
  last_index_of_path = PlanPath (&Start, &mission[0]);
  Serial.println("Finish C4 setup");

}

/*---------------------------------------------------------------------------------------*/
/**
   All the Methods for C3 starts here
*/
/*---------------------------------------------------------------------------------------*/
/**This method computes the turning radius for the trike
  by taking in the speed

  param : speed_mmPs
  return : turning radius
*/
long turning_radius_mm(long speed_mmPs) {
  return 2 * MIN_TURNING_RADIUS;
}

/**
   The method finds the trike position on the path and compute
   the distance from the intersection you are currently approaching

   param : n = next -> the index of the intersection you're approaching
   return : distance
*/
long findDistance(int n) {
  long north_distance = ((estimated_position.north_mm - path[n].north_mm) * (estimated_position.north_mm - path[n].north_mm));
  long east_distance = ((estimated_position.east_mm - path[n].east_mm) * (estimated_position.east_mm - path[n].east_mm));
  long distance = sqrt(north_distance + east_distance);

  return distance;
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

  //setting the speed to default
  speed_mmPs = default_speed_mmPs;

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
        }
        else if (test_approach_intersection(turn_radius_mm, n)) {
          //changing to a slowwer speed
          speed_mmPs = slow_speed_mmPs;
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
  Serial.println("Start up C3 setup");
  //Trike state starts Straight
  state = STRAIGHT;
  //the path is set to approach the first intersection at index 1
  next = 1;

  C3Results.clear();
  Serial2.end();
  Serial2.begin(baudrate);
  ps3.dt = &C3Results;
  ps3.input = &Serial2; //connection to read from
  ps3.output = &Serial2; //connection to write to
  ps3.capture = MsgType::drive;

  Serial.println("Finish up C3 setup");
}
void loop_C3() {

  Serial.println("Start of C3 loop");

  //Getting the turning radius
  long turn_radius_mm = turning_radius_mm(speed_mmPs);

  //Determining the state of the Trike
  find_state(turn_radius_mm, next);

  //send this data to C2 and design a 3 LCD interface for C2
  //Determining the turn direction for the trike "left, right or straight"
  long turn_direction_angle = get_turn_direction_angle(next);

  //If trike has reached goal
  //  Set desured speed to 0

  //Send speed and angle to C2
  C3Results.clear();
  C3Results.kind = MsgType::drive;
  C3Results.speed_cmPs = DESIRED_SPEED_mmPs;
  C3Results.angle_mDeg = turn_direction_angle;
  C3Results.write(&Serial2);

  Serial.println("End of C3 loop");

}
void setup() {
  setup_C6();
  setup_C4();
  setup_C3();

}
void loop() {
  Serial.println("In loop");
  //start our timer
  long time_start = millis();
  loop_C6();
  //if(too far off path)
  //recompute c4
  loop_C3();
  long time_finish_loop = millis();

  //find out our time
  long time_elapsed = abs(time_finish_loop - time_start);
  Serial.print("total loop time: ");
  Serial.print(time_elapsed);
  while (time_elapsed < 100) {
    //ong time_to_delay = 100 - time_elapsed - 10;
    //wait till the time that we set reach the end
    //delay(time_to_delay);
    time_finish_loop = millis();
    time_elapsed = abs(time_finish_loop - time_start);
  }
  Serial.print("total loop time: ");
  Serial.println(time_elapsed);
}





