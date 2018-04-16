#include <Common.h>
//dont need 4/6/18
//#include <Matrix.h>
#include <ElcanoSerial.h>
//dont need 4/6/18
//#include <SPI.h>
// Pooja - The baudrate constant was moved from the ElcanoSerial.h to
// the Elcano_C3 Pilot.ino
const int32_t baudrate = 74800;
using namespace elcano;
const long default_speed_mmPs = 1000;
const long slow_speed_mmPs = 100;
const long MIN_TURNING_RADIUS = 2400;
int next = 1; //index into path 

long speed_mmPs = default_speed_mmPs;

enum States { STOP, STRAIGHT, ENTER_TURN, LEAVE_TURN, APPROACH_GOAL, LEAVE_GOAL};
States state;


void setup() {

  Serial.begin(9600);
  Serial.flush();
  Serial1.begin(baudrate);
  next = 1;
  state = STRAIGHT;
}

void loop() {
  
  long turning_radius_mm = turning_radius_mm(speed_mmPs);

  // Find trike position on path
  //Find distance from trike to next intersection point
  long distanceFound_mm = findDistance(next);

  //If distance less than turning radius,compute the steering angle
  find_state(turning_radious_mm, next);

 

  //send to C2 
  long turn_direction = get_turn_direction(turning_radius_mm, next);
  /*
   * do 3 lcd for low level system
   */
  //send speed to C2
  
  
  /* future expansion
    Get obstacles from sonar;
    Get obstacles from Scanse sweep;
    If necessary, alter desired speed and steer angle;
  */

  //If trike has reached goal
  //  Set desured speed to 0
  //Send speed and angle to C2


}

long turning_radius_mm(long speed_mmPs) {
  return 2 * MIN_TURNING_RADIUS;
}

long findDistance(int n) {
  long north_distance = ((estimated_position.north_mm - path[n].north_mm) * (estimated_position.north_mm - path[n].north_mm));
  long east_distance = ((estimated_position.east_mm - path[n].east_mm) * (estimated_position.east_mm - path[n].east_mm));
  long distance = sqrt(north_distance + east_distance);

  return distance;
}

//set state 
void find_state(long turning_radius_mm, int n) {



  switch (state) {
    case STRAIGHT:
      if(test_approach_intersection(n, turning_radius_mm)){
       //Needs to find the last index of path 
       // if(n is a goal)
       //   state = APPROACH_GOAL;
       //else 
        state = ENTER_TURN;
      }
      break;

    case STOP:
        break;

    case ENTER_TURN:
      //if(past halfway point) state = LEAVE_TURN; next++;
      
      if (test_past_destination(n)) {
        state = LEAVE_TURN;
        next++;
        
      }
      break;

    case LEAVE_TURN:
      if(test_leave_intersection(n, turning_radius_mm) {
         state = STRAIGHT;
      }
      break;
    case APPROACH_GOAL:
      if(test_past_destination(n)) {
        state = STOP;
      }
      else if(test_approach_intersection(n, turning_radius_mm)) {
        //changing to a lower speed 
        speed_mmPs = slow_speed_mmPs;
      }
      break;
    case LEAVE_GOAL:
      break;
  }
}

//Checking to see if we have passed the destination or not
//Also check for type of rectangles skinny, long, square of the path
bool test_past_destination(int n) {
  if (abs(path[n - 1].east_mm - path[n].east_mm) > abs(path[n - 1].north_mm - path[n].north_mm)) {
    if (path[n].east_mm > path[n - 1].east_mm && estimated_position.east_mm > path[n].east_mm) {
      return false;
    }
    else if (path[n - 1].east_mm < path[n].east_mm && estimated_position.east_mm < path[n].east_mm) {
      return false  ;
    }
  }
  else {
    if (path[n].north_mm > path[n - 1].north_mm && estimated_position.north_mm > path[n].north_mm) {
      return false;
    }
    else if (path[n - 1].north_mm < path[n].north_mm && estimated_position.north_mm < path[n].north_mm) {
      return false;
    }
  }
  return true;
}
bool test_approach_intersection(int n, long turning_radius) {
  if (abs(path[n - 1].east_mm - path[n].east_mm) > abs(path[n - 1].north_mm - path[n].north_mm)) {
    if (path[n].east_mm > path[n - 1].east_mm && estimated_position.east_mm > path[n].east_mm + turning_radius) {
      return false;
    }
    else if (path[n - 1].east_mm < path[n].east_mm && estimated_position.east_mm < path[n].east_mm - turning_radius) {
      return false  ;
    }
  }
  else {
    if (path[n].north_mm > path[n - 1].north_mm && estimated_position.north_mm > path[n].north_mm + turning radius) {
      return false;
    }
    else if (path[n - 1].north_mm < path[n].north_mm && estimated_position.north_mm < path[n].north_mm - turning radius) {
      return false;
    }
  }
  return true;
}
bool test_leave_intersection(int n, long turning_radius) {
  if (abs(path[n - 1].east_mm - path[n].east_mm) > abs(path[n - 1].north_mm - path[n].north_mm)) {
    if (path[n].east_mm > path[n - 1].east_mm && estimated_position.east_mm > path[n-1].east_mm + turning_radius) {
      return false;
    }
    else if (path[n - 1].east_mm < path[n].east_mm && estimated_position.east_mm < path[n-1].east_mm - turning_radius) {
      return false  ;
    }
  }
  else {
    if (path[n].north_mm > path[n - 1].north_mm && estimated_position.north_mm > path[n-1].north_mm + turning radius) {
      return false;
    }
    else if (path[n - 1].north_mm < path[n].north_mm && estimated_position.north_mm < path[n-1].north_mm - turning radius) {
      return false;
    }
  }
  return true;
}

//This method determines the direction of turns of the vehicle
//pos = right, neg = left, 0 = straight
long get_turn_direction(long turning_radius_mm, int n) {
  if (state == ENTER_TURN || state == LEAVE_TURN) {
    long turn_direction = (estimated_position_Evector_x1000 * path[n].Evector_x1000) + (estimated_position_Nvector_x1000 * path[n].Nvector_x1000);
    //60000 = 2.5 degrees of 360 turning angle
    if (turn_direction) > 60000)
      return 1;
      else if (turn_direction < -60000)
      return -1;
      else
        return 0;
      }
else {
  //no turn required
  return 0;
}
}
