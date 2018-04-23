#include <ElcanoSerial.h>

using namespace elcano;

/**
   Data normally in <Setting.h>
*/
// Trike-specific physical parameters
#define WHEEL_DIAMETER_MM 482
// Wheel Circumference
#define WHEEL_CIRCUM_MM (long) (WHEEL_DIAMETER_MM * PI)
//Interrupt from the speedometer
#define IRPT_WHEEL 3

//This is the end of the <Setting.h>

long startTime;


/*
   C2 is the low-level controller that sends control signals to the hub motor,
   brake servo, and steering servo.  It is (or will be) a PID controller, but
   may also impose limits on control values for the motor and servos as a safety
   measure to protect against incorrect PID settings.

   It receives desired speed and heading from either of two sources, an RC
   controller operated by a person, or the C3 pilot module.  These are mutually
   exclusive.

   RC commands are received directly as interrupts on a bank of pins.  C3 commands
   are received over a serial line using the Elcano Serial protocol.  Heading and
   speed commands do not need to be passed through to other modules, but because
   the Elcano Serial protocol uses a unidirectional ring structure, C2 may need to
   pass through *other* commands that come from C3 but are intended for modules
   past C2 on the ring.
*/
static struct hist {
  long olderSpeed_mmPs;  // older data
  unsigned long olderTime_ms;   // time stamp of older speed
  float currentSpeed_kmPh;
  long oldSpeed_mmPs;  // last data from the interrupt
  byte oldClickNumber;
  unsigned long oldTime_ms;  // time stamp of old speed
  long tickMillis;
  long oldTickMillis;
  byte nowClickNumber;  // situation when we want to display the speed
  unsigned long nowTime_ms;
  unsigned long TickTime_ms;  // Tick times are used to compute speeds
  unsigned long OldTick_ms;   // Tick times may not match time stamps if we don't process
  // results of every interrupt
} history;

// 10 milliseconds -- atordjust to accomodate the fastest needed response or
// sensor data capture.
#define LOOP_TIME_MS 10

//#define TEN_SECONDS_IN_MICROS 10000000
#define ULONG_MAX 4294967295

static long distance_mm = 0;
volatile boolean synced = false;
volatile bool flipping;

float Odometer_m = 0;
float HubSpeed_kmPh;
//const float  HubSpeed2kmPh = 13000000;
const unsigned long HubAtZero = 1159448;

// Time at which this loop pass should end in order to maintain a
// loop period of LOOP_TIME_MS.
unsigned long nextTime;
// Time at which we reach the end of loop(), which should be before
// nextTime if we have set the loop period long enough.
unsigned long endTime;
// How much time we need to wait to finish out this loop pass.
unsigned long delayTime;

// Inter-module communications data.
SerialData results_of_c3, c2_results;

ParseState parseState, parseState2;

/*========================================================================/
  ============================WheelRev4 code==============================/
  =======================================================================*/

/* Wheel Revolution Interrupt routine
   Ben Spencer 10/21/13
   Modified by Tyler Folsom 3/16/14; 3/3/16
   A cyclometer gives a click once per revolution.
   This routine computes the speed.
*/
#define MEG 1000000
#define MAX_SPEED_KPH 50  //re-defined value from Setting.h
#define MAX_SPEED_mmPs   ((MAX_SPEED_KPH * MEG) / 3600)
// MAX_SPEED_mmPs = 13,888 mm/s = 13.888 m/s
unsigned long MinTickTime_ms;
// MinTickTime_ms = 89 ms
#define MIN_SPEED_mPh 3000
// A speed of less than 0.3 KPH is zero.
unsigned long MaxTickTime_ms;
// MinTickTime_ms = 9239 ms = 9 sec

double SpeedCyclometer_mmPs = 0;
// Speed in revolutions per second is independent of wheel size.
float SpeedCyclometer_revPs = 0.0;//revolutions per sec

#define IRQ_NONE 0
#define IRQ_FIRST 1
#define IRQ_SECOND 2
#define IRQ_RUNNING 3
#define NO_DATA 0x7FFFFFFF
volatile byte InterruptState = IRQ_NONE;  // Tells us if we have initialized.
volatile byte ClickNumber = 0;         // Used to distinguish old data from new.
volatile unsigned long TickTime = 0;  // Time from one wheel rotation to the next gives speed.
volatile unsigned long OldTick = 0;
int oldClickNumber;


long desired_speed = 0;
long actual_turn_angle = 0;



/*-----------------------------------setup-----------------------------------------------*/
void setup()  {
  startTime = millis();

  //Set up pins


  // put vehicle in initial state

  setupWheelRev(); // WheelRev4 addition


  Serial.begin(9600);

  //set up for receiving buffers from C3
  results_of_c3.clear();
  parseState.dt = &results_of_c3;
  parseState.input = &Serial3;
  parseState.output = &Serial3;
  Serial3.begin(baudrate);
  parseState.capture = MsgType::drive;
  // msgType::drive uses `speed_cmPs` and `angle_deg`


  //set up for transmitting data from C2
  c2_results.clear();
  parseState2.dt = &c2_results;
  parseState2.input = &Serial3;
  parseState2.output = &Serial3;
  Serial3.begin(baudrate);
  parseState2.capture = MsgType::drive;




  SpeedCyclometer_mmPs = 0;

  // THIS MUST BE THE LAST LINE IN setup().
  nextTime = millis();
}

/*-----------------------------------loop------------------------------------------------*/
void loop() {

  // Get the next loop start time. Note this (and the millis() counter) will
  // roll over back to zero after they exceed the 32-bit size of unsigned long,
  // which happens after about 1.5 months of operation (should check this).
  // But leave the overflow computation in place, in case we need to go back to
  // using the micros() counter.
  // If the new nextTime value is <= LOOP_TIME_MS, we've rolled over.
  nextTime = nextTime + LOOP_TIME_MS;

  // DO NOT INSERT ANY LOOP CODE ABOVE THIS LINE.

  //read data from C3 using Elcano_Serial
  ParseStateError r = parseState.update();
  if (r == ParseStateError::success) {
    Serial.println("done");
    Serial.println(static_cast<int8_t>(r));
    Serial.println(results_of_c3.speed_cmPs);

    //set desired_speed = results_of_c3.speed_cmPs;
    //set desired_angle = results_of_c3.angle_mDeg;
  }
  //call function for displaying speed and angle

  // send data to C6 of the actual speed
  c2_results.clear();
  c2_results.kind = MsgType::drive;
  c2_results.speed_cmPs = SpeedCyclometer_mmPs / 10;
  c2_results.angle_mDeg = 0;
  c2_results.write(&Serial3);
  computeSpeed(&history);

  // END TEMPORARY

  // @ToDo: What information do we need to send to C6? Is that communication already
  // hiding in here somewhere, or does it need to be added?

  // DO NOT INSERT ANY LOOP CODE BELOW THIS POINT.

  // Figure out how long we need to wait to reach the desired end time
  // for this loop pass. First, get the actual end time. Note: Beyond this
  // point, there should be *no* more controller activity -- we want
  // minimal time between now, when we capture the actual loop end time,
  // and when we pause til the desired loop end time.
  endTime = millis();
  delayTime = 0L;

  // Did the millis() counter or nextTime overflow and roll over during
  // this loop pass? Did the loop's processing time overrun the desired
  // loop period? We have different computations for the delay time in
  // various cases:
  if ((nextTime >= endTime) &&
      (((endTime < LOOP_TIME_MS) && (nextTime < LOOP_TIME_MS)) ||
       ((endTime >= LOOP_TIME_MS) && (nextTime >= LOOP_TIME_MS)))) {
    // 1) Neither millis() nor nextTime rolled over --or-- millis() rolled
    // over *and* nextTime rolled over when we incremented it. For this case,
    // endTime and nextTime will be in their usual relationship, with
    // nextTime >= endTime, and both nextTime and endTime are either greater
    // than the desired loop period, or both are smaller than that.
    // In this case, we want a delayTime of nextTime - endTime here.
    delayTime = nextTime - endTime;
  } else {
    // (We get here if:
    // nextTime < endTime -or- exactly one of nextTime or endTime rolled over.
    // Negate the first if condition and use DeMorgan's laws...
    // Now pick out the "nextTime rolled over" case. We don't need to test both
    // nextTime and endTime as we know only one rolled over.)
    if (nextTime < LOOP_TIME_MS) {
      // 2) nextTime rolled over when we incremented it, but the millis() timer
      // hasn't yet rolled over.
      // In this case, we know we didn't exhaust the loop time, and the time we
      // need to wait is the remaining time til millis() will roll over, i.e.
      // from endTime until the max long value, plus the time from zero to
      // nextTime.
      delayTime = ULONG_MAX - endTime + nextTime;
    } else {
      // (We get here if:
      // nextTime < endTime -or-
      // nextTime >= endTime -and- nextTime did not roll over but endTime did.)
      // What remains are these two cases:
      // 3) nextTime hasn't rolled over, but millis() has.
      // In this case, we overran the loop time. Since millis() has rolled over,
      // we can just use the normal overrun fixup. So combine this with...
      // 4) Neither nextTime nor millis rolled over, but we overran the desired
      // loop period.
      // In this case, we have no delay, but instead extend the allowed time for
      // this loop pass to the actual time it took.
      nextTime = endTime;
      delayTime = 0L;
    }
  }

  // Did we spend long enough in the loop that we should immediately start
  // the next pass?
  if (delayTime > 0L) {
    // No, pause til the next loop start time.
    delay(delayTime);
  }

}
/*------------------------------------processHighLevel------------------------------------*/
// Process information received form high level
// Precondition: SerialData exists
// Postcondition: desiredAngle and desiredSpeed are both set based on high level information
void processHighLevel(SerialData * results) {
  Serial3.end();
  Serial3.begin(baudrate);  // clear the buffer

  //needs to compute the speed and send it to the high level
}

/*========================================================================/
  ============================WheelRev4 code==============================/
  =======================================================================*/

/*----------------------------WheelRev---------------------------------------------------*/
// WheelRev is called by an interrupt.
void WheelRev() {
  oldClickNumber = ClickNumber;
  //static int flip = 0;
  unsigned long tick;
  noInterrupts();
  tick = millis();
  if (InterruptState != IRQ_RUNNING) {
    // Need to process 1st two interrupts before results are meaningful.
    InterruptState++;
  }

  if (tick - TickTime > MinTickTime_ms) {
    OldTick = TickTime;
    TickTime = tick;
    ++ClickNumber;
  }
  interrupts();
}
/*----------------------------setupWheelRev----------------------------------------------*/
void setupWheelRev()  {
  float MinTick = WHEEL_CIRCUM_MM;
  MinTick *= 1000.0;
  MinTick /= MAX_SPEED_mmPs;
  MinTickTime_ms = MinTick;
  float MIN_SPEED_mmPs =  ((MIN_SPEED_mPh * 1000.0) / 3600.0);
  float MaxTick = (WHEEL_DIAMETER_MM * PI * 1000.0) / MIN_SPEED_mmPs;
  MaxTickTime_ms = MaxTick;
  TickTime = millis();
  // OldTick will normally be less than TickTime.
  // When it is greater, TickTime - OldTick is a large positive number,
  // indicating that we have not moved.
  // TickTime would overflow after days of continuous operation, causing a glitch of
  // a display of zero speed.  It is unlikely that we have enough battery power to ever see this.
  OldTick = TickTime;
  InterruptState = IRQ_NONE;
  ClickNumber = 0;

  attachInterrupt (digitalPinToInterrupt(IRPT_WHEEL), WheelRev, RISING);//pin 3 on Mega
}


/*----------------------------computeSpeed-----------------------------------------------*/
void computeSpeed(struct hist *data) {
  //cyclometer has only done 1 or 2 revolutions
  //normal procedures begin here
  unsigned long WheelRev_ms = TickTime - OldTick;
  float SpeedCyclometer_revPs = 0.0;//revolutions per sec
  if (InterruptState == IRQ_NONE || InterruptState == IRQ_FIRST)  { // No data
    SpeedCyclometer_mmPs = 0;
    SpeedCyclometer_revPs = 0;
    return;
  }

  if (InterruptState == IRQ_SECOND) { //  first computed speed
    SpeedCyclometer_revPs = 1000.0 / WheelRev_ms;
    SpeedCyclometer_mmPs  =
      data->oldSpeed_mmPs = data->olderSpeed_mmPs = WHEEL_CIRCUM_MM * SpeedCyclometer_revPs;
    data->oldTime_ms = OldTick;
    data->nowTime_ms = TickTime;  // time stamp for oldSpeed_mmPs
    data->oldClickNumber = data->nowClickNumber = ClickNumber;
    return;
  }

  if (InterruptState == IRQ_RUNNING)  { //  new data for second computed speed
    if (TickTime == data->nowTime_ms) { //no new data
      //check to see if stopped first
      unsigned long timeStamp = millis();
      if (timeStamp - data->nowTime_ms > MaxTickTime_ms)  { // too long without getting a tick
        SpeedCyclometer_mmPs = 0;
        SpeedCyclometer_revPs = 0;
        if (timeStamp - data->nowTime_ms > 2 * MaxTickTime_ms)  {
          InterruptState = IRQ_FIRST;  //  Invalidate old data
          data->oldSpeed_mmPs = NO_DATA;
          data->olderSpeed_mmPs = NO_DATA;
        }
        return;
      }
      if (data->oldSpeed_mmPs > SpeedCyclometer_mmPs) { // decelerrating, extrapolate new speed using a linear model
        float deceleration = (float) (data->oldSpeed_mmPs - SpeedCyclometer_mmPs) / (float) (timeStamp - data->nowTime_ms);
        SpeedCyclometer_mmPs = data->oldSpeed_mmPs - deceleration * (timeStamp - data->nowTime_ms);

        if (SpeedCyclometer_mmPs < 0)
          SpeedCyclometer_mmPs = 0;

        SpeedCyclometer_revPs = SpeedCyclometer_mmPs / WHEEL_CIRCUM_MM;
      }
      else  { // accelerating; should get new data soon

      }
      return;
    }

    //update time block
    data->olderTime_ms = data->oldTime_ms;
    data->oldTime_ms = data->nowTime_ms;
    data->nowTime_ms = TickTime;
    data->oldClickNumber = data->nowClickNumber;
    data->nowClickNumber = ClickNumber;

    //update speed block
    data->olderSpeed_mmPs = data->oldSpeed_mmPs;
    data->oldSpeed_mmPs = SpeedCyclometer_mmPs;
    SpeedCyclometer_revPs = 1000.0 / WheelRev_ms;
    SpeedCyclometer_mmPs  = WHEEL_CIRCUM_MM * SpeedCyclometer_revPs;

    data->oldTickMillis = data->tickMillis;
    data->tickMillis = millis();

    data->currentSpeed_kmPh = SpeedCyclometer_mmPs / 260.0;
    distance_mm += ((data -> oldTime_ms - data -> olderTime_ms) / 1000.0) * (data -> oldSpeed_mmPs);

    if (data->TickTime_ms - data->OldTick_ms > 1000)
      data->currentSpeed_kmPh = 0;

    return;
  }
}




