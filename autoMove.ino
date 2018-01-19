/*
*  Sketch : autoMove.ino
*  Authors: Roberto Tagliabue, Donato Tagliabue
*  Date   : Jan 2018
*
*  Move an Arduino (Elegoo) car avoiding obstacles
*/
#define PIN_SERVO 3
#define PIN_ENGINE_R_SPEED 5
#define PIN_ENGINE_L_SPEED 11
#define PIN_R_FORWARD 6
#define PIN_R_BACK 7
#define PIN_L_BACK 8
#define PIN_L_FORWARD 9
#define PIN_US_TRIG A5
#define PIN_US_ECHO A4
// uncomment the following #define to print some debug info through the serial interface
//#define DEBUG

// the servo motor is used to "look" (with ultrasounds) at the left and right
// before moving forward or rotating
#include <Servo.h>
Servo myservo;

// NewPing library is used to measure distances with ultrasounds. More or less the same
// data (and random errors) can also be obtained without using this library
#include <NewPing.h>
#define MAX_DISTANCE 200
NewPing sonar(PIN_US_TRIG, PIN_US_ECHO);

int WHEEL_SPEED         = 128;  // motor speed
int RIGHT_WHEELS_OFFSET = 20;   // extra speed for right wheels (needed for my car)
int SERVO_ANGLE         = 30;   // max rotation angle to measure distances
int SERVO_OFFSET        = -5;   // in my car, the servo is not front aligned with a 90° angle..
int STOP_DISTANCE       = 25;   // threshold, in cm, to stop motors and rotate
int SAME_DISTANCE_CM    = 10;   // to compare consecutive measures (to check if the car is really moving)

int DELAY_SERVO         = 400;  // in ms, move servo motor and wait before measuring distances
int DELAY_STEP_FORWARD  = 400;  // move forward for about 25cm
int DELAY_STEP_ROTATE   = 400;  // turn left or right of about 30°
int DELAY_STEP_REVERSE  = 1350; // delay to reverse the moving direction of 180° (for my car)
int DELAY_STEP_STOP     = 100;  // little delay to have the car stopped
int DELAY_STEP_BRAKE    = 10;   // to stop faster the car, after a forward moving
int DELAY_LOOP          = 50;   // just to add a little delay at every cycle

int MAX_REVERSE_CYCLES  = 4;
int MAX_ROTATE_CYCLES   = 6;
int MAX_BLOCKED_CYCLES  = 1;

// at every loop cycle, three distances are measured: left, front, right (left and
// right at SERVO_ANGLE degrees). To save time (reducing servo motor movements),
// during a cycle they are measured from left to right and the next cycle from right to left
boolean servoLeftToRight = true;

// the measured distances during the current cycle
int distanceLeft, distanceFront, distanceRight;

// the distances measured the cycle before, to be compared to the current ones
// (if for three cycles the values are the same, the car is supposed to be "blocked")
int oldDistanceLeft, oldDistanceFront, oldDistanceRight;

// counters used by the "watchdog" function, to cope with possible loops and "blocks"
int rotateCounter = 0, reverseCounter = 0, sameDistanceCounter = 0;
boolean rotateLoop = false, reverseLoop = false, isBlocked = false;

// for an unknown reason, the car just switched on turns right on the first cycle..
boolean firstCycle = true;

void setup() {
  // initialize the pins for ultrasound sensors and wheel motors
  pinMode(PIN_US_ECHO, INPUT);    
  pinMode(PIN_US_TRIG, OUTPUT);
  pinMode(PIN_ENGINE_R_SPEED, OUTPUT);
  pinMode(PIN_ENGINE_L_SPEED, OUTPUT);
  pinMode(PIN_R_FORWARD, OUTPUT);
  pinMode(PIN_R_BACK, OUTPUT);
  pinMode(PIN_L_BACK, OUTPUT);
  pinMode(PIN_L_FORWARD, OUTPUT);

  // move the ultrasound sensors on the left, ready for the first measure
  myservo.attach(PIN_SERVO);
  #ifdef DEBUG
  Serial.begin(9600);
  #endif
  move_servo(SERVO_ANGLE);
}

void loop() {
  // "blocked" here means more than MAX_REVERSE_CYCLES (4) consecutive 180-degree
  // rotations. In this case, stop forever
  if (!isBlocked) {
    measure_distances(); // find obstacles
    moving_algorithm();  // move the car
    watch_dog();         // find abnormal situations
  
    // save distance measures for the next cycle (for the watch_dog function)
    oldDistanceLeft  = distanceLeft;
    oldDistanceFront = distanceFront;
    oldDistanceRight = distanceRight;
  }

  delay(DELAY_LOOP);
  firstCycle = false;
}

void measure_distances() {
  // measure the three distances (left, front, right) from left to right
  // or vice-versa, depending on where the servo motor is
  if (servoLeftToRight) {
    distanceLeft = distance_test();
    move_servo(0);
    distanceFront = distance_test();
    move_servo(-SERVO_ANGLE);
    distanceRight = distance_test();
  }
  else {
    distanceRight = distance_test();
    move_servo(0);
    distanceFront = distance_test();
    move_servo(SERVO_ANGLE);
    distanceLeft = distance_test();
  }
  
  printInt(distanceLeft);
  printStr(",");
  printInt(distanceFront);
  printStr(",");
  printLnInt(distanceRight);
  
  // change the order of the measures for the next cycle
  servoLeftToRight = !servoLeftToRight;
}

int move_servo(int angle) {
  // move the servo (with the ultrasound sensors) and wait
  // the small SERVO_OFFSET is to cope with the servo of my car, a bit unaligned
  myservo.write(90 + SERVO_OFFSET + angle);
  delay(DELAY_SERVO);  
}

int distance_test() {
  // take the average of three measures, to avoid random errors that sometimes
  // happen. Even with this average, not always you'll get good values...
  int distance1 = _measure();
  delay(20);
  int distance2 = _measure();
  delay(20);
  int distance3 = _measure();
  int distance = (distance1 + distance2 + distance3) / 3;
  
  if (distance <= 0) {
    // this should never happen if the servo is not initialized with a max
    // distance. Just to be on the safe side
    return MAX_DISTANCE;
  }
  else {
    return distance;
  }
}

int _measure() {
  // get the distance (in cm) between the sensors and an obstacle
  return sonar.ping_cm();
}

void moving_algorithm() {
  // ok, this is the moving algorithm, after getting the three distances:
  // - everywhere free in front of the car -> go ahead a little and stop
  // - obstacle in front-left -> turn right (and vice-versa)
  // - obstacle just in front -> turn towards the freer direction (i.e. greater distance)
  // - obstacle everywhere (left, front, right) -> reverse
  
  if (distanceFront > STOP_DISTANCE && distanceLeft > STOP_DISTANCE && distanceRight > STOP_DISTANCE) {
    moving_forward();
  }
  else if (distanceFront <= STOP_DISTANCE) {
    if (distanceLeft <= STOP_DISTANCE && distanceRight > STOP_DISTANCE) {
      moving_rotate(false); //turn right
    }
    else if (distanceLeft > STOP_DISTANCE && distanceRight <= STOP_DISTANCE) {
      moving_rotate(true); //turn left
    }
    else if (distanceLeft > STOP_DISTANCE && distanceRight > STOP_DISTANCE) {
      moving_rotate(distanceLeft > distanceRight);
    }
    else {
      moving_reverse();
    }
  }
  else if (distanceLeft <= STOP_DISTANCE) {
    // when you switch on the car, it would turn always right at first (even without obstacles)
    if (!firstCycle) {
      moving_rotate(false); //turn right
    }
  }
  else if (distanceRight <= STOP_DISTANCE) {
    moving_rotate(true); //turn left
  }  
}

void moving_forward() {
  //printLnStr("forward");
  reverseCounter = 0;
  rotateCounter = 0;

  digitalWrite(PIN_R_FORWARD, HIGH);
  digitalWrite(PIN_R_BACK, LOW);
  digitalWrite(PIN_L_BACK, LOW);
  digitalWrite(PIN_L_FORWARD, HIGH);
  // use the analogWrite to move the car slowly. 128 should be a good value. My car
  // tends to deviate to right, so add some RIGHT_WHEELS_OFFSET to the right motor
  analogWrite(PIN_ENGINE_R_SPEED, WHEEL_SPEED + RIGHT_WHEELS_OFFSET);
  analogWrite(PIN_ENGINE_L_SPEED, WHEEL_SPEED);

  delay(DELAY_STEP_FORWARD); // this should be enough for 20-30 cm
  moving_brake(); // to stop faster the car, a small back motion ("brake")
}

void moving_rotate(boolean toLeft) {
  //printLnStr("rotate");
  reverseCounter = 0;
  rotateCounter = rotateCounter + 1; // to catch too many consecutive rotations

  if (toLeft) {
    digitalWrite(PIN_R_FORWARD, HIGH);
    digitalWrite(PIN_R_BACK, LOW);
    digitalWrite(PIN_L_BACK, HIGH);
    digitalWrite(PIN_L_FORWARD, LOW);
  }
  else {
    digitalWrite(PIN_R_FORWARD, LOW);
    digitalWrite(PIN_R_BACK, HIGH);
    digitalWrite(PIN_L_BACK, LOW);
    digitalWrite(PIN_L_FORWARD, HIGH);
  }
 
  analogWrite(PIN_ENGINE_R_SPEED, WHEEL_SPEED + RIGHT_WHEELS_OFFSET);
  analogWrite(PIN_ENGINE_L_SPEED, WHEEL_SPEED);

  delay(DELAY_STEP_ROTATE); // this should give a 30 degree rotation of the car
  moving_stop();
}

void moving_reverse() {
  //printLnStr("reverse");
  reverseCounter = reverseCounter + 1; // to catch too many consecutive reversals
  rotateCounter = 0;

  digitalWrite(PIN_R_FORWARD, HIGH);
  digitalWrite(PIN_R_BACK, LOW);
  digitalWrite(PIN_L_BACK, HIGH);
  digitalWrite(PIN_L_FORWARD, LOW);
  analogWrite(PIN_ENGINE_R_SPEED, WHEEL_SPEED + RIGHT_WHEELS_OFFSET);
  analogWrite(PIN_ENGINE_L_SPEED, WHEEL_SPEED);
 
  delay(DELAY_STEP_REVERSE); // adjust it, in order to have a 180 degree rotation
  moving_stop();  
}

void moving_back() {
  //printLnStr("back");
  reverseCounter = 0;
  rotateCounter = 0;
  digitalWrite(PIN_R_FORWARD, LOW);
  digitalWrite(PIN_R_BACK, HIGH);
  digitalWrite(PIN_L_BACK, HIGH);
  digitalWrite(PIN_L_FORWARD, LOW);
  analogWrite(PIN_ENGINE_R_SPEED, WHEEL_SPEED + RIGHT_WHEELS_OFFSET);
  analogWrite(PIN_ENGINE_L_SPEED, WHEEL_SPEED);

  delay(DELAY_STEP_FORWARD);
  moving_stop();
}

void moving_stop() {
  //printLnStr("stop");
  digitalWrite(PIN_R_FORWARD, LOW);      
  digitalWrite(PIN_R_BACK, LOW);
  digitalWrite(PIN_L_BACK, LOW);      
  digitalWrite(PIN_L_FORWARD, LOW);
  digitalWrite(PIN_ENGINE_R_SPEED, LOW);
  digitalWrite(PIN_ENGINE_L_SPEED, LOW);
  delay(DELAY_STEP_STOP);
} 

void moving_brake() {
  //printLnStr("brake");
  digitalWrite(PIN_R_FORWARD, LOW);      
  digitalWrite(PIN_R_BACK, HIGH);
  digitalWrite(PIN_L_BACK, HIGH);      
  digitalWrite(PIN_L_FORWARD, LOW);
  delay(DELAY_STEP_BRAKE);
  moving_stop();
}

void watch_dog() {
  // this function is added to the "normal" algorithm in order to try to solve
  // these potential problems:
  // - the car continues to reverse its motion (for istance, in a box)
  // - the car cannot decide whether to turn left or right
  // - the car is blocked by an unseen obstacle
  
  if (reverseCounter > MAX_REVERSE_CYCLES) {
    //printLnStr("reverseLoop");
    reverseLoop = true; // in this case, stop forever
  }
  else if (rotateCounter > MAX_ROTATE_CYCLES) {
    //printLnStr("rotateLoop");
    rotateLoop = true; // in this case, reverse the motion
  }

  // this is a kind of bet: if all the three measures (left, front, right) are
  // more or less the same for some consecutive cycles, the car is not really
  // moving (even if the main algorithm "thinks" so). It's the only way to handle
  // this (using only the ultrasound sensors)
  int diffLeft  = distanceLeft - oldDistanceLeft;
  int diffFront = distanceFront - oldDistanceFront;
  int diffRight = distanceRight - oldDistanceRight;

  if ((abs(diffLeft) < SAME_DISTANCE_CM)
    && (abs(diffFront) < SAME_DISTANCE_CM)
    && (abs(diffRight) < SAME_DISTANCE_CM)) {
    sameDistanceCounter = sameDistanceCounter + 1;
  }
  else {
    sameDistanceCounter = 0;
  }

  if (sameDistanceCounter > MAX_BLOCKED_CYCLES) {
    // if the car is blocked by an unseen obstacle, go back and rotate
    sameDistanceCounter = 0;
    moving_back();
    delay(DELAY_LOOP);  
    moving_rotate(distanceLeft > distanceRight);    
    moving_rotate(distanceLeft > distanceRight);    
    delay(DELAY_LOOP);  
  }
  else if (reverseLoop) {
    isBlocked = true;
  }
  else if (rotateLoop) {
    rotateLoop = false;
    rotateCounter = 0;
    moving_reverse();
    delay(DELAY_LOOP);  
  }
}

void printStr(String msg) {
  #ifdef DEBUG
  Serial.print(msg);
  #endif  
}

void printLnStr(String msg) {
  #ifdef DEBUG
  Serial.println(msg);
  #endif    
}

void printInt(int i) {
  #ifdef DEBUG
  Serial.print(i);
  #endif  
}

void printLnInt(int i) {
  #ifdef DEBUG
  Serial.println(i);
  #endif    
}
