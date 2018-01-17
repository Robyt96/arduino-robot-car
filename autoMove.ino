#define PIN_SERVO 3
#define PIN_ENGINE_R_SPEED 5
#define PIN_ENGINE_L_SPEED 11
#define PIN_R_FORWARD 6
#define PIN_R_BACK 7
#define PIN_L_BACK 8
#define PIN_L_FORWARD 9
#define PIN_US_TRIG A5
#define PIN_US_ECHO A4
//#define DEBUG

#include <Servo.h>
Servo myservo;

#include <NewPing.h>
#define MAX_DISTANCE 200
NewPing sonar(PIN_US_TRIG, PIN_US_ECHO);

int WHEEL_SPEED         = 128;
int RIGHT_WHEELS_OFFSET = 20;
int SERVO_ANGLE         = 30;
int SERVO_OFFSET        = -5;
int STOP_DISTANCE       = 25;
int SAME_DISTANCE_CM    = 10;

int DELAY_SERVO         = 400;
int DELAY_STEP_FORWARD  = 400;
int DELAY_STEP_ROTATE   = 400;
int DELAY_STEP_REVERSE  = 1350;
int DELAY_STEP_STOP     = 100;
int DELAY_STEP_BRAKE    = 10;
int DELAY_LOOP          = 50;

int MAX_REVERSE_CYCLES  = 4;
int MAX_ROTATE_CYCLES   = 6;
int MAX_BLOCKED_CYCLES  = 1;

boolean servoLeftToRight = true;
int distanceLeft, distanceFront, distanceRight;
int oldDistanceLeft, oldDistanceFront, oldDistanceRight;
int rotateCounter = 0, reverseCounter = 0, sameDistanceCounter = 0;
boolean rotateLoop = false, reverseLoop = false, isBlocked = false;
boolean firstCycle = true;

void setup() {
  pinMode(PIN_US_ECHO, INPUT);    
  pinMode(PIN_US_TRIG, OUTPUT);
  pinMode(PIN_ENGINE_R_SPEED, OUTPUT);
  pinMode(PIN_ENGINE_L_SPEED, OUTPUT);
  pinMode(PIN_R_FORWARD, OUTPUT);
  pinMode(PIN_R_BACK, OUTPUT);
  pinMode(PIN_L_BACK, OUTPUT);
  pinMode(PIN_L_FORWARD, OUTPUT);

  myservo.attach(PIN_SERVO);
  #ifdef DEBUG
  Serial.begin(9600);
  #endif
  move_servo(SERVO_ANGLE);
}

void loop() {
  if (!isBlocked) {
    measure_distances();
    
    printInt(distanceLeft);
    printStr(",");
    printInt(distanceFront);
    printStr(",");
    printLnInt(distanceRight);

    moving_algorithm();
    watch_dog();
  
    oldDistanceLeft  = distanceLeft;
    oldDistanceFront = distanceFront;
    oldDistanceRight = distanceRight;
  }

  delay(DELAY_LOOP);
  firstCycle = false;
}

void measure_distances() {
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

  servoLeftToRight = !servoLeftToRight;
}

int move_servo(int angle) {
    myservo.write(90 + SERVO_OFFSET + angle);
    delay(DELAY_SERVO);  
}

int distance_test() {
  int distance1 = _measure();
  delay(20);
  int distance2 = _measure();
  delay(20);
  int distance3 = _measure();
  int distance = (distance1 + distance2 + distance3) / 3;
  
  if (distance <= 0) {
    return MAX_DISTANCE;
  }
  else {
    return distance;
  }
}

int _measure() {
  return sonar.ping_cm();
}

void moving_algorithm() {
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
    //bug: all'accensione gira a destra anche se davanti e' libero. dopo il reset non accade
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
  analogWrite(PIN_ENGINE_R_SPEED, WHEEL_SPEED + RIGHT_WHEELS_OFFSET);
  analogWrite(PIN_ENGINE_L_SPEED, WHEEL_SPEED);

  delay(DELAY_STEP_FORWARD);
  moving_brake();
  //moving_stop();
}

void moving_rotate(boolean toLeft) {
  //printLnStr("rotate");
  reverseCounter = 0;
  rotateCounter = rotateCounter + 1;

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

  delay(DELAY_STEP_ROTATE);
  moving_stop();
}

void moving_reverse() {
  //printLnStr("reverse");
  reverseCounter = reverseCounter + 1;
  rotateCounter = 0;

  digitalWrite(PIN_R_FORWARD, HIGH);
  digitalWrite(PIN_R_BACK, LOW);
  digitalWrite(PIN_L_BACK, HIGH);
  digitalWrite(PIN_L_FORWARD, LOW);
  analogWrite(PIN_ENGINE_R_SPEED, WHEEL_SPEED + RIGHT_WHEELS_OFFSET);
  analogWrite(PIN_ENGINE_L_SPEED, WHEEL_SPEED);
 
  delay(DELAY_STEP_REVERSE);
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
  if (reverseCounter > MAX_REVERSE_CYCLES) {
    //printLnStr("reverseLoop");
    reverseLoop = true;
  }
  else if (rotateCounter > MAX_ROTATE_CYCLES) {
    //printLnStr("rotateLoop");
    rotateLoop = true;
  }

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
