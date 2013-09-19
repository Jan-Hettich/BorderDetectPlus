#include <ZumoBuzzer.h>
#include <ZumoMotors.h>
#include <Pushbutton.h>
#include <QTRSensors.h>
#include <ZumoReflectanceSensorArray.h>
#include <avr/pgmspace.h>
 
#define LED 13
 
// this might need to be tuned for different lighting conditions, surfaces, etc.
#define QTR_THRESHOLD  1500 // microseconds
  
// these might need to be tuned for different motor types
#define REVERSE_SPEED     200 // 0 is stopped, 400 is full speed
#define TURN_SPEED        200
#define FORWARD_SPEED     200
#define ATTACK_SPEED      400
#define REVERSE_DURATION  200 // ms
#define TURN_DURATION     400 // ms


const char charge[] PROGMEM = "O4 T100 V15 L4 MS g12>c12>e12>G6>E12 ML>G2";

 
ZumoBuzzer buzzer;
ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON); // pushbutton on pin 12
 
#define NUM_SENSORS 6
unsigned int sensor_values[NUM_SENSORS];

#define SEARCH 0
#define ATTACK 1
#define REGROUP 2

unsigned char mode;

#define LOOP_MAX 2500

ZumoReflectanceSensorArray sensors(QTR_NO_EMITTER_PIN);

void waitForButtonAndCountDown()
{
  digitalWrite(LED, HIGH);
  button.waitForButton();
  digitalWrite(LED, LOW);
   
  // play audible countdown
  for (int i = 0; i < 3; i++)
  {
    delay(1000);
    buzzer.playNote(NOTE_G(3), 50, 12);
  }
  delay(1000);
  buzzer.playFromProgramSpace(charge);
  delay(1000);
}
 
void setup()
{
  // uncomment if necessary to correct motor directions
  //motors.flipLeftMotor(true);
  //motors.flipRightMotor(true);

  mode = SEARCH;
  pinMode(LED, HIGH);
  buzzer.playMode(PLAY_AUTOMATIC);
  waitForButtonAndCountDown();
}

void loop()
{
  static unsigned int loop_count = 0; 
  if (button.isPressed())
  {
    // if button is pressed, stop and wait for another press to go again
    motors.setSpeeds(0, 0);
    button.waitForRelease();
    waitForButtonAndCountDown();
  }
  
  if (loop_count % LOOP_MAX == LOOP_MAX - 1) 
  {
    buzzer.playFromProgramSpace(charge);
  }

  switch (mode)
  {
    case SEARCH:
      search();
      break;
    case ATTACK:
      attack();
      break;
    case REGROUP:
      regroup();
      break;
  }
  
  loop_count += 1;  
}

void search() 
{
  
  sensors.read(sensor_values);
  
  if (sensor_values[0] < QTR_THRESHOLD)
  {
    // if leftmost sensor detects line, reverse and turn to the right
    motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
    delay(200);
    motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
    delay(300);
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
  }
  else if (sensor_values[5] < QTR_THRESHOLD)
  {
    // if rightmost sensor detects line, reverse and turn to the left
    motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
    delay(200);
    motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
    delay(300);
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
  }
  else
  {
    // otherwise, go straight
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
  }
}

void attack()
{
}

void regroup()
{
}
