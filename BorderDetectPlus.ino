#include <ZumoBuzzer.h>
#include <ZumoMotors.h>
#include <Pushbutton.h>
#include <QTRSensors.h>
#include <ZumoReflectanceSensorArray.h>
#include <avr/pgmspace.h>
#include <Wire.h>
#include <LSM303.h>
#include <RunningAverage.h> // from playground.arduino.cc/Main/RunningAverage
 
#define LED 13

// #define LOG_SERIAL // write log output to serial port
 
// this might need to be tuned for different lighting conditions, surfaces, etc.
#define QTR_THRESHOLD  1500 // microseconds
  
// these might need to be tuned for different motor types
#define REVERSE_SPEED     200 // 0 is stopped, 400 is full speed
#define TURN_SPEED        200
#define SEARCH_SPEED      200
#define SUSTAINED_SPEED   400 // switches to SUSTAINED_SPEEND from FULL_SPEED after FULL_SPEED_DURATION_LIMIT ms
#define FULL_SPEED        400
#define STOP_DURATION     100 // ms
#define REVERSE_DURATION  200 // ms
#define TURN_DURATION     300 // ms

#define RIGHT 1
#define LEFT -1

#define FULL_SPEED_DURATION_LIMIT     250    // ms
#define MIN_DELAY_AFTER_TURN          400    // ms
#define MIN_DELAY_BETWEEN_CONTACTS   1000    // ms = min delay between contact events

#define XY_ACCELERATION_THRESHOLD 150  // for detection of contact (1000 = magnitude of acceleration due to gravity)
boolean in_contact;
unsigned long contact_made_time;
unsigned long last_turn_time;
unsigned long loop_start_time;

ZumoBuzzer buzzer;
const char charge[] PROGMEM = "O4 T100 V15 L4 MS g12>c12>e12>G6>E12 ML>G2";  // use V0 to suppress "charge" sound effect; v15 for max volume

ZumoMotors motors;
enum ForwardSpeed { SearchSpeed, SustainedSpeed, FullSpeed };
ForwardSpeed _forwardSpeed;
unsigned long full_speed_start_time;

Pushbutton button(ZUMO_BUTTON); // pushbutton on pin 12
 
#define NUM_SENSORS 6
unsigned int sensor_values[NUM_SENSORS];

ZumoReflectanceSensorArray sensors(QTR_NO_EMITTER_PIN);

#define RA_SIZE 3  // number of readings to include in running average of accelerometer readings

class Accelerometer : public LSM303
{
  typedef struct acc_data_xy
  {
    unsigned long timestamp;
    float x;
    float y;
    float len;
    float dir;
  } acc_data_xy;
  
  public:
  
    Accelerometer() : ra_x(RA_SIZE), ra_y(RA_SIZE), ra_len_xy(RA_SIZE) {}
    
    // enable accelerometer only
    // to enables both accelerometer and magnetometer, call enableDefault() instead
    void enable(void)
    {
      // Enable Accelerometer
      // 0x27 = 0b00100111
      // Normal power mode, all axes enabled
      writeAccReg(LSM303_CTRL_REG1_A, 0x27);
  
      if (getDeviceType() == LSM303DLHC_DEVICE)
      writeAccReg(LSM303_CTRL_REG4_A, 0x08); // DLHC: enable high resolution mode
    }
    
    void getLogHeader(void)
    {
      Serial.print("millis    x      y     len     dir  | len_avg  dir_avg  |  avg_len");
      Serial.println();
    }
    
    void readAcceleration(unsigned long timestamp)
    {
      readAcc();
      if (a.x == last.x && a.y == last.y) return;
      
      last.timestamp = timestamp;
      last.x = a.x;
      last.y = a.y;
      last.len = sqrt(a.x*a.x + a.y*a.y);
      // last.dir = atan2(a.x, a.y) * 180.0 / M_PI;
      
      ra_x.addValue(last.x);
      ra_y.addValue(last.y);
      // ra_len_xy.addValue(last.len); 
 
#ifdef LOG_SERIAL
     Serial.print(last.timestamp);
     Serial.print("  ");
     Serial.print(last.x);
     Serial.print("  ");
     Serial.print(last.y);
     Serial.print("  ");
     Serial.print(last.len);
     Serial.print("  ");
     Serial.print(last.dir);
     Serial.print("  |  ");
     Serial.print(len_xy_avg());
     Serial.print("  ");
     Serial.print(dir_xy_avg());
     Serial.print("  |  ");
     Serial.print(avg_len_xy());
     Serial.println();
#endif
    }
    
    float x_avg(void) const
    {
      // getAverage should have been declared const in RunningAverage class
      return const_cast<RunningAverage&>(ra_x).getAverage();
    }
    
    float y_avg(void) const
    {
      // getAverage should have been declared const in RunningAverage class
      return const_cast<RunningAverage&>(ra_y).getAverage();
    }
    
    float len_xy_avg(void) const
    {
      return sqrt(x_avg()*x_avg() + y_avg()*y_avg());
    }
    
    float dir_xy_avg(void) const
    {
      return atan2(x_avg(), y_avg()) * 180.0 / M_PI;
    }
    
    float avg_len_xy(void) const
    {
      // getAverage should have been declared const in RunningAverage class
      return const_cast<RunningAverage&>(ra_len_xy).getAverage();
    }
    
  private:
    acc_data_xy last;
    RunningAverage ra_x;
    RunningAverage ra_y;
    RunningAverage ra_len_xy;  // running average xy vector magnitues
};
Accelerometer lsm303;

void waitForButtonAndCountDown(bool restarting);
void setForwardSpeed(ForwardSpeed speed);

void setup()
{  
  // Initiate the Wire library and join the I2C bus as a master
  Wire.begin();
  
  // Initiate LSM303
  lsm303.init();
  lsm303.enable();
  
#ifdef LOG_SERIAL
  Serial.begin(9600);
  lsm303.getLogHeader();
#endif

  randomSeed((unsigned int) millis());
  
  // uncomment if necessary to correct motor directions
  //motors.flipLeftMotor(true);
  //motors.flipRightMotor(true);

  pinMode(LED, HIGH);
  buzzer.playMode(PLAY_AUTOMATIC);
  waitForButtonAndCountDown(false);
}

void waitForButtonAndCountDown(bool restarting)
{ 
#ifdef LOG_SERIAL
  Serial.print(restarting ? "Restarting Countdown" : "Starting Countdown");
  Serial.println();
#endif
  
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
  
  // reset loop variables
  in_contact = false;  // 1 if contact made; 0 if no contact or contact lost
  contact_made_time = 0;
  last_turn_time = millis();  // prevents false contact detection on initial acceleration
  _forwardSpeed = SearchSpeed;
  full_speed_start_time = 0;
}

void loop()
{
  if (button.isPressed())
  {
    // if button is pressed, stop and wait for another press to go again
    motors.setSpeeds(0, 0);
    button.waitForRelease();
    waitForButtonAndCountDown(true);
  }
  
  loop_start_time = millis();
  lsm303.readAcceleration(loop_start_time); 
  sensors.read(sensor_values);
  
  if ((_forwardSpeed == FullSpeed) && (loop_start_time - full_speed_start_time > FULL_SPEED_DURATION_LIMIT))
  { 
    setForwardSpeed(SustainedSpeed);
  }
  
  if (sensor_values[0] < QTR_THRESHOLD)
  {
    // if leftmost sensor detects line, reverse and turn to the right
    turn(RIGHT, true);
  }
  else if (sensor_values[5] < QTR_THRESHOLD)
  {
    // if rightmost sensor detects line, reverse and turn to the left
    turn(LEFT, true);
  }
  else  // otherwise, go straight
  {
    if (check_for_contact()) on_contact_made();
    int speed = getForwardSpeed();
    motors.setSpeeds(speed, speed);
  }
}

// execute turn 
// direction:  RIGHT or LEFT
// randomize: to improve searching
void turn(char direction, bool randomize)
{
#ifdef LOG_SERIAL
  Serial.print("turning ...");
  Serial.println();
#endif

  // assume contact lost
  on_contact_lost();
  
  static unsigned int duration_increment = TURN_DURATION / 4;
  
  // motors.setSpeeds(0,0);
  // delay(STOP_DURATION);
  motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
  delay(REVERSE_DURATION);
  motors.setSpeeds(TURN_SPEED * direction, -TURN_SPEED * direction);
  delay(randomize ? TURN_DURATION + (random(8) - 2) * duration_increment : TURN_DURATION);
  int speed = getForwardSpeed();
  motors.setSpeeds(speed, speed);
  last_turn_time = millis();
}

void setForwardSpeed(ForwardSpeed speed)
{
  _forwardSpeed = speed;
  if (speed == FullSpeed) full_speed_start_time = loop_start_time;
}

int getForwardSpeed()
{
  int speed;
  switch (_forwardSpeed)
  {
    case FullSpeed:
      speed = FULL_SPEED;
      break;
    case SustainedSpeed:
      speed = SUSTAINED_SPEED;
      break;
    default:
      speed = SEARCH_SPEED;
      break;
  }
  return speed;
}
  
// check for contact, but ignore readings immediately after turning or losing contact
bool check_for_contact()
{
  return (lsm303.len_xy_avg() >  XY_ACCELERATION_THRESHOLD) && \
    (loop_start_time - last_turn_time > MIN_DELAY_AFTER_TURN) && \
    (loop_start_time - contact_made_time > MIN_DELAY_BETWEEN_CONTACTS);
}

// sound horn and accelerate on contact -- fight or flight
void on_contact_made()
{
#ifdef LOG_SERIAL
  Serial.print("contact made");
  Serial.println();
#endif
  in_contact = true;
  contact_made_time = loop_start_time;
  setForwardSpeed(FullSpeed);
  buzzer.playFromProgramSpace(charge);
}

// reset forward speed
void on_contact_lost()
{
#ifdef LOG_SERIAL
  Serial.print("contact lost");
  Serial.println();
#endif
  in_contact = false;
  setForwardSpeed(SearchSpeed);
}

