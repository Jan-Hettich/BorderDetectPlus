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

#define LOG_SERIAL // write log output to serial port
 
// this might need to be tuned for different lighting conditions, surfaces, etc.
#define QTR_THRESHOLD  1500 // microseconds
  
// these might need to be tuned for different motor types
#define REVERSE_SPEED     200 // 0 is stopped, 400 is full speed
#define TURN_SPEED        200
#define FORWARD_SPEED     200
#define FULL_SPEED        400
#define STOP_DURATION     100 // ms
#define REVERSE_DURATION  200 // ms
#define TURN_DURATION     300 // ms

#define IGNORE_ACCERATION_AFTER_TURN  4  // # of loops to ignore acceleration reading after turning
unsigned int loops_after_turning;
#define MIN_DELAY_BETWEEN_CONTACTS 1500  // min delay between lost_contact and made_contact events
unsigned long contact_lost_time;

ZumoBuzzer buzzer;
const char charge[] PROGMEM = "O4 T100 V0 L4 MS g12>c12>e12>G6>E12 ML>G2";  // use V0 to suppress "charge" sound effect; v15 for max volume

ZumoMotors motors;

Pushbutton button(ZUMO_BUTTON); // pushbutton on pin 12
 
#define NUM_SENSORS 6
unsigned int sensor_values[NUM_SENSORS];

ZumoReflectanceSensorArray sensors(QTR_NO_EMITTER_PIN);

#define RA_SIZE 3  // number of readings to include in running average of accelerometer readings

class Accelerometer : public LSM303
{
  typedef struct acc_data_xy
  {
    unsigned long ms;
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
    
    void getAccelerationHeader(void)
    {
      Serial.print("millis    x      y     len     dir  | len_avg  dir_avg  |  avg_len");
      Serial.println();
    }
    
    void readAcceleration(void)
    {
      readAcc();
      if (a.x == last.x && a.y == last.y) return;
      
      last.ms = millis();
      last.x = a.x;
      last.y = a.y;
      last.len = sqrt(a.x*a.x + a.y*a.y);
      last.dir = atan2(a.x, a.y) * 180.0 / M_PI;
      
      ra_x.addValue(last.x);
      ra_y.addValue(last.y);
      ra_len_xy.addValue(last.len);
 
#ifdef LOG_SERIAL
     Serial.print(last.ms);
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
int forward_speed;

void setup()
{  
  // Initiate the Wire library and join the I2C bus as a master
  Wire.begin();
  
  // Initiate LSM303
  lsm303.init();
  lsm303.enable();
  
#ifdef LOG_SERIAL
  Serial.begin(9600);
  lsm303.getAccelerationHeader();
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
  forward_speed = FORWARD_SPEED;
  contact_lost_time = millis();
  loops_after_turning = 0;
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
  
  if (loops_after_turning > 0) loops_after_turning--;
  
  lsm303.readAcceleration(); 
  sensors.read(sensor_values);
  
  if (sensor_values[0] < QTR_THRESHOLD)
  {
    // if leftmost sensor detects line, reverse and turn to the right
#ifdef LOG_SERIAL
    Serial.print("turning right ...");
    Serial.println();
#endif
    // assume contact lost
    on_contact_lost();
    loops_after_turning = IGNORE_ACCERATION_AFTER_TURN;
    
    motors.setSpeeds(0,0);
    delay(STOP_DURATION);
    motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
    delay(REVERSE_DURATION);
    motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
    delay(turnDuration(true));
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
  }
  else if (sensor_values[5] < QTR_THRESHOLD)
  {
    // if rightmost sensor detects line, reverse and turn to the left
#ifdef LOG_SERIAL
    Serial.print("turning left ...");
    Serial.println();
#endif
    // assume contact lost
    on_contact_lost();
    loops_after_turning = IGNORE_ACCERATION_AFTER_TURN;
    
    motors.setSpeeds(0,0);
    delay(STOP_DURATION);
    motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
    delay(REVERSE_DURATION);
    motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
    delay(turnDuration(true));
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
  }
  else  // otherwise, go straight
  {
    if (check_for_contact()) on_contact_made();
    motors.setSpeeds(forward_speed, forward_speed);
  }
}

// randomized turn duration to improve searching
unsigned long turnDuration(boolean bRand)
{
  return bRand ? TURN_DURATION + random(5) * 50 : TURN_DURATION;
}
  
// check for contact, but ignore readings immediately after turning or losing contact
bool check_for_contact()
{
  return (lsm303.len_xy_avg() > 150.0) && \
    (loops_after_turning <= 0) && \
    (millis() - contact_lost_time > MIN_DELAY_BETWEEN_CONTACTS);
}

// sound horn and accelerate on contact -- fight or flight
void on_contact_made()
{
#ifdef LOG_SERIAL
  Serial.print("contact made");
  Serial.println();
#endif
  forward_speed = FULL_SPEED;
  buzzer.playFromProgramSpace(charge);
}

// reset forward speed
void on_contact_lost()
{
#ifdef LOG_SERIAL
  Serial.print("contact lost");
  Serial.println();
#endif
  contact_lost_time = millis();
  forward_speed = FORWARD_SPEED;
}

