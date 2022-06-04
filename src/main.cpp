#include <Arduino.h>
#include <LOLIN_I2C_MOTOR.h>
#include <Wire.h>
#include <PID_v1.h>

#define LEFT_INPUT 0
#define RIGHT_INPUT 0
#define PIN_OUTPUT 3

/*
TODO
sync github
  idle rotation
  moskau analysis
  kjhgfjhgf
  kytf
*/

// Define Variables we'll be connecting to
double Setpoint, Input, Output;

// Specify the links and initial tuning parameters
double Kp = 2, Ki = 5, Kd = 1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
LOLIN_I2C_MOTOR motor; // I2C address 0x30

void setup()
{
  Serial.begin(115200);
  initPID();

  Serial.println("Motor Shield Testing...");
  initMotor();
}

void loop()
{

  Input = analogRead(LEFT_INPUT) - analogRead(RIGHT_INPUT);
  myPID.Compute();
}

bool idle()
{
}

void initPID()
{
  // initialize the variables we're linked to
  Input = analogRead(LEFT_INPUT) - analogRead(RIGHT_INPUT);
  Setpoint = 0;

  // turn the PID on
  myPID.SetMode(AUTOMATIC);
}

void initMotor()
{
  while (motor.PRODUCT_ID != PRODUCT_ID_I2C_MOTOR) // wait motor shield ready.
    motor.getInfo();

  motor.changeFreq(MOTOR_CH_BOTH, 1000); // Change A & B 's Frequency to 1000Hz.
  /*
      motor.changeFreq(MOTOR_CH_A, 1000);//Change A 's Frequency to 1000Hz.
      motor.changeFreq(MOTOR_CH_B, 2000);//Change B 's Frequency to 2000Hz.
  */
  motor.changeStatus(MOTOR_CH_A, MOTOR_STATUS_CCW);
  motor.changeStatus(MOTOR_CH_B, MOTOR_STATUS_CW);
}