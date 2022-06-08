#include <Arduino.h>
#include <LOLIN_I2C_MOTOR.h>
#include <Wire.h>
#include <PID_v1.h>
#include <HCSR04.h>
#include <driver/dac.h>

#define LED 15
#define LEFT_INPUT 16
#define RIGHT_INPUT 12
#define IDLE_OFFSET -20
#define NR_SAMPLES 32

#define TRIG 1
#define ECHO 2
#define STOP_DISTANCE 30

void initPID();
void initMotor();
void calibratePhotodiodes();
void readPhotodiodes();
void handleDistance();
void blinkLed();
bool checkForSearch();
void search();

/*
TODO
  idle rotation
  moskau analysis

*/

// Define Variables we'll be connecting to
double setpoint, pidOutput;
// Specify the links and initial tuning parameters
double Kp = 0.0002, Ki = 0.2, Kd = 0.2;

float cal_r = 0; // holds the average value of the photodiodes
float cal_l = 0;

int16_t sens_l = 0; // holds the denoised reading from the photodiodes
int16_t sens_r = 0;
double pidIn = 0; // holds sens_l - sens_r

int speed_l = 0;
int speed_r = 0;
int base_speed = 100;
bool reverse_l = false;
bool reverse_r = false;

double distance = 0;

uint16_t lastBlink = 0;

uint8_t c=0;

PID myPID(&pidIn, &pidOutput, &setpoint, Kp, Ki, Kd, DIRECT);
LOLIN_I2C_MOTOR motor;                     // I2C address 0x30
UltraSonicDistanceSensor dist(TRIG, ECHO); // initialisation class HCSR04 (trig pin , echo pin)

void setup()
{
  pinMode(LED, OUTPUT);
  pinMode(LEFT_INPUT, INPUT);
  pinMode(RIGHT_INPUT, INPUT);
  pinMode(39, OUTPUT);
  dac_output_enable(DAC_CHANNEL_1);
  dac_output_enable(DAC_CHANNEL_2);

  digitalWrite(LED, HIGH); // turn onboard led on to indicate functionaity

  Serial.begin(115200);
  // while (!Serial)
  //   ;
  delay(2000);
  Serial.println("Begin!");

  calibratePhotodiodes();
  Serial.println("cal_l");
  Serial.println(cal_l);
  Serial.println("cal_r");
  Serial.println(cal_r);

  initPID();

  // initMotor();
}

void loop()
{
 
  u32_t start = millis();
  blinkLed();
  readPhotodiodes();
  // if (checkForSearch())
  //   search();
  // handleDistance();

  myPID.Compute();
  //Serial.println(pidOutput);
  dac_output_voltage(DAC_CHANNEL_1, pidOutput / 2 + 128);
  dac_output_voltage(DAC_CHANNEL_2, map(pidIn,0,255,-1000,1000));

  speed_l = base_speed + pidOutput;
  speed_r = base_speed + pidOutput * -1;
  reverse_l = speed_l < 0;
  reverse_r = speed_r < 0;

  // motorA.setDirection(reverse_l);
  // motorB.setDirection(reverse_r);
  // motorA.setSpeed(speed_l);
  // motorB.setSpeed(speed_r);

  Serial.println(millis() - start);
}

void initPID()
{
  setpoint = 0;
  myPID.SetOutputLimits(-UINT8_MAX, UINT8_MAX);
  myPID.SetSampleTime(5);
  myPID.SetMode(AUTOMATIC);
}

void initMotor()
{
  while (motor.PRODUCT_ID != PRODUCT_ID_I2C_MOTOR)
  { // wait motor shield ready.
    motor.getInfo();
    Serial.println("Waiting for motor");
    Serial.println(motor.PRODUCT_ID);
  }

  motor.changeFreq(MOTOR_CH_BOTH, 1000); // Change A & B 's Frequency to 1000Hz.
  /*
      motor.changeFreq(MOTOR_CH_A, 1000);//Change A 's Frequency to 1000Hz.
      motor.changeFreq(MOTOR_CH_B, 2000);//Change B 's Frequency to 2000Hz.
  */
  motor.changeStatus(MOTOR_CH_A, MOTOR_STATUS_CCW);
  motor.changeStatus(MOTOR_CH_B, MOTOR_STATUS_CW);
}

void calibratePhotodiodes()
{
  cal_l = analogRead(LEFT_INPUT);
  cal_r = analogRead(RIGHT_INPUT);

  for (int16_t i = 1; i < INT16_MAX; i++)
  {
    uint16_t l = analogRead(LEFT_INPUT);
    uint16_t r = analogRead(RIGHT_INPUT);
    cal_l = (l + (i - 1) * cal_l) / i;
    cal_r = (r + (i - 1) * cal_r) / i;
  }
}

void readPhotodiodes() // takes 2 ms to run
{
  uint16_t l = analogRead(LEFT_INPUT);
  uint16_t r = analogRead(RIGHT_INPUT);

  for (uint16_t i = 1; i < NR_SAMPLES; i++)
  {
    l += analogRead(LEFT_INPUT);
    r += analogRead(RIGHT_INPUT);
  }
  sens_l = l / NR_SAMPLES;
  sens_r = r / NR_SAMPLES;
  pidIn = sens_l - sens_r;
}

void handleDistance()
{
  distance = dist.measureDistanceCm();
  if (distance < STOP_DISTANCE)
  {
    delay(1000);
  }
}

void blinkLed()
{
  uint16_t now = millis();
  if (now - lastBlink > 200)
  {
    digitalWrite(LED, HIGH);
    lastBlink = now;
  }
  else if (now - lastBlink > 100)
  {
    digitalWrite(LED, LOW);
  }
}
// returns true if the robot should search
bool checkForSearch()
{
  return !(sens_l < cal_l + IDLE_OFFSET || sens_r < cal_r + IDLE_OFFSET);
}

void search()
{
  while (checkForSearch)
  {
    readPhotodiodes();
    // turn
    // flash leds
  }
}