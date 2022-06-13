#include <Arduino.h>
#include "WEMOS_Motor.h"
#include <Wire.h>
#include <PID_v1.h>
#include <HCSR04.h>
#include <driver/dac.h>
#include <Ewma.h>
#include <WiFi.h>
#include <string.h>

#define LED 15
#define LEFT_INPUT 3
#define RIGHT_INPUT 5
#define STOP_BUTTON 1
#define START_BUTTON 2
#define SEARCH_LED 40
#define STOP_LED 38
#define OUT DAC_CHANNEL_1

#define IDLE_THRESHOLD 20

#define NR_SAMPLES 32

#define TRIG 1
#define ECHO 2
#define STOP_DISTANCE 30

void initPID();
void calibratePhotodiodes();
void readPhotodiodes();
void handleDistance();
void blinkLed(int led);
bool checkForSearch();
void search();
void print(String s);
void print(double i);
void setSpeeds();
void initWiFi();
void CheckForConnections();
void EchoReceivedData();

const char *ssid = "StrangerPings";
const char *password = "Fe@th3rwing";
const uint ServerPort = 23;

double setpoint, pidOutput;
// Specify tuning parameters
double Kp = 0.05, Ki = 0.2, Kd = 0;

double filter_value = 0.0001;
double filter_value2 = 0.9998;

float cal_l = 0; // holds the average value of the photodiodes
float cal_r = 0;

double sens_l = 0; // holds the denoised reading from the photodiodes
double sens_r = 0;
double pidIn = 0; // holds sens_l - sens_r

int speed_l = 0;
int speed_r = 0;
int base_speed = 5;
bool reverse_l = false;
bool reverse_r = false;

double distance = 0;

u32_t lastBlink = 0;
bool ledOn = false;

uint8_t c = 0;

PID myPID(&pidIn, &pidOutput, &setpoint, Kp, Ki, Kd, DIRECT);
// Motor shield default I2C Address: 0x30
// PWM frequency: 1000Hz(1kHz)
Motor ML;                                  // Motor A
Motor MR;                                  // Motor B
UltraSonicDistanceSensor dist(TRIG, ECHO); // initialisation class HCSR04 (trig pin , echo pin)
Ewma adcFilterL(0.1);
Ewma adcFilterR(0.1);
WiFiServer Server(ServerPort);
WiFiClient RemoteClient;

void setup()
{
  pinMode(LED, OUTPUT);
  pinMode(LEFT_INPUT, INPUT);
  pinMode(RIGHT_INPUT, INPUT);
  pinMode(39, OUTPUT);
  dac_output_enable(OUT);
  digitalWrite(LED, HIGH); // turn onboard led on to indicate functionaity

  Serial.begin(115200);
  delay(2000);
  Serial.println("Begin!");

  Wire.setPins(33, 35);
  ML.start(0x30, _MOTOR_A, 1000); // Motor A
  MR.start(0x30, _MOTOR_B, 1000); // Motor B

  initWiFi();
  calibratePhotodiodes();
  initPID();
}

void loop()
{
  unsigned long start = millis();
  blinkLed(LED);
  CheckForConnections();
  readPhotodiodes();
  // if (checkForSearch())
  //   search();
  // handleDistance();

  setSpeeds();
  EchoReceivedData();
}

void CheckForConnections()
{
  if (Server.hasClient())
  {
    // If we are already connected to another computer,
    // then reject the new connection. Otherwise accept
    // the connection.
    if (RemoteClient.connected())
    {
      Serial.println("Connection rejected");
      Server.available().stop();
    }
    else
    {
      Serial.println("Connection accepted");
      RemoteClient = Server.available();
    }
  }
}

void EchoReceivedData()
{
  uint8_t ReceiveBuffer[30];
  while (RemoteClient.connected() && RemoteClient.available())
  {
    int Received = RemoteClient.read(ReceiveBuffer, sizeof(ReceiveBuffer));
    RemoteClient.write(ReceiveBuffer, Received);
    switch (ReceiveBuffer[0])
    {
    case 'p':
    case 'P':
      Kp = RemoteClient.parseFloat();
      RemoteClient.print("Kp value is now ");
      RemoteClient.println(Kp);
      break;
    case 'i':
    case 'I':
      Ki = RemoteClient.parseFloat();
      RemoteClient.print("Ki value is now ");
      RemoteClient.println(Ki);
      break;
    case 'd':
    case 'D':
      Kd = RemoteClient.parseFloat();
      RemoteClient.print("Kd value is now ");
      RemoteClient.println(Kd);
      break;
    }
  }
}

void setSpeeds()
{
  myPID.Compute();

  dac_output_voltage(OUT, map(pidOutput, -255, 255, 10, 255));

  speed_l = base_speed + pidOutput;
  speed_r = base_speed + pidOutput * -1;

  reverse_l = speed_l < 0;
  reverse_r = speed_r < 0;

  speed_l = abs(speed_l);
  speed_r = abs(speed_r);

  speed_l = map(speed_l, 0, 255, 0, 30);
  speed_r = map(speed_r, 0, 255, 0, 30);

  ML.setmotor(reverse_l ? 1 : 2, speed_l);
  MR.setmotor(reverse_r ? 1 : 2, speed_r);
}

void initPID()
{
  setpoint = 0;
  myPID.SetOutputLimits(-255, 255);
  myPID.SetSampleTime(5);
  myPID.SetMode(AUTOMATIC);
}

// take lots of readings of the photodiodes to establish a baseline
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

  Serial.println("cal_l:");
  Serial.println(cal_l);
  Serial.println("cal_r:");
  Serial.println(cal_r);
}

// take a reading from the photodiodes and filter it
void readPhotodiodes() // takes 2 ms to run
{
  delayMicroseconds(10);
  double l = analogRead(LEFT_INPUT) - cal_l;
  delayMicroseconds(10);
  double r = analogRead(RIGHT_INPUT) - cal_r;

  sens_l = adcFilterL.filter(l);
  sens_r = adcFilterR.filter(r);

  // print(sens_r);
  // print(sens_l);
  // print("");

  pidIn = sens_l - sens_r;
}

// takes a distance measurement and if an object is too close, pause for a second
void handleDistance()
{
  // exponentiall smooth so slow down response time?
  distance = dist.measureDistanceCm();
  if (distance < STOP_DISTANCE)
  {
    digitalWrite(STOP_LED, HIGH);
    delay(1000);
    digitalWrite(STOP_LED, LOW);
  }
}

// blinks the led every 200 ms, call every loop
void blinkLed(int led)
{
  u32_t now = millis();
  if (now - lastBlink > 200)
  {
    digitalWrite(led, HIGH);
    lastBlink = now;
    ledOn = true;
  }
  else if (now - lastBlink > 100 && ledOn)
  {
    digitalWrite(led, LOW);
    ledOn = false;
  }
}
// returns true if the robot should search
bool checkForSearch()
{
  return (sens_l < IDLE_THRESHOLD && sens_r < IDLE_THRESHOLD);
}

// turns the robot until photodiodes read above a certain value
void search()
{
  ML.setmotor(_CW, 20);
  MR.setmotor(_CCW, 20);

  while (checkForSearch)
  {
    readPhotodiodes();
    blinkLed(LED);
  }
}

void print(String s)
{
  Serial.println(s);
}

void print(double i)
{
  Serial.println(i, 16);
}

void initWiFi()
{
  String hostname = "Robot";
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(hostname.c_str()); // define hostname
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());

  Server.begin();
}