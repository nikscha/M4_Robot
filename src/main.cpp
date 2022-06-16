#include <Arduino.h>
#include "WEMOS_Motor.h"
#include <Wire.h>
#include <PID_v1.h>
#include <HCSR04.h>
#include <driver/dac.h>
#include <Ewma.h>
#include <WiFi.h>
#include <string.h>
#include <Adafruit_NeoPixel.h>

#define LED 15
#define LEFT_INPUT 13
#define RIGHT_INPUT 12
#define STOP_BUTTON 1
#define START_BUTTON 2
#define PAUSE_BUTTON 0
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
void initAP();
void checkForPause();

const char *ssid = "StrangerPings";
const char *password = "Fe@th3rwing";
const uint ServerPort = 23;

double setpoint, pidOutput;
// Specify tuning parameters
double Kp = 3, Ki = 0.05, Kd = 0;

double filter_value = 0.0001;
double filter_value2 = 0.9998;

float cal_l = 0; // holds the average value of the photodiodes
float cal_r = 0;

double sens_l = 0; // holds the denoised reading from the photodiodes
double sens_r = 0;
double pidIn = 0; // holds sens_l - sens_r

int speed_l = 0;
int speed_r = 0;
int base_speed = 0;
double base_speed_multiplier = 0.14; // lower means faster
bool reverse_l = false;
bool reverse_r = false;

double distance = 0;

u32_t lastBlink = 0;
bool ledOn = false;
bool pause_robot = false;

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
Adafruit_NeoPixel strip(10, 3, NEO_GRB + NEO_KHZ800);

void setup()
{
  pinMode(LED, OUTPUT);
  pinMode(LEFT_INPUT, INPUT);
  pinMode(RIGHT_INPUT, INPUT);
  pinMode(PAUSE_BUTTON, INPUT_PULLUP);

  pinMode(39, OUTPUT);
  dac_output_enable(OUT);
  digitalWrite(LED, HIGH); // turn onboard led on to indicate functionaity

  Serial.begin(115200);
  strip.begin();
  strip.setBrightness(255);
  strip.fill(strip.Color(255, 255, 255));

  Wire.setPins(33, 35);
  ML.start(0x30, _MOTOR_B, 1000); // Motor A
  MR.start(0x30, _MOTOR_A, 1000); // Motor B

  // initWiFi();
  initAP();
  delay(2000);
  calibratePhotodiodes();
  initPID();
}

void loop()
{
  strip.fill(strip.Color(255, 255, 255));
  unsigned long start = millis();

  do
  {
    CheckForConnections();
    EchoReceivedData();
    checkForPause();
  } while (pause_robot);

  blinkLed(LED);
  readPhotodiodes();
  setSpeeds();

  // if (checkForSearch())
  //   search();
  // handleDistance();

  // print(millis()-start);
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
    char c = RemoteClient.read();
    switch (c)
    {
    case 'p':
    case 'P':
      Kp = RemoteClient.parseFloat();
      RemoteClient.print("Kp value is now ");
      RemoteClient.println(Kp, 5);
      break;
    case 'i':
    case 'I':
      Ki = RemoteClient.parseFloat();
      RemoteClient.print("Ki value is now ");
      RemoteClient.println(Ki, 5);
      break;
    case 'd':
    case 'D':
      Kd = RemoteClient.parseFloat();
      RemoteClient.print("Kd value is now ");
      RemoteClient.println(Kd, 5);
      break;
    case 'b':
    case 'B':
      base_speed = RemoteClient.parseInt();
      RemoteClient.print("base_speed is now ");
      RemoteClient.println(base_speed);
      break;
    case 'c':
    case 'C':
      RemoteClient.println("Calibrating");
      calibratePhotodiodes();
      RemoteClient.println("DONE!");

      break;
    case 'm':
    case 'M':
      base_speed_multiplier = RemoteClient.parseFloat();
      RemoteClient.print("base_speed_multiplier is now ");
      RemoteClient.println(base_speed_multiplier);
      break;
    case 's':
    case 'S':
      pause_robot = !pause_robot;
      RemoteClient.println(pause_robot ? "paused robot" : "unpaused robot");

      break;
    default:
      RemoteClient.flush();
      RemoteClient.println("Not recognized");
      break;
    }
  }
}

void setSpeeds()
{
  myPID.Compute();
  dac_output_voltage(OUT, map(pidOutput, -255, 255, 10, 255));

  base_speed = 180 - base_speed_multiplier * abs(pidOutput);
  base_speed = max(50, base_speed);

  speed_l = base_speed + pidOutput;
  speed_r = base_speed + (pidOutput * -1);

  reverse_l = speed_l < 0;
  reverse_r = speed_r < 0;

  speed_l = abs(speed_l);
  speed_r = abs(speed_r);

  speed_l = map(speed_l, 0, 255, 3, 35);
  speed_r = map(speed_r, 0, 255, 3, 35);

  ML.setmotor(reverse_l ? _CW : _CCW, speed_l);
  MR.setmotor(reverse_r ? _CCW : _CW, speed_r);
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
  ML.setmotor(_CW, 20);
  MR.setmotor(_CW, 20);

  cal_l = analogRead(LEFT_INPUT);
  cal_r = analogRead(RIGHT_INPUT);

  for (int16_t i = 1; i < INT16_MAX; i++)
  {
    uint16_t l = analogRead(LEFT_INPUT);
    uint16_t r = analogRead(RIGHT_INPUT);
    cal_l = (l + (i - 1) * cal_l) / i;
    cal_r = (r + (i - 1) * cal_r) / i;
  }

  ML.setmotor(_CW, 0);
  MR.setmotor(_CW, 0);

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
  strip.clear();
  // strip.setPixelColor()
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
  // WiFi.mode(WIFI_STA);
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

void initAP()
{
  String hostname = "Robot";
  WiFi.setHostname(hostname.c_str()); // define hostname
  WiFi.mode(WIFI_AP);
  WiFi.softAP("IAMROBOT", password);
  Server.begin();
}

void checkForPause()
{
  if (digitalRead(PAUSE_BUTTON) == LOW)
  {
    pause_robot = !pause_robot;
    RemoteClient.println(pause_robot ? "paused robot" : "unpaused robot");
    delay(500);
  }

  if (pause_robot)
  {
    digitalWrite(LED, LOW);
    strip.fill(strip.Color(255, 0, 0));
    ML.setmotor(_CW, 0);
    MR.setmotor(_CCW, 0);
  }
}
