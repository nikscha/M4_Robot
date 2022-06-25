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
#define STRIP_PIN 6
#define NR_LEDS 13
#define OUT DAC_CHANNEL_1

#define IDLE_THRESHOLD 100

#define NR_SAMPLES 32

#define TRIG 11
#define ECHO 14
#define SLOWDOWNDIST 50

void initPID();
void calibratePhotodiodes();
void readPhotodiodes();
void handleDistance();
void blinkLed(int led);
void checkForSearch();
void search();
void print(String s);
void print(double i);
void setSpeeds();
void initWiFi();
void CheckForConnections();
void EchoReceivedData();
void initAP();
void checkForPause();
void animateLEDS();

const char *ssid = "StrangerPings";
const char *password = "Fe@th3rwing";
const uint ServerPort = 23;

double setpoint, pidOutput;
// Specify tuning parameters
double Kp = 1.5, Ki = 0.05, Kd = 0.5;

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
bool tooClose = false;

u32_t now = 0;
u32_t lastBlink = 0;
u32_t lastStripUpdate = 0;
bool eyeMovingLeft = true;
bool ledOn = false;
bool pause_robot = false;
int search_led = 0;
bool search_for_player = false;

uint8_t c = 0;

PID myPID(&pidIn, &pidOutput, &setpoint, Kp, Ki, Kd, DIRECT);
// Motor shield default I2C Address: 0x30
// PWM frequency: 1000Hz(1kHz)
Motor ML;                                  // Motor A
Motor MR;                                  // Motor B
UltraSonicDistanceSensor dist(TRIG, ECHO); // initialisation class HCSR04 (trig pin , echo pin)
Ewma adcFilterL(0.1);
Ewma adcFilterR(0.1);
Ewma poidOutFilter(0.1);
Ewma distanceFilter(0.01);
WiFiServer Server(ServerPort);
WiFiClient RemoteClient;
Adafruit_NeoPixel strip(NR_LEDS, STRIP_PIN, NEO_GRB + NEO_KHZ800);
uint32_t RED = strip.Color(200, 0, 0);

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

  unsigned long start = millis();

  CheckForConnections();
  EchoReceivedData();
  checkForPause();
  checkForSearch();
  animateLEDS();
  blinkLed(LED);
  readPhotodiodes();
  handleDistance();
  setSpeeds();

  // print(millis()-start);
}

// returns true if the robot should search
void checkForSearch()
{
  search_for_player = sens_l < IDLE_THRESHOLD && sens_r < IDLE_THRESHOLD;
}

void animateLEDS()
{
  if (pause_robot)
  {
    strip.fill(RED);
    strip.show();
  }
  else if (search_for_player)
  {
    strip.clear();
    strip.setPixelColor(search_led, RED);
    strip.setPixelColor(search_led + 1, strip.Color(20, 0, 0));
    strip.setPixelColor(search_led - 1, strip.Color(20, 0, 0));

    now = millis();
    if (lastStripUpdate + 50 < now)
    {
      lastStripUpdate = now;
      if (search_led == 0)
        eyeMovingLeft = false;
      else if (search_led == NR_LEDS - 1)
        eyeMovingLeft = true;

      if (eyeMovingLeft)
        search_led--;
      else
        search_led++;
    }
    strip.show();
  }
  else if (tooClose)
  {
    strip.clear();
    strip.setPixelColor(map(distance, 10, SLOWDOWNDIST, 1, NR_LEDS), strip.Color(0, 200, 0));
    strip.show();
  }
  else
  {
    strip.clear();
    strip.setPixelColor(map(poidOutFilter.filter(pidOutput), -255, 255, 1, NR_LEDS), strip.Color(0, 0, 200));
    strip.show();
  }
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
  if (pause_robot)
  {
    ML.setmotor(_CCW, 0);
    MR.setmotor(_CW, 0);
  }
  else if (search_for_player) // rotates the robot unitl it finds a signal
  {
    ML.setmotor(_CCW, 30);
    MR.setmotor(_CCW, 30);
  }
  else // compute PID and speeds for each motor
  {
    myPID.Compute();
    // dac_output_voltage(OUT, map(pidOutput, -255, 255, 10, 255));

    if (tooClose) // reduces the base speed of the robot if it comes too close to an obstacle
    {
      base_speed = map(distance, 10, SLOWDOWNDIST, -10, 100);
    }
    else
    {
      base_speed = 220 - base_speed_multiplier * abs(pidOutput);
      base_speed = max(50, base_speed);
    }

    speed_l = base_speed + pidOutput;
    speed_r = base_speed + (pidOutput * -1);
    // speed_r = base_speed;

    reverse_l = speed_l < 0;
    reverse_r = speed_r < 0;

    speed_l = abs(speed_l);
    speed_r = abs(speed_r);

    speed_l = map(speed_l, 0, 255, 3, 45);
    speed_r = map(speed_r, 0, 255, 3, 45);

    ML.setmotor(reverse_l ? _CW : _CCW, speed_l);
    MR.setmotor(reverse_r ? _CCW : _CW, speed_r);
  }
}
// initialize the PID controller
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
  ML.setmotor(_CW, 30);
  MR.setmotor(_CW, 0);

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

// takes a distance measurement and sets the tooClose flag
void handleDistance()
{
  distance = distanceFilter.filter(dist.measureDistanceCm());
  tooClose = distance < SLOWDOWNDIST;
}

// blinks the led every 200 ms, call every loop
void blinkLed(int led)
{
  now = millis();
  if (now - lastBlink > 200)
  {
    digitalWrite(led, HIGH);
    lastBlink = now;
    ledOn = true;
    Serial.println(pidIn);
  }
  else if (now - lastBlink > 100 && ledOn)
  {
    digitalWrite(led, LOW);
    ledOn = false;
  }
}

// prints a string to the Serial Monitor
void print(String s)
{
  Serial.println(s);
}

// prints a double to the serial monitor
void print(double i)
{
  Serial.println(i, 16);
}

// connect to wifi
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

// initialize access point
void initAP()
{
  String hostname = "Robot";
  WiFi.setHostname(hostname.c_str()); // define hostname
  WiFi.mode(WIFI_AP);
  WiFi.softAP("IAMROBOT", password);
  Server.begin();
}

// sets pause flag if button is pressed
void checkForPause()
{
  if (digitalRead(PAUSE_BUTTON) == LOW)
  {
    pause_robot = !pause_robot;
    RemoteClient.println(pause_robot ? "paused robot" : "unpaused robot");
    delay(500);
  }
}
