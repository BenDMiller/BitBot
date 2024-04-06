#include <Arduino.h>
#include "WiFi.h"
#include "ESPAsyncWebServer.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <iostream>
#include <string>

//Change test strings
String my_code[6] = {"Left Fast", "Right Fast","Both Fast", "Left Slow", "Right Slow", "Both Slow"};
int code_index = 0;

//Gyroscope and Acceleromter Setup
Adafruit_MPU6050 mpu;


// Setting Motor and PWM properties
const int freq = 30000;
const int resolution = 8;
const int leftpwmChannel = 0;
const int rightpwmChannel = 1;
//Duty cycle is between 0-255
int dutyCycleleft = 200;
int dutyCycleright = 200;
const int leftmotorpin1= 17;
const int leftmotorpin2 = 5;
const int leftmotorpwm = 16;
int leftmotordirection = 1;
const int rightmotorpin1= 18;
const int rightmotorpin2 = 19;
const int rightmotorpwm = 21;
int rightmotordirection = 1;

int leftdutycycletransmit = 0;
int leftdirectiontransmit = 1;
int rightdutycycletransmit = 0;
int rightdirectiontransmit = 1;


//Wifi communication Setup
//Set your access point network credentials
const char* ssid = "ESP32-Access-Point";
const char* password = "123456789";

const char* serverleftmotordutycycle = "http://192.168.4.1/dutycycleleft";
const char* serverleftmotordirection = "http://192.168.4.1/leftmotordirection";
const char* serverrightmotordutycycle = "http://192.168.4.1/dutycycleright";
const char* serverrightmotordirection = "http://192.168.4.1/rightmotordirection";

unsigned long previousMillis = 0;
unsigned long previousTime = 0;
const long interval = 5000; 
float AccX;
float AccY;
float AccZ;
float GyroX;
float GyroY;
float GyroZ;
float GyroXDelta;
float GyroYDelta;
float GyroZDelta;
float GyroAngleRoll = 0;
float GyroAnglePitch = 0;
float GyroXCallibrate = 0;
float GyroYCallibrate = 0;
float GyroZCallibrate = 0;
int CalibrateCount;
float AccAngleRoll;
float AccAnglePitch;
float ComplimentaryRoll = 0;
float ComplimentaryPitch = 0;

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

//General Constants and Variables
const char* System = "Controller";
// const char* System = "Bot";

String leftdutycycle;
String leftdirection;
String rightdutycycle;
String rightdirection;

//Get string from Wifi communiation
String httpGETRequest(const char* serverName) {
  WiFiClient client;
  HTTPClient http;
    
  // Your Domain name with URL path or IP address with path
  http.begin(client, serverName);
  
  // Send HTTP POST request
  int httpResponseCode = http.GET();
  
  String payload = "--"; 
  
  if (httpResponseCode>0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    payload = http.getString();
  }
  else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }
  // Free resources
  http.end();

  return payload;
}

void connectToServer(){
  WiFi.begin(ssid, password);
  Serial.println("Connecting");
  while(WiFi.status() != WL_CONNECTED) { 
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());
}

void setup() {
  // Serial port for debugging purposes
  Serial.begin(115200);
  
  if(System == "Controller"){
    //MPU Sensor
    Serial.println();
    // Try to initialize!
    if (!mpu.begin()) {
      Serial.println("Failed to find MPU6050 chip");
      while (1) {
        delay(10);
      }
    }
    Serial.println("MPU6050 Found!");

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    Serial.print("Accelerometer range set to: ");
    switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:
      Serial.println("+-2G");
      break;
    case MPU6050_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case MPU6050_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case MPU6050_RANGE_16_G:
      Serial.println("+-16G");
      break;
    }
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    Serial.print("Gyro range set to: ");
    switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:
      Serial.println("+- 250 deg/s");
      break;
    case MPU6050_RANGE_500_DEG:
      Serial.println("+- 500 deg/s");
      break;
    case MPU6050_RANGE_1000_DEG:
      Serial.println("+- 1000 deg/s");
      break;
    case MPU6050_RANGE_2000_DEG:
      Serial.println("+- 2000 deg/s");
      break;
    }

    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
    Serial.print("Filter bandwidth set to: ");
    switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ:
      Serial.println("260 Hz");
      break;
    case MPU6050_BAND_184_HZ:
      Serial.println("184 Hz");
      break;
    case MPU6050_BAND_94_HZ:
      Serial.println("94 Hz");
      break;
    case MPU6050_BAND_44_HZ:
      Serial.println("44 Hz");
      break;
    case MPU6050_BAND_21_HZ:
      Serial.println("21 Hz");
      break;
    case MPU6050_BAND_10_HZ:
      Serial.println("10 Hz");
      break;
    case MPU6050_BAND_5_HZ:
      Serial.println("5 Hz");
      break;
    }
    delay(1000);

    for (CalibrateCount=0; CalibrateCount<2000; CalibrateCount++){
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      GyroXCallibrate += g.gyro.x;
      GyroYCallibrate += g.gyro.y;
      GyroZCallibrate += g.gyro.z;
      delay(1);
    }
      GyroXCallibrate /= 2000;
      GyroYCallibrate /= 2000;
      GyroZCallibrate /= 2000;

    // Setting the ESP as an access point
    Serial.print("Setting AP (Access Point)…");
    // Remove the password parameter, if you want the AP (Access Point) to be open
    WiFi.softAP(ssid, password);

    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);
    
    server.on("/dutycycleleft", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send_P(200, "text/plain", String(leftdutycycletransmit).c_str());
      });
    server.on("/dutycycleright", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send_P(200, "text/plain", String(rightdutycycletransmit).c_str());
      });
    server.on("/leftmotordirection", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send_P(200, "text/plain", String(leftdirectiontransmit).c_str());
      });
    server.on("/rightmotordirection", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send_P(200, "text/plain", String(rightdirectiontransmit).c_str());
      });

  // Start server
  server.begin();
  }
  else if(System == "Bot"){
    // sets the pins as outputs:
    pinMode(leftmotorpin1, OUTPUT);
    pinMode(leftmotorpin2, OUTPUT);
    pinMode(leftmotorpwm, OUTPUT);
    pinMode(rightmotorpin1, OUTPUT);
    pinMode(rightmotorpin2, OUTPUT);
    pinMode(rightmotorpwm, OUTPUT);
    // configure LED PWM functionalitites
    ledcSetup(leftpwmChannel, freq, resolution);
    ledcSetup(rightpwmChannel, freq, resolution);
    // attach the channel to the GPIO to be controlled
    ledcAttachPin(leftmotorpwm, leftpwmChannel); 
    ledcAttachPin(rightmotorpwm, rightpwmChannel); 
    //Connect to Server
    connectToServer();
  }
}

void loop() {
  //Controller Code
  float time;
  if(System == "Controller"){
    //Change test data
    unsigned long currentTime = millis();
    // unsigned long currentMillis = millis();
    // if(currentMillis - previousMillis >= interval) {
    //   leftdutycycletransmit = (leftdutycycletransmit + 1)%125 + 130;
    //   rightdutycycletransmit = (rightdutycycletransmit + 1)%125 + 130;
    //   previousMillis = currentMillis;
    // }
  
    //MPU Sensor
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    time = currentTime-previousTime;
    AccX = a.acceleration.x/9.8 - 0.04;
    AccY = a.acceleration.y/9.8 + 0.01;
    AccZ = a.acceleration.z/9.8 + 0.04;
    GyroX = g.gyro.x - GyroXCallibrate;
    GyroY = g.gyro.y - GyroYCallibrate;
    GyroZ = g.gyro.z - GyroZCallibrate;
    GyroXDelta = GyroX*time/(PI/180)/1000;
    GyroYDelta = GyroY*time/(PI/180)/1000;
    GyroZDelta = GyroZ*time/(PI/180)/1000;
    
    AccAngleRoll = -atan(AccY/sqrt(pow(AccX, 2) + pow(AccZ, 2)))/(PI/180);
    AccAnglePitch = -atan(AccX/sqrt(pow(AccY, 2)+ pow(AccZ, 2)))/(PI/180);
    GyroAngleRoll = GyroAngleRoll - GyroX*time/(PI/180)/1000;
    GyroAnglePitch = GyroAnglePitch + GyroY*time/(PI/180)/1000;

    ComplimentaryRoll = ((ComplimentaryRoll-GyroXDelta)*0.98) + (0.02*AccAngleRoll);
    ComplimentaryPitch = ((ComplimentaryPitch-GyroYDelta)*0.98) + (0.02*AccAnglePitch);

    // int leftwheel = (pow(ComplimentaryRoll,3))/pow(60,2) + (ComplimentaryPitch*0.2)*((ComplimentaryRoll)/abs(ComplimentaryRoll));
    // int rightwheel = (pow(ComplimentaryRoll,3))/pow(60,2) + (-ComplimentaryPitch*0.2)*((ComplimentaryRoll)/abs(ComplimentaryRoll));
    
    int leftwheel;
    int rightwheel;
    // if(ComplimentaryPitch >= 0){
    //   leftwheel = (pow(ComplimentaryRoll,3))/pow(60,2) + (30/(1+pow(M_E, (-0.15*ComplimentaryPitch+5))))*((ComplimentaryRoll)/abs(ComplimentaryRoll));
    //   rightwheel = (pow(ComplimentaryRoll,3))/pow(60,2) + (-30/(1+pow(M_E, (-0.15*ComplimentaryPitch+5))))*((ComplimentaryRoll)/abs(ComplimentaryRoll));
    // }
    // else{
    //   leftwheel = (pow(ComplimentaryRoll,3))/pow(60,2) + (30/(1+pow(M_E, (-0.15*ComplimentaryPitch-5)))-30)*((ComplimentaryRoll)/abs(ComplimentaryRoll));
    //   rightwheel = (pow(ComplimentaryRoll,3))/pow(60,2) + (-(30/(1+pow(M_E, (-0.15*ComplimentaryPitch-5)))-30))*((ComplimentaryRoll)/abs(ComplimentaryRoll));
    // }
    if(ComplimentaryPitch >= 0){
      leftwheel = ComplimentaryRoll + (15/(1+pow(M_E, (-0.25*ComplimentaryPitch+5))))*((ComplimentaryRoll)/abs(ComplimentaryRoll));
      rightwheel = ComplimentaryRoll + (-15/(1+pow(M_E, (-0.25*ComplimentaryPitch+5))))*((ComplimentaryRoll)/abs(ComplimentaryRoll));
    }
    else{
      leftwheel = ComplimentaryRoll + (15/(1+pow(M_E, (-0.25*ComplimentaryPitch-5)))-30)*((ComplimentaryRoll)/abs(ComplimentaryRoll));
      rightwheel = ComplimentaryRoll + (-(15/(1+pow(M_E, (-0.25*ComplimentaryPitch-5)))-30))*((ComplimentaryRoll)/abs(ComplimentaryRoll));
    }

    leftdutycycletransmit = min<int>((132 + abs(leftwheel)), 190);
    rightdutycycletransmit = min<int>((132 + abs(rightwheel)), 190);

    // leftdutycycletransmit = min<int>((130 + abs(ComplimentaryRoll)-5), 180);
    // rightdutycycletransmit = min<int>((130 - 1 + abs(ComplimentaryRoll)-5), 180);
    leftdirectiontransmit = leftwheel >=0 ? 1:-1;
    rightdirectiontransmit = rightwheel >=0 ? 1:-1;

    Serial.print("ComplimentaryAngleRoll:");
    Serial.print(ComplimentaryRoll);
    Serial.print(",");
    Serial.print("ComplimentaryAnglePitch:");
    Serial.print(ComplimentaryPitch);
    Serial.print(",");
    Serial.print("LeftWheel:");
    Serial.print(leftwheel);
    Serial.print(",");
    Serial.print("RightWheel:");
    Serial.print(rightwheel);
    Serial.print(",");
    Serial.print("LeftDirection:");
    Serial.print(leftdirectiontransmit);
    Serial.print(",");
    Serial.print("RightDirection:");
    Serial.println(rightdirectiontransmit);

    previousTime = currentTime;
    
  }
  //Bot Code
  else if(System == "Bot"){
    //Print Data
    // unsigned long currentMillis = millis();
    // if(currentMillis - previousMillis >= interval) {
      // Check WiFi connection status
      if(WiFi.status()== WL_CONNECTED ){
        leftdutycycle = httpGETRequest(serverleftmotordutycycle);
        leftdirection = httpGETRequest(serverleftmotordirection);
        rightdutycycle = httpGETRequest(serverrightmotordutycycle);
        rightdirection = httpGETRequest(serverrightmotordirection);
        Serial.println("LDC: " + leftdutycycle);
        Serial.println("RDC: " + rightdutycycle);
        // save the last HTTP GET Request
        // previousMillis = currentMillis;
      } 
      else{
        Serial.println("WiFi Disconnected");
        ledcWrite(leftpwmChannel, 0);
        ledcWrite(rightpwmChannel, 0);
        connectToServer();
      }
    // }

    dutyCycleleft = leftdutycycle.toInt();
    leftmotordirection = leftdirection.toInt();
    dutyCycleright = rightdutycycle.toInt();
    rightmotordirection = rightdirection.toInt();

    if(leftmotordirection == 1){
      digitalWrite(leftmotorpin1, LOW);
      digitalWrite(leftmotorpin2, HIGH);
    }
    else if(leftmotordirection == -1){
      digitalWrite(leftmotorpin1, HIGH);
      digitalWrite(leftmotorpin2, LOW);
    }
    if(rightmotordirection == 1){
      digitalWrite(rightmotorpin1, LOW);
      digitalWrite(rightmotorpin2, HIGH);
    }
    else if(rightmotordirection == -1){
      digitalWrite(rightmotorpin1, HIGH);
      digitalWrite(rightmotorpin2, LOW);
    }
    ledcWrite(leftpwmChannel, dutyCycleleft);
    ledcWrite(rightpwmChannel, dutyCycleright);
  }
}