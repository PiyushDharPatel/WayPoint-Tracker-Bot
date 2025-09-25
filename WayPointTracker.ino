#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#define INCLUDE_TERMINAL_MODULE

#include <AltSoftSerial.h>
#include <Dabble.h>
#include <TinyGPS++.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <SoftwareSerial.h>

float minrawLat = 30;
const int IN1 = 6, IN2 = 7, IN3 = 2, IN4 = 3;

TinyGPSPlus gps;
AltSoftSerial GPS;
bool isAutoMode = false;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
float destLat = 0.0, destLon = 0.0;
bool destinationSet = false;

#define GPS_AVG_WINDOW 5
float latBuffer[GPS_AVG_WINDOW] = {0};
float lonBuffer[GPS_AVG_WINDOW] = {0};
int bufferIndex = 0;
unsigned long lastUpdate = 0;

void setup() {
  Serial.begin(9600);
  GPS.begin(9600);
  Dabble.begin(9600, 4, 5);
  mag.begin();

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.println("Ready. Manual mode active.");
}

void loop() {
  if (GPS.available() > 0) gps.encode(GPS.read());
  Dabble.processInput();

  if (Terminal.available()) {
    String input = Terminal.readString();
    input.trim();
    int commaIndex = input.indexOf(',');
    if (commaIndex > 0) {
      destLat = input.substring(0, commaIndex).toFloat();
      destLon = input.substring(commaIndex + 1).toFloat();
      destinationSet = true;
      isAutoMode = true;
    }
  }

  if (GamePad.isSelectPressed()) {
    if (gps.location.lat() && gps.satellites.value() >= 4) {
      destLat = gps.location.lat();
      destLon = gps.location.lng();
      destinationSet = true;
    }
  }

  if (GamePad.isStartPressed()) {
    isAutoMode = !isAutoMode;
    delay(500);
  }

  if (isAutoMode)
    handleAutoMode();
  else
    handleManualMode();
}

void handleAutoMode() {
  if (!destinationSet || !gps.location.lat() || gps.satellites.value() < 4) return;

  if (millis() - lastUpdate >= 1000) {
    float rawLat = gps.location.lat();
    float rawLon = gps.location.lng();

    latBuffer[bufferIndex] = rawLat;
    lonBuffer[bufferIndex] = rawLon;
    bufferIndex = (bufferIndex + 1) % GPS_AVG_WINDOW;

    float currLat = getAverage(latBuffer);
    float currLon = getAverage(lonBuffer);

    float dist = distanceBetween(currLat, currLon, destLat, destLon);
    if (dist < 4.0) {
      stopMotors();
    } else {
      float bearing = calculateBearing(currLat, currLon, destLat, destLon);
      float heading = getCompassHeading();
      float diff = bearing - heading;

      if (diff > 180) diff -= 360;
      if (diff < -180) diff += 360;

      if (abs(diff) < 15)
        moveForward();
      else if (diff > 0)
        turnRight();
      else
        turnLeft();
    }

    lastUpdate = millis();
  }
}

void handleManualMode() {
  if (GamePad.isUpPressed())
    moveForward();
  else if (GamePad.isDownPressed())
    moveBackward();
  else if (GamePad.isLeftPressed())
    turnLeft();
  else if (GamePad.isRightPressed())
    turnRight();
  else
    stopMotors();
}

float getAverage(float *buffer) {
  float sum = 0;
  for (int i = 0; i < GPS_AVG_WINDOW; i++) sum += buffer[i];
  return sum / GPS_AVG_WINDOW;
}

float distanceBetween(float lat1, float lon1, float lat2, float lon2) {
  const float R = 6371000;
  float dLat = radians(lat2 - lat1);
  float dLon = radians(lon2 - lon1);
  float a = sin(dLat / 2) * sin(dLat / 2) +
            cos(radians(lat1)) * cos(radians(lat2)) *
            sin(dLon / 2) * sin(dLon / 2);
  return R * 2 * atan2(sqrt(a), sqrt(1 - a));
}

float calculateBearing(float lat1, float lon1, float lat2, float lon2) {
  float y = sin(radians(lon2 - lon1)) * cos(radians(lat2));
  float x = cos(radians(lat1)) * sin(radians(lat2)) -
            sin(radians(lat1)) * cos(radians(lat2)) * cos(radians(lon2 - lon1));
  return fmod((degrees(atan2(y, x)) + 360), 360);
}

float getCompassHeading() {
  sensors_event_t event;
  mag.getEvent(&event);
  float heading = atan2(event.magnetic.y + 47.45, event.magnetic.x - 8.55);
  if (heading < 0) heading += 2 * PI;
  if (heading >= 2 * PI) heading -= 2 * PI;
  return degrees(heading);
}

void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void moveBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
