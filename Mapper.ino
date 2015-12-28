/*
 * @author("Raymond Blum" <raymond@insanegiantrobots.com>)
 *
 * Various mapping of N distance sensors to an LED matrix with IR remote display mode switching.
 *
 * Copyright (c) 2015 by Raymond Blum
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published  by the Free Software Foundation; eitfher
 * version 2.1 of the License, or (at your option) any later version.
 *
 **/
 
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_LEDBackpack.h>
#include <NewPing.h>
#include <IRremote.h>
// ---------------------------------------------------------------------------

// #define DEBUG_

#define BUZZ_TONE 300
#define BUZZ_DUR 500l
#define SHINE_DUR 1000l

#define SHIFT_IN_THRESHOLD -25
#define SHIFT_OUT_THRESHOLD 25

#define MAP_DISPLAY_TIERS 3

int buzzing_, shining_;
int displayed_;
unsigned long buzz_until_, shine_until_;
#define SONAR_NUM    8    // Number of sonar sensors we are reading.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define MIN_DISTANCE 0 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 30 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM], prev_cm[SONAR_NUM];     // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

unsigned int closestReading, prevClosestReading;       // The closest reading from all of the sensors
uint8_t closestSensor, prevClosestSensor;              // The sensor that had the closest reading

unsigned int farthestReading, prevFarthestReading;       // The closest reading from all of the sensors
uint8_t farthestSensor, prevFarthestSensor;              // The sensor that had the closest reading

// Avoid using 0,1 which are UART
#define RX_ 0
#define TX_ 1

// Use i2C for our LED matrix
#define SDA_ 18
#define SCL_ 19

// Connect an IR sensor on pin 2
#define IR_SENSOR_ 2

// Connect a speaker or buzzer on pin 3
#define BUZZER_ 3

// There's a built in LED on 13, hook any additional one to the same pin
#define LED_ 13

// Use pins 5-12 on side 1, skip 13 (LED) 18:19 (i2C) on side 2
NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  // side 1
  NewPing(5, 6, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(7, 8, MAX_DISTANCE),
  NewPing(9, 10, MAX_DISTANCE),
  NewPing(11, 12, MAX_DISTANCE),
  // side 2
  NewPing(14, 15, MAX_DISTANCE),
  NewPing(16, 17, MAX_DISTANCE),
  NewPing(20, 21, MAX_DISTANCE),
  NewPing(22, 23, MAX_DISTANCE),
};

IRrecv irrecv(IR_SENSOR_);
decode_results results;

Adafruit_BicolorMatrix matrix = Adafruit_BicolorMatrix();

// The threshold of "hot" objects' proximity
#define MIN_HOT 10

#define BIG_SHIFT_THRESHOLD 15

// The threshold of "warm" objects' proximity
#define MIN_WARM 100

// The bounds of our display map
#define MIN_X 0
#define MIN_Y 0
#define MAX_X 7
#define MAX_Y 7

#define REMOTE_CMD_NONE 0
#define REMOTE_CMD_1 1

int remote_cmd_;

#define DISPLAY_MODE_HISTOGRAM 0
#define DISPLAY_MODE_QUADRANTS 1
int display_mode_;

typedef struct point {
  int x, y;
} point_;

typedef struct spot {
  point left;
  point center;
  point right;
  point tail;
} spot_;

typedef struct spotNode {
  spot object;
  spotNode* nextObject;
} spotNode_;

// Our current points of interest on the map
spotNode* objects;

void setup() {
  #ifdef DEBUG_
  Serial.begin(115200);
  Serial.println("setup()");
  #endif
  pinMode(LED_, OUTPUT);
  pinMode(BUZZER_, OUTPUT);
  buzz();
  shine();

  display_mode_ = DISPLAY_MODE_HISTOGRAM;
  displayed_ = 0;

  irrecv.enableIRIn(); // Start the receiver
  matrix.begin(0x70);  // pass in the address
  matrix.clear();      // clear display
  matrix.drawPixel(1, 1, LED_YELLOW);
  matrix.drawPixel(1, 2, LED_GREEN);
  matrix.drawPixel(2, 1, LED_RED);
  matrix.writeDisplay();  // write the changes we just made to the display

  digitalWrite(LED_, LOW);
  pingTimer[0] = millis() + 75;              // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++) {  // Set the starting time for each sensor.
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
  }
  farthestReading = MIN_DISTANCE;
  farthestSensor = 0;
  closestReading = MAX_DISTANCE;
  closestSensor = 0;
  objects = 0l;
}

void stopBuzzing() {
  #ifdef DEBUG_
  Serial.println("stopBuzzing");
  #endif
  buzzing_ = 0;
  buzz_until_ = 0l;
  noTone(BUZZER_);
}

void stopShining() {
  shining_ = 0;
  shine_until_ = 0l;
  digitalWrite(LED_, LOW);
}

enum objectShapes {
  toLeft,   // we specify the rightmost of 2 points
  toBottom, // we specify the topmost of 2 points
  topLeftCorner,    // we specify the topLeftCorner of 3 points
  topRightCorner,    // we specify the topRightCorner of 3 points
  bottomLeftCorner,    // we specify the bottomLeftCorner of 3 points
  bottomRightCorner    // we specify the bottomRightCorner of 3 points
};

void setObject(int shape, struct point location, struct point tail) {

}

void refreshSensors() {
  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.
    if (millis() >= pingTimer[i]) {         // Is it this sensor's time to ping?
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
      sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      currentSensor = i;                          // Sensor being accessed.
      sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
  }
}

void echoCheck() { // If ping received, set this sensor's distance
  if (sonar[currentSensor].check_timer()) {
    int pingReading = sonar[currentSensor].ping_result;
    int pingDistance = pingReading / US_ROUNDTRIP_CM;
    if (pingDistance != 0 && pingDistance != MAX_DISTANCE) {
      prev_cm[currentSensor] = cm[currentSensor];
      cm[currentSensor] = pingDistance;
      updateSensorStats();
    }
  }
}

void updateSensorStats() { // Sensor ping  complete, reexmine the full set of readings
  prevClosestSensor = closestSensor;
  prevClosestReading = closestReading;
  closestReading = MAX_DISTANCE;

  prevFarthestSensor = farthestSensor;
  prevFarthestReading = farthestReading;
  farthestReading = MIN_DISTANCE;

  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    #ifdef DEBUG_
    Serial.print(i);
    Serial.print("=");
    Serial.print(cm[i]);
    Serial.print("cm ");
    #endif
    if (cm[i] < closestReading) {
      closestReading = cm[i];
      closestSensor = i;
      #ifdef DEBUG_
      Serial.print("Set closest to: ");
      Serial.print(closestReading);
      Serial.print(" for sensor: ");
      Serial.println(closestSensor);
      #endif
    }
    if (cm[i] > farthestReading) {
      farthestReading = cm[i];
      farthestSensor = i;
      #ifdef DEBUG_
      Serial.print("Set farthest to: ");
      Serial.print(farthestReading);
      Serial.print(" for sensor: ");
      Serial.println(farthestSensor);
      #endif
    }
  }
  #ifdef DEBUG_
  Serial.println();
  #endif
  displayed_ = 0;
}

void refreshDisplay() {
  if (!displayed_) {
    #ifdef DEBUG_
    Serial.print("refresh ");
    Serial.println(display_mode_);
    #endif
    switch (display_mode_) {
      case DISPLAY_MODE_HISTOGRAM:
        displayGraph();
        break;;
      case DISPLAY_MODE_QUADRANTS:
        displayQuadrantMap();
        break;;
    }
    displayed_ = 1;
  }
}

void displayQuadrantMap() {
  #ifdef DEBUG_
  Serial.print("closest, farthest: ");
  Serial.print(closestReading);
  Serial.print(",");
  Serial.println(farthestReading);
  #endif
  matrix.clear();      // clear display
  mapQuadrantForSensors(0, 7);
  mapQuadrantForSensors(2, 1);
  mapQuadrantForSensors(4, 3);
  mapQuadrantForSensors(6, 5);
  matrix.writeDisplay();  // write the changes we just made to the display
}

void mapQuadrantForSensors(int CCSensor, int counterCCSensor) {
  int color = LED_YELLOW;
  int x = 0, y = 0;

  int scaledCC = scaled(cm[CCSensor], MIN_DISTANCE, MAX_DISTANCE, 0, MAP_DISPLAY_TIERS+1);
  int scaledCounter = scaled(cm[counterCCSensor], MIN_DISTANCE, MAX_DISTANCE, 0, MAP_DISPLAY_TIERS+1);
  switch (CCSensor) {
    case 0:
      y = MAP_DISPLAY_TIERS + 1 + scaledCC;
      x = MAP_DISPLAY_TIERS - scaledCounter;
      color = mapColor(scaledCC, scaledCounter);
      matrix.drawPixel(x, y, color);
      break;
    case 2:
      x = MAP_DISPLAY_TIERS + 1 + scaledCounter;
      y = MAP_DISPLAY_TIERS + 1 + scaledCC;
      color = mapColor(scaledCC, scaledCounter);
      matrix.drawPixel(x, y, color);
      break;
    case 4:
      x = MAP_DISPLAY_TIERS + 1 + scaledCounter;
      y = MAP_DISPLAY_TIERS - scaledCC;
      color = mapColor(scaledCC, scaledCounter);
      matrix.drawPixel(x, y, color);
      break;
    case 6:
      x = MAP_DISPLAY_TIERS - scaledCC;
      y = MAP_DISPLAY_TIERS - scaledCounter;
      color = mapColor(scaledCC, scaledCounter);
      matrix.drawPixel(x, y, color);
      break;
  }
}

int mapColor(int distance1, int distance2) {
  int color = LED_YELLOW;
  if (max(distance1, distance2) == MAP_DISPLAY_TIERS) {
    color = LED_RED;
  }
  if (min(distance1, distance2) == 0) {
    color = LED_GREEN;
  }
  return color;
}

void displayGraph() {
  int color;

  #ifdef DEBUG_
  Serial.print("Closest reading from: ");
  Serial.print(closestSensor);
  Serial.print("was: ");
  Serial.println(closestReading);
  #endif
  matrix.clear();      // clear display
  for (int sensor = 0; sensor < SONAR_NUM; sensor++) {
    int shift = (signed int) cm[sensor] - (signed int) prev_cm[sensor];
    if (shift >= SHIFT_OUT_THRESHOLD) color = LED_RED;
    else if (shift <= SHIFT_IN_THRESHOLD) color = LED_GREEN;
    else color = LED_YELLOW;
    matrix.drawPixel(sensor, scaled(cm[sensor], MIN_DISTANCE, MAX_DISTANCE, 0, 8), color);
  }
  matrix.writeDisplay();  // write the changes we just made to the display
}

int scaled(int num, int min, int max, int target_min, int target_max) {
  return ((float)(num - min) / (max - min)) * ((target_max - target_min)  + target_min);
}

void buzz() {
  buzz_until_ = millis() + BUZZ_DUR;
  #ifdef DEBUG_
  Serial.print("buzz until");
  Serial.println(buzz_until_);
  #endif
}

void shine() {
  shine_until_ = millis() + SHINE_DUR;
}

void shineLed() {
  if (shine_until_ > 0) {
    if (shine_until_ < millis()) {
      stopShining();
    } else if (!shining_) {
      digitalWrite(LED_, HIGH);
      shining_ = 1;
    }
  }
}


void playBuzz() {
  if (buzzing_) {
    if (buzz_until_ < millis()) {
      #ifdef DEBUG_
      Serial.println("buzz expired");
      #endif
      stopBuzzing();
    }
  } else if (buzz_until_ > 0) {
    #ifdef DEBUG_
    Serial.println("start buzzing");
    #endif
    tone(BUZZER_, BUZZ_TONE);
    buzzing_ = 1;
  }
}

void readRemote() {
  if (irrecv.decode(&results)) {
    remote_cmd_ = REMOTE_CMD_1;
    irrecv.resume(); // Receive the next value
  }
}

/*
When any command is received, toggle the display mode.
*/
void handleCmd() {
  if (remote_cmd_ != REMOTE_CMD_NONE) {
    // we've received a command
    shine();
    buzz();
    #ifdef DEBUG_
    Serial.println("Processed remote cmd");
    #endif
    switch (display_mode_) {
      case DISPLAY_MODE_HISTOGRAM:
        display_mode_ = DISPLAY_MODE_QUADRANTS;
        break;;
      case DISPLAY_MODE_QUADRANTS:
        display_mode_ = DISPLAY_MODE_HISTOGRAM;
        break;;
    }
  remote_cmd_ = REMOTE_CMD_NONE;  // Reset our command tracking
  }
}

void loop() {
  refreshSensors();
  readRemote();
  refreshDisplay();
  playBuzz();
  shineLed();
  handleCmd();
}

