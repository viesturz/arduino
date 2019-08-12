#include <stdio.h>

#define VERBOSE 0
#define LOOP 40

int ENABLE_MOTORS = 1;
int ACCEL_DIV = 10; // 0 to 100 in 2 sec, needs to be less than loop

// speed = -255..255
void writeMotorSpeed(int speed, int revPin, int fwdPin) {  
  if (ENABLE_MOTORS) {
    if (speed > 0) {
      analogWrite(revPin, 0);
      analogWrite(fwdPin, speed);
    }
    else {
      analogWrite(fwdPin, 0);
      analogWrite(revPin, -speed);
    }  
  }
  if (VERBOSE) {
    Serial.print(" Motor: ");
    Serial.print(speed);
  }
}


int sign(int value) { 
 return int((value>0)-(value<0)); 
}


typedef struct {
  int revPin;
  int fwdPin;
  int revSens;
  int fwdSens;
  int maxRevSpeed;
  int maxFwdSpeed;

  // ms to run in reverse
  int atRevLimit;
  int atFwdLimit;
  int speed;  

  void setup() {
   pinMode(fwdPin, OUTPUT);
   pinMode(revPin, OUTPUT);  
   if (fwdSens) pinMode(fwdSens, INPUT);
   if (revSens) pinMode(revSens, INPUT);
   speed = 0;
   atFwdLimit = 0;
   atRevLimit = 0;   
  }

  void update(int direction, int dt) {    
    bool fwdSensHit = fwdSens ? digitalRead(fwdSens) == LOW : 0;
    bool revSensHit = revSens ? digitalRead(revSens) == LOW: 0;
  
    if (fwdSensHit && revSensHit) {
      // No connection to sensors!
      return;
    }
  
    // Set retract amounts if high/low hit.
    if (revSensHit) {
      atRevLimit = 50;
      atFwdLimit = 0;
    }
    if (fwdSensHit) {
      atFwdLimit = 50;
      atRevLimit = 0;
    }
  
    // retract if too low/high
    if (atRevLimit > 1) {
      writeMotorSpeed(maxFwdSpeed, revPin, fwdPin);
      atRevLimit = max(1, atRevLimit-dt);   
      speed = 0;
      return;
    }     
    if (atFwdLimit > 1) {
      writeMotorSpeed(-maxRevSpeed, revPin, fwdPin);
      atFwdLimit = max(1, atFwdLimit-dt);
      speed = 0;
      return;
    }
 
    int accel = dt / ACCEL_DIV;
    if (direction != 0) {
      speed = min(maxFwdSpeed, max(-maxRevSpeed, speed + sign(direction) * accel));
      
      if (direction > 0) {
        atRevLimit = 0;
        if (atFwdLimit) {
          speed = 0;
        }
      } else if (direction < 0) {
        atFwdLimit = 0;
        if (atRevLimit) {
          speed = 0;
        }
      }     
    } else {
      accel = accel * 5;
      if (abs(speed) < accel) {
       speed = 0;
      } else {
       speed = speed - sign(speed) * accel;
      }
    }
    writeMotorSpeed(speed, revPin, fwdPin);
  }  
} Motor;

typedef struct {
  int pin;
  int power;
  unsigned long dayNightDuration;
  unsigned long dayNightCooldown;
  unsigned long sunlightDuration;
  unsigned long sunlightCooldown;

  unsigned long watering; // for how long yet to water
  unsigned long lastWateringTime;
  unsigned long sunlightTimeAccumulated;

  int state; // 0 = off, 1 = on

  void setup() {
   pinMode(pin, OUTPUT);
   analogWrite(pin, 0);
   lastWateringTime = 0;
   watering = 0;
   sunlightTimeAccumulated = 0;   
   state = 0;
  }

  void update(unsigned long dt) {
    if (VERBOSE) {
      char buffer[200];
      sprintf(buffer, " W:%lu Sun: %lu, SunCool:%lu, LastDNW: %lu", watering, sunlightTimeAccumulated, sunlightCooldown, lastWateringTime);
      Serial.print(buffer);
    }      

    if (watering == 0) {
      return;
    } else if (watering == 1) {
        watering = 0;
        stopWatering();
    } else if (watering <= dt) {
      updateWatering(dt);
      watering = 1;
    } else {
      updateWatering(dt);
      watering -= dt;
    }
  }

  void water(unsigned long ms) {
    if (watering == 0) {
      startWatering();
    }
    watering = max(watering, ms);
  }

  // Watering low level logic
  void startWatering() {
    analogWrite(pin, power);
    state = 1;  
  }

  void updateWatering(unsigned long dt) {
  }

  void stopWatering() {
     analogWrite(pin, 0);
     state = 0;
  }

  void addSunlight(unsigned long dt) {
    sunlightTimeAccumulated += dt;
    if (sunlightTimeAccumulated >= sunlightCooldown) {
      water(sunlightDuration);
      sunlightTimeAccumulated -= sunlightCooldown;
    }
  }

  void waterDayNight() {
    unsigned long t = millis();
    if ((lastWateringTime == 0) || (t - lastWateringTime > dayNightCooldown)) {
      lastWateringTime = t;
      water(dayNightDuration);
    }
  }

} Watering;


Motor pan = {
  revPin: 5,
  fwdPin: 6,
  revSens: 0,
  fwdSens: 0,
  maxRevSpeed: 60,
  maxFwdSpeed: 60
};

Motor tilt = {
  revPin: 10,
  fwdPin: 11,
  revSens: 3,
  fwdSens: 2,
  maxRevSpeed: 66,
  maxFwdSpeed: 72
};

Watering water = {
  pin: 9,
  power: 120,
  dayNightDuration: 1000UL * 40,
  dayNightCooldown: 1000UL * 60 * 60 * 2,
  sunlightDuration: 1000UL * 15,
  sunlightCooldown: 1000UL * 60 * 60
};

int TRACKER_LED_TL = A3;
int TRACKER_LED_TR = A1; //
int TRACKER_LED_BL = A0; //
int TRACKER_LED_BR = A2;

// LED calibration, maps to 0..1000
int TRACKER_LED_MIN[] = {0,8,0,8};
int TRACKER_LED_MAX[] = {1000,1000,1000,1000};

int TRACKER_SLEEP = 65;
int TRACKER_WAKE = 70;
int TRACKER_SUNLIGHT = 90;
int TRACKER_WAKE_MIN_DIFF = 7;
int TRACKER_START_DIFF = 7;
int TRACKER_STOP_DIFF = 3;

int SENSOR_AVG = 4;
unsigned long NIGHT_WAKE_TIMEOUT = 1000UL * 60 * 60 ; // 1h

int MODE_NIGHT = 1;
int MODE_WAKING = 2;
int MODE_DAY = 3;


// Updated by updateTracker. 
int panSpeed = 0;
int tiltSpeed = 0;
int mode = MODE_WAKING; 
unsigned long timeSinceNight = 0;

int prevTL = 0;
int prevTR = 0;
int prevBL = 0;
int prevBR = 0;
int iter = 0;

void setupTracker() {
  pinMode(TRACKER_LED_TL, INPUT);
  pinMode(TRACKER_LED_BL, INPUT);
  pinMode(TRACKER_LED_TR, INPUT);
  pinMode(TRACKER_LED_BR, INPUT);
}

void updateTracker(unsigned long dt) { 
  bool wasMoving = panSpeed != 0 || tiltSpeed != 0;
  panSpeed = 0;
  tiltSpeed = 0;
  int tl = map(analogRead(TRACKER_LED_TL), TRACKER_LED_MIN[0], TRACKER_LED_MAX[0], 0, 1000);
  int tr = map(analogRead(TRACKER_LED_TR), TRACKER_LED_MIN[1], TRACKER_LED_MAX[1], 0, 1000);
  int bl = map(analogRead(TRACKER_LED_BL), TRACKER_LED_MIN[2], TRACKER_LED_MAX[2], 0, 1000);
  int br = map(analogRead(TRACKER_LED_BR), TRACKER_LED_MIN[3], TRACKER_LED_MAX[3], 0, 1000);

  tl = (prevTL * (SENSOR_AVG - 1) + tl) / SENSOR_AVG;
  tr = (prevTR * (SENSOR_AVG - 1) + tr) / SENSOR_AVG;
  bl = (prevBL * (SENSOR_AVG - 1) + bl) / SENSOR_AVG;
  br = (prevBR * (SENSOR_AVG - 1) + br) / SENSOR_AVG;
  prevTL = tl;
  prevTR = tr;
  prevBL = bl;
  prevBR = br;

  if (VERBOSE) {
    char buffer[100];
    sprintf(buffer, "M:%i TL: %i, TR: %i, BL: %i, BR: %i", mode, tl, tr, bl, br);
    Serial.print(buffer);
  }  

  iter ++;  
  if (iter < SENSOR_AVG * 2) {
    return;
  }

  int high = max(max(tl,tr), max(bl,br));
  int low = min(min(tl,tr), min(bl,br));
  int leftHigh = max(tl,bl);
  int leftLow = min(tl,bl);
  int rightHigh = max(tr,br);
  int rightLow = min(tr,br);
  int topHigh = max(tl,tr);
  int topLow = min(tl,tr);
  int bottomHigh = max(br,bl);
  int bottomLow = min(br,bl);
  int lrDiff = (tr + br -tl-bl)/2;
  int tbDiff = (tr + tl -br-bl)/2;

  if (high > TRACKER_SUNLIGHT) {
    water.addSunlight(dt);
  }

  if (mode == MODE_NIGHT) {
    timeSinceNight += dt;
    if (high >= TRACKER_WAKE && timeSinceNight > NIGHT_WAKE_TIMEOUT) {
      mode = MODE_WAKING; 
      water.waterDayNight();
      return;
    } else {
      // Tilt up and sleep
      tiltSpeed = 1;
      return;
    }
  } else if (mode == MODE_WAKING) {
    // Tilt up until horizontal    
    if (tilt.atFwdLimit == 0) {
      tiltSpeed = 1;
      return;
    }
    // Pan until good aim.
    if (abs(lrDiff) > TRACKER_WAKE_MIN_DIFF) {
      panSpeed = lrDiff;
      return;
    }
    // Back to day mode
    mode = MODE_DAY;
  } else if (mode == MODE_DAY) {
    if (high < TRACKER_SLEEP) { 
      mode = MODE_NIGHT;
      water.waterDayNight();
      timeSinceNight = 0;
      return;
    }

    // Track the sun
    int diff = max(abs(lrDiff), abs(tbDiff));
    if ((wasMoving && diff >= TRACKER_STOP_DIFF) || (diff >= TRACKER_START_DIFF)) {
      if (abs(lrDiff) > TRACKER_STOP_DIFF) panSpeed = lrDiff;
      if (abs(tbDiff) > TRACKER_STOP_DIFF) tiltSpeed = tbDiff;
    }
    
  } else {
    // Should never happen, but still.
    mode = MODE_WAKING;
  }
}

unsigned long prevTime = 0;

void setup() {   
   water.setup();
   setupTracker(); 
   pan.setup();
   tilt.setup();
   if (VERBOSE) {
     Serial.begin(115200);
     Serial.print("Setup done verbose=");
     Serial.print(VERBOSE);
     Serial.print("\n");
   }
   water.water(1000);
   prevTime = millis();
}

void loop() { 
  unsigned long time = millis();
  unsigned long dt = time - prevTime;

  if (VERBOSE) {
    unsigned long time = millis() - time; 
    Serial.print(" Time ");
    Serial.print(time);
    Serial.print("\n");
  }
  if (dt < LOOP) {
    delay(LOOP-dt);
    dt = LOOP;
    time = time -dt + LOOP;    
  }
  prevTime = time;

  updateTracker(dt);
  water.update(dt);
  pan.update(panSpeed, dt);
  tilt.update(tiltSpeed, dt);

  if (VERBOSE) {
    char buffer[200];
    sprintf(buffer, " Pan: %i, Tilt: %i, Water:%i", panSpeed, tiltSpeed, water.watering);
    Serial.print(buffer);
  }
}
