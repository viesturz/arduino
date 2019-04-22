// For use with the relays shield: https://store.arduino.cc/4-relays-shield


int PIN = 4;

// Arduino has no real time clock, so this should be the exact time when to plug in in minutes.
const int WATER_DURATION = 10*1000; // ms
const int START_TIME = 11*60+10;
const int SCHEDULE[3] = {9*60, 12*60+1, 18*60};

unsigned long startMillis;
unsigned long prevMinute = -1;

void setup() {
  pinMode(PIN, OUTPUT);
  startMillis = millis();
  water(100); // Boot up blip to signal this is running.  
}

void loop() {
  delay(1000); // Sleep for a second
  int minute = ((millis() - startMillis) / 1000 / 60 + START_TIME) % (24*60);
  if (prevMinute == minute) return; // Still the same minute, skip.

  prevMinute = minute;
  // Water
  for (int i = 0; i <  sizeof(SCHEDULE) / sizeof(SCHEDULE[0]); i ++ ) {
    if (minute == SCHEDULE[i]) water(WATER_DURATION);
  }
}

void water(int timeMs) {
  digitalWrite(PIN, HIGH);
  delay(timeMs);
  digitalWrite(PIN, LOW);
}
