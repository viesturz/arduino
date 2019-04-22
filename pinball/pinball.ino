#include <Servo.h>

Servo left;
Servo right;

void setup() {
  left.attach(9);
  right.attach(10);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
}

void loop() {
  int left_pressed = digitalRead(3);
  int right_pressed = digitalRead(2);
  if (left_pressed == HIGH) {
    left.write(90);
  } else {
    left.write(20);    
  } 
  if (right_pressed == HIGH) {
    right.write(70);
  } else {
    right.write(140);
  }
  delay(5);
}
