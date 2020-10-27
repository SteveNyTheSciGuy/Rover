#include <Servo.h> //servo driver
Servo s0;


int pos = 0;    // variable to store the servo position
int maxangle=180;
int cnt = 0;

void setup() {
  s0.attach(30);  // attaches the servo on pin 9 to the servo object
}

void loop() {
  while (cnt < 2 ) {
    for (pos = 0; pos <= maxangle; pos += 1) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      s0.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
    }
    for (pos = maxangle; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
      s0.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
    }
    cnt++;
  }
  if (cnt=2) {
    s0.write(maxangle/2);
    delay(15);
    cnt++;
  }
}
