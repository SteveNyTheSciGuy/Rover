#include <Psx.h>  //for v1.6

Psx psx;
int data;

//Psx.setupPins(2, 3, 4, 5, 10); //setupPins(dataPin, cmndPin, attPin, clockPin, delay)

void setup() {
  // put your setup code here, to run once:
  psx.setupPins(2, 3, 4, 5, 50); //setupPins(dataPin, cmndPin, attPin, clockPin, delay)
  //data = Psx.read();
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  data = psx.read();
  if (data & psxUp) {
    Serial.print(data);
  }
}
