#include <Arduino.h>
#include <ServoEasing.h>
// put function declarations here:
int myFunction(int, int);
ServoEasing myServo;
void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
  myServo.attach(9); // Attach the servo to pin 9
}

void loop() {
  // put your main code here, to run repeatedly:}
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}