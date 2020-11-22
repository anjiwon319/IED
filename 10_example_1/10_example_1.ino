#include <Servo.h>
#define PIN_SERVO 10

Servo myservo;

void setup() {
  myservo.attach(PIN_SERVO);
  myservo.write(90); 
  delay(1000);

}

void loop() {
    // add code here.
  myservo.write(130);
  delay(1000);
  myservo.write(90);
  delay(1000);
  myservo.write(50);
  delay(1000);
  myservo.write(90);
  delay(1000);
}
