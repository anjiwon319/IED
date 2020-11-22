// LED breathing using analog input (PWM)
#define PIN_LED 9

void setup()
{
  pinMode(PIN_LED, OUTPUT); // analog output
}

void loop()
{
  for (int val = 0; val<= 255; val += 5){
    analogWrite(PIN_LED, val);
    delay(25);
  }
  
   for (int val = 255; val>= 0; val -= 5){
    analogWrite(PIN_LED, val);
    delay(25);
  }

}
