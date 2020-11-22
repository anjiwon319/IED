#include <Servo.h>

// Arduino pin assignment
#define PIN_IR A0
#define PIN_LED 9
#define PIN_SERVO 10

// configurable parameters
int a = 69; // unit: mm
int b = 285; // unit: mm
#define INTERVAL 10 // sampling interval (unit: ms)
#define _DIST_MIN 100 // minimum distance to be measured (unit: mm)
#define _DIST_MAX 450 // maximum distance to be measured (unit: mm)

#define _DUTY_MIN 1370 // servo full clockwise position (수평일 때보다 200mm위 )
#define _DUTY_NEU 1470 // servo neutral position (수평일 때)
#define _DUTY_MAX 1570 // servo full counterclockwise position (수평일 때보다 200mm 아래)

// global variables
float timeout; // unit: us
float dist_min, dist_max, raw_dist, ema_dist, alpha, duty_val, dist_cali; // unit: mm
unsigned long last_sampling_time; // unit: ms
Servo myservo;

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);

  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_NEU);

// initialize USS related variables
  dist_min = _DIST_MIN; 
  dist_max = _DIST_MAX;
  timeout = (INTERVAL / 2) * 1000.0; // precalculate pulseIn() timeout value. (unit: us)
  raw_dist = ema_dist = 0.0; // raw distance output from USS (unit: mm)
  alpha = 0.1;
  
// initialize serial port
  Serial.begin(57600);
}

void loop() {
   if(millis() < last_sampling_time + INTERVAL) return;

// get a distance reading from the USS
  raw_dist = ir_distance();
  ema_dist = alpha*raw_dist + (1-alpha)*ema_dist;
  dist_cali = 100 + 300.0 / (b - a) * (ema_dist - a);

// adjust servo position according to the USS read value

  if (dist_cali > 255){
    duty_val = _DUTY_MIN;
    myservo.writeMicroseconds(duty_val);
  }
  else{
    duty_val = _DUTY_MAX;
    myservo.writeMicroseconds(duty_val);
  }
   
// update last sampling time
  last_sampling_time += INTERVAL;
   
  Serial.print("min:0,max:500,dist:");
  Serial.print(raw_dist);
  Serial.print(",ema:");
  Serial.print(ema_dist);
  Serial.print(",dist_cali:");
  Serial.println(dist_cali);
  
  if(raw_dist > 156 && raw_dist <224) digitalWrite(PIN_LED, 0);
  else digitalWrite(PIN_LED, 255);
  delay(20);

}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}
