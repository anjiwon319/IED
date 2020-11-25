#include <Servo.h>

// Arduino pin assignment
#define PIN_IR A0
#define PIN_LED 9
#define PIN_SERVO 10

// configurable parameters
int a = 69; // unit: mm
int b = 285; // unit: mm
#define _DIST_MIN 100 // minimum distance to be measured (unit: mm)
#define _DIST_MAX 450 // maximum distance to be measured (unit: mm)
#define _DIST_TARGET 255 //target distance to be meaured (unit: mm)

#define _DUTY_MIN 1370 // servo full clockwise position (수평일 때보다 200mm위 )
#define _DUTY_NEU 1470 // servo neutral position (수평일 때)
#define _DUTY_MAX 1570 // servo full counterclockwise position (수평일 때보다 200mm 아래)

// Servo speed control
#define _SERVO_ANGLE 30 
#define _SERVO_SPEED 30 

// Event periods
#define _INTERVAL_DIST 20  // [3074] 거리측정주기 (ms)
#define _INTERVAL_SERVO 20 // [3078] 서보제어주기 (ms)
#define _INTERVAL_SERIAL 100 // [3078] Serial제어주기 (ms)

// global variables
float timeout; // unit: us
float dist_min, dist_max, raw_dist, ema_dist, alpha, duty_val, dist_cali; // unit: mm
Servo myservo;
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; // unit: ms
bool event_dist, event_servo, event_serial;

// Servo speed control
int duty_chg_per_interval; 
int duty_target, duty_curr;

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);

  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_NEU);

// initialize USS related variables
  dist_min = _DIST_MIN; 
  dist_max = _DIST_MAX;
  timeout = (_INTERVAL_DIST / 2) * 1000.0; // precalculate pulseIn() timeout value. (unit: us)
  raw_dist = ema_dist = 0.0; // raw distance output from USS (unit: mm)
  alpha = 0.1;

// initialize serial port
  Serial.begin(57600);
  
// convert angle speed into duty change per interval
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / 180.0) * (_INTERVAL_SERVO / 1000.0);
  
// initialize event variables
  last_sampling_time_dist = last_sampling_time_servo = last_sampling_time_serial = 0;
  event_dist = event_servo = event_serial = false;

}

void loop() {
  unsigned long time_curr = millis();
  if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
        last_sampling_time_dist += _INTERVAL_DIST;
        event_dist = true;
  }
  if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO) {
        last_sampling_time_servo += _INTERVAL_SERVO;
        event_servo = true;
  }
  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
        last_sampling_time_serial += _INTERVAL_SERIAL;
        event_serial = true;
  }
  if(event_dist) {
    event_dist = false;
    // get a distance reading from the USS
    raw_dist = ir_distance();
    ema_dist = alpha*raw_dist + (1-alpha)*ema_dist;
    dist_cali = 100 + 300.0 / (b - a) * (ema_dist - a);
  }

  if(event_servo) {
    event_servo = false;
    // adjust servo position according to the USS read value
    if (dist_cali > _DIST_TARGET){
      duty_val = _DUTY_MIN;
      myservo.writeMicroseconds(duty_val);
    }
    else{
      duty_val = _DUTY_MAX;
      myservo.writeMicroseconds(duty_val);
    }
    // update last servo sampling time
    last_sampling_time_servo += _INTERVAL_SERVO;
   }
  
  if(event_serial) {
    event_serial = false;   
    Serial.print("min:100,max:450,dist:");
    Serial.print(raw_dist);
    Serial.print(",ema:");
    Serial.print(ema_dist);
    Serial.print(",dist_cali:");
    Serial.println(dist_cali);
    // update last Serial sampling time
    last_sampling_time_serial += _INTERVAL_SERIAL;
  }
  
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
