#include <Servo.h>

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9    // [xxxx] LED 핀은 아두이노 9번 핀에 연결
#define PIN_SERVO 10
#define PIN_IR A0

// Framework setting
#define _DIST_TARGET 255
#define _DIST_MIN 100
#define _DIST_MAX 410

// Distance sensor
#define _DIST_ALPHA 0.0

// Servo range
#define _DUTY_MIN 1000 
#define _DUTY_NEU 1450 
#define _DUTY_MAX 2000 

// Servo speed control
#define _SERVO_ANGLE 30 
#define _SERVO_SPEED 30 

// Event periods
#define _INTERVAL_DIST 20 
#define _INTERVAL_SERVO 20 
#define _INTERVAL_SERIAL 100 

// PID parameters
#define _KP 0.0 

//////////////////////
// global variables //
//////////////////////

// Servo instance
Servo myservo;

// Distance sensor
float dist_target; // location to send the ball
float dist_raw, dist_ema;

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist, event_servo, event_serial;

// Servo speed control
int duty_chg_per_interval; 
int duty_target, duty_curr;

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;

void setup() {
// initialize GPIO pins for LED and attach servo 

// initialize global variables

// move servo to neutral position

// initialize serial port

// convert angle speed into duty change per interval.
  duty_chg_per_interval = ??;
}
  
void loop() {
/////////////////////
// Event generator //
/////////////////////


////////////////////
// Event handlers //
////////////////////

  if(event_dist) {
      = false;
  // get a distance reading from the distance sensor
      ?? = ir_distance_filtered();

  // PID control logic
    error_curr = 
    pterm = 
    control = 

  // duty_target = f(duty_neutral, control)
    duty_target = 

  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]

  }
  
  if(event_servo) {

    // adjust duty_curr toward duty_target by duty_chg_per_interval

    // update servo position

  }
  
  if(event_serial) {
    event_serial = ??;
    Serial.print("dist_ir:");
    Serial.print(dist_raw);
    Serial.print(",pterm:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",duty_target:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");
  }
}

float ir_distance(void){ // return value unit: mm

}

float ir_distance_filtered(void){ // return value unit: mm
  return ir_distance(); // for now, just use ir_distance() without noise filter.
}
