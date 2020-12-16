#include <Servo.h>

// Arduino pin assignment
#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_IR A0

// Framework setting
#define _DIST_TARGET 255
#define _DIST_MIN 100
#define _DIST_MAX 430

// Distance sensor
#define _DIST_ALPHA 0.1

// Servo range
#define _DUTY_MIN 1020 
#define _DUTY_NEU 1520
#define _DUTY_MAX 2220 

// Servo speed control
#define _SERVO_ANGLE 30 
#define _SERVO_SPEED 300

// Event periods
#define _INTERVAL_DIST 20
#define _INTERVAL_SERVO 20
#define _INTERVAL_SERIAL 100 

// PID parameters
#define _KP 1.7 // 비례이득
#define _KD 82.0 // 미분이득
#define _KI 0.01 // 적분이득

//unit: mm
#define _CALI_MIN 69
#define _CALI_MAX 362

// global variables
const float coE[] = {-0.0000061, 0.0035824, 0.5338517, 56.9175414};

// Servo instance
Servo myservo;

// Distance sensor
float dist_target = _DIST_TARGET; // location to send the ball
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
  myservo.attach(PIN_SERVO);
  pinMode(PIN_LED, OUTPUT);
  myservo.writeMicroseconds(_DUTY_NEU);

// initialize global variables
  last_sampling_time_dist = 0;
  last_sampling_time_servo = 0;
  last_sampling_time_serial = 0;

  event_dist = false;
  event_servo = false;
  event_serial = false;

  pterm = 0;
  dterm = 0;
  iterm = 0;
  
// initialize serial port
  Serial.begin(57600);

// convert angle speed into duty change per interval.
  duty_chg_per_interval = (float)(_DUTY_MAX - _DUTY_MIN) * _SERVO_SPEED / 180 * _INTERVAL_SERVO / 1000;
}
  
void loop() {
/////////////////////
// Event generator //
/////////////////////
  //거리 측정 주기가 되었는지 검사 
  if (millis() >= last_sampling_time_dist + _INTERVAL_DIST)
      event_dist = true;
    
  //서보 제어 주기가 되었는지 검사 
  if (millis() >= last_sampling_time_servo + _INTERVAL_SERVO)
      event_servo= true;
    
  //Serial 제어 주기가 되었는지 검사 
  if (millis() >= last_sampling_time_serial + _INTERVAL_SERIAL)
      event_serial= true;

////////////////////
// Event handlers //
////////////////////

  if(event_dist) {
    event_dist = false;
    
  // calibrate distance reading from the IR sensor
    float x = ir_distance_filtered();
    dist_raw = coE[0] * pow(x, 3) + coE[1] * pow(x, 2) + coE[2] * x + coE[3];

  // PID control logic
    error_curr = dist_target - dist_raw;
    pterm = _KP * error_curr;
    dterm = _KD * (error_curr - error_prev);
    iterm += _KI * error_curr;
    control = pterm + dterm + iterm;

  // duty_target = f(duty_neutral, control)
    duty_target = _DUTY_NEU + control;

  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    if (duty_target < _DUTY_MIN) duty_target = _DUTY_MIN;
    else if (duty_target > _DUTY_MAX) duty_target = _DUTY_MAX;

    // update error_prev
    error_prev = error_curr;

    last_sampling_time_dist += _INTERVAL_DIST;
  }
  
  if(event_servo) {
    event_servo = false;

    // adjust duty_curr toward duty_target by duty_chg_per_interval
    if(duty_target > duty_curr) {
      duty_curr += duty_chg_per_interval;
      if(duty_curr > duty_target) duty_curr = duty_target;
    }
    else {
      duty_curr -= duty_chg_per_interval;
      if(duty_curr < duty_target) duty_curr = duty_target;
    }

    // update servo position
    myservo.writeMicroseconds(duty_curr);

    last_sampling_time_servo += _INTERVAL_SERVO;
  }
  
  if(event_serial) {
    event_serial = false;
    Serial.print("IR:");
    Serial.print(dist_raw);
    Serial.print(",T:");
    Serial.print(dist_target);
    Serial.print(",P:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",D:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",I:");
    Serial.print(map(iterm,-1000,1000,510,610));
    Serial.print(",DTT:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",DTC:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",-G:245,+G:265,m:0,M:800");

    last_sampling_time_serial += _INTERVAL_SERIAL;
  }
}

float ir_distance(void){ // return value unit: mm
  float val, dist_cali;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  dist_cali = 100 + 300.0 / (_CALI_MAX - _CALI_MIN) * (val - _CALI_MIN);
  return dist_cali;
}

float ir_distance_filtered(void){ // return value unit: mm
  static float VAL = dist_target;
  static float dist_ema  = 0;
  float raw = ir_distance();
  if (raw >= _DIST_MIN && raw <= _DIST_MAX)
    VAL = raw;
  dist_ema =  _DIST_ALPHA * raw + (1 - _DIST_ALPHA) * dist_ema;
  return dist_ema;
}
