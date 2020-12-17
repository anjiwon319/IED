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
#define _DIST_ALPHA 0.15
#define _DELAY_MICROS 1500

// Servo range
#define _DUTY_MIN 1020 
#define _DUTY_NEU 1520
#define _DUTY_MAX 2020 

// Servo speed control
#define _SERVO_ANGLE 30 
#define _SERVO_SPEED 300

// Event periods
#define _INTERVAL_DIST 20
#define _INTERVAL_SERVO 20
#define _INTERVAL_SERIAL 100 

// PID parameters
#define _KP 2.7187 // 비례이득
#define _KD 78.0 // 미분이득
#define _KI 0.005 // 적분이득

//unit: mm
#define _CALI_MIN 69
#define _CALI_MAX 356

// global variables
const float coE[] = {-0.0000104, 0.0057987, 0.2360595, 69.2162580};

// Servo instance
Servo myservo;

// Distance sensor
float dist_target = _DIST_TARGET; // location to send the ball
float dist_raw, dist_filtered;
float dist_ema = 0;
float samples_num = 3;

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist, event_servo, event_serial;

// Servo speed control
int duty_chg_per_interval; 
int duty_target, duty_curr;

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;

void setup() {
// initialize serial port
  Serial.begin(57600);
  
// initialize GPIO pins for LED and attach servo 
  myservo.attach(PIN_SERVO);
  pinMode(PIN_LED, OUTPUT);
  duty_curr = _DUTY_NEU;
  myservo.writeMicroseconds(duty_curr);

// initialize global variables
  last_sampling_time_dist = 0;
  last_sampling_time_servo = 0;
  last_sampling_time_serial = 0;

  event_dist = false;
  event_servo = false;
  event_serial = false;

  pterm = iterm = dterm = 0;

// convert angle speed into duty change per interval.
  duty_chg_per_interval = (float)(_DUTY_MAX - _DUTY_MIN) * _SERVO_SPEED / 180 * _INTERVAL_SERVO / 1000;
}
  
void loop() {
  //거리 측정 주기가 되었는지 검사 
  if (millis() >= last_sampling_time_dist + _INTERVAL_DIST)
      event_dist = true;
    
  //서보 제어 주기가 되었는지 검사 
  if (millis() >= last_sampling_time_servo + _INTERVAL_SERVO)
      event_servo= true;
    
  //Serial 제어 주기가 되었는지 검사 
  if (millis() >= last_sampling_time_serial + _INTERVAL_SERIAL)
      event_serial= true;

  if(event_dist) {
    event_dist = false;
    
  // calibrate distance reading from the IR sensor
    dist_raw = ir_distance_filtered();
    dist_filtered = 100 + 300.0 / (_CALI_MAX - _CALI_MIN) * (dist_raw - _CALI_MIN);

  // PID control logic
    error_curr = dist_target - dist_filtered;
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
    Serial.print(dist_filtered);
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
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

float under_noise_filter(void){ // 아래로 떨어지는 형태의 스파이크를 제거해주는 필터
  int currReading;
  int largestReading = 0;
  for (int i = 0; i < samples_num; i++) {
    currReading = ir_distance();
    if (currReading > largestReading) { largestReading = currReading; }
    // Delay a short time before taking another reading
    delayMicroseconds(_DELAY_MICROS);
  }
  return largestReading;
}

float ir_distance_filtered(void){ // return value unit: mm
  int currReading;
  int lowestReading = 1024;
  for (int i = 0; i < samples_num; i++) {
    currReading = under_noise_filter();
    if (currReading < lowestReading) { lowestReading = currReading; }
  }
  dist_ema =  _DIST_ALPHA * lowestReading + (1 - _DIST_ALPHA) * dist_ema;
  return dist_ema;
}
