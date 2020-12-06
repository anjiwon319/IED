#include <Servo.h>

// Arduino pin assignment
#define PIN_IR A0
#define PIN_LED 9
#define PIN_SERVO 10

// configurable parameters
int a = 69; // unit: mm
int b = 298; // unit: mm

#define _DIST_MIN 100 // minimum distance to be measured (unit: mm)
#define _DIST_MAX 450 // maximum distance to be measured (unit: mm)
#define _DIST_TARGET 255 //target distance to be meaured (unit: mm)

#define _DUTY_MIN 1106 // servo full clockwise position (0 degree)
#define _DUTY_NEU 1476 // servo neutral position (90 degree)
#define _DUTY_MAX 2106 // servo full counterclockwise position (180 degree)

// Distance sensor
#define _DIST_ALPHA 0.3 // ema필터의 alpha 값을 설정

// Servo speed control
#define _SERVO_ANGLE 30 
#define _SERVO_SPEED 80 

// Event periods
#define _INTERVAL_DIST 20  // 거리측정주기 (ms)
#define _INTERVAL_SERVO 20 // 서보제어주기 (ms)
#define _INTERVAL_SERIAL 100 // Serial제어주기 (ms)

// PID parameters
#define _KP 1.5 // 비례이득

// Distance sensor
float dist_target; // location to send the ball
float dist_min, dist_max, dist_raw, dist_ema; 

// Servo instance
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
  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_NEU);
  pinMode(PIN_LED,OUTPUT);

// initialize global variables
  dist_min = _DIST_MIN; 
  dist_max = _DIST_MAX; // raw distance output from USS (unit: mm)
  dist_target = _DIST_TARGET;
  duty_curr = 1476;

// initialize serial port
  Serial.begin(57600);
  
// convert angle speed into duty change per interval
  duty_chg_per_interval = (float)(_DUTY_MAX - _DUTY_MIN) * _SERVO_SPEED / 180 * _INTERVAL_SERVO / 1000.0;
    
// initialize event variables
  last_sampling_time_dist = last_sampling_time_servo = last_sampling_time_serial = 0;
  event_dist = event_servo = event_serial = false;

}

void loop() {
  unsigned long time_curr = millis();
  if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
        //last_sampling_time_dist += _INTERVAL_DIST;
        event_dist = true;
  }
  if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO) {
        //last_sampling_time_servo += _INTERVAL_SERVO;
        event_servo = true;
  }
  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
        //last_sampling_time_serial += _INTERVAL_SERIAL;
        event_serial = true;
  }
  if(event_dist) {
    event_dist = false;
  // get a distance reading from the distance sensor
    dist_raw = ir_distance_filtered();

  // PID control logic
    error_curr = dist_target - dist_raw; // 현재 읽어들인 데이터와 기준 값의 차이
    pterm = _KP * error_curr; // p게인 값인 kp와 error 값의 곱
    control = pterm;

  // duty_target = f(duty_neutral, control)
    duty_target = _DUTY_NEU + control;

  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    if (duty_target < _DUTY_MIN) {
      duty_target = _DUTY_MIN;  // duty_target < _DUTY_MIN 일 때 duty_target 를 _DUTY_MIN 로 고정
    } 
    else if (duty_target > _DUTY_MAX) {
       duty_target = _DUTY_MAX; // duty_target > _DUTY_MAX 일 때 duty_target 를 _DUTY_MAX 로 고정
    }
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
    myservo.writeMicroseconds(duty_curr);   //[3166]위에서 바뀐 현재위치 값을 갱신
  }
  
   if(event_serial) {
    event_serial = false;
    Serial.print("dist_ir:");
    Serial.print(dist_raw);
    Serial.print(",pterm:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",duty_target:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");

    // update last Serial sampling time
    last_sampling_time_serial += _INTERVAL_SERIAL;
  }
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;  // 변환 식 보정 필요
  float dist_cali = 100 + 300.0 / (b - a) * (val - a);
  return dist_cali;
}

float ir_distance_filtered(void){ // return value unit: mm
  static float val = dist_target;
  static float dist_ema  = 0; 
  float raw = ir_distance();
  if (raw >= _DIST_MIN && raw <= _DIST_MAX){
    val = raw;
    dist_ema =  _DIST_ALPHA * raw + (1.0 - _DIST_ALPHA) * dist_ema;
    return dist_ema;
  }
}
