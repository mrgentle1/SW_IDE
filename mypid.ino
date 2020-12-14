#include <Servo.h>

#define PIN_SERVO 10
#define PIN_IR A0

#define _DIST_TARGET 255    // 목표위치
#define _DIST_MIN 100   // 최소거리
#define _DIST_MAX 487   // 최대거리

#define A 79  // 센서 최소
#define B 500  // 센서 최대

#define _ITERM_MAX 60
#define _DIST_ALPHA 0.1

#define DELAY_MICROS  1550 // 필터에 넣을 샘플값을 측정하는 딜레이(고정값!)
#define EMA_ALPHA 0.35     // EMA 필터 값을 결정하는 ALPHA 값. 작성자가 생각하는 최적값임.

#define _DUTY_MIN 1400   // 서보 최소각도
#define _DUTY_NEU 1600   // 서보 수평
#define _DUTY_MAX 1800   // 서보 최대각도

#define _SERVO_ANGLE 30
#define _SERVO_SPEED 250   // 서보 속도
 
#define _INTERVAL_DIST 20
#define _INTERVAL_SERVO 20
#define _INTERVAL_SERIAL 200

#define _KP 0.1
#define _KD 7.0
#define _KI 0.5

/*
   pterm = _KP * error_curr;
   dterm = _KD * (error_curr - error_prev);
   iterm += _KI * error_curr;
 */

Servo myservo;

float dist_raw, dist_ema;
unsigned long last_sampling_time_dist, last_sampling_time_servo,
last_sampling_time_serial;
bool event_dist, event_servo, event_serial;

int duty_chg_per_interval;
int duty_target, duty_curr;

float error_curr, error_prev, control, pterm, dterm, iterm;
float prev_dist;

int dist_target = 255;

float ir_distance(void){ // return value unit: mm
    float val; // [3055] 변수 val 선언
    float volt = float(analogRead(PIN_IR));
    val = ((6762.0/(volt - 9.0)) - 4.0) * 10.0;
    return val;
}

float ir_distance_filtered(void){ // return value unit: mm
  return ir_distance(); // for now, just use ir_distance() without noise filter.
}



float ema_dist=0;            // EMA 필터에 사용할 변수
float filtered_dist;       // 최종 측정된 거리값을 넣을 변수. loop()안에 filtered_dist = filtered_ir_distance(); 형태로 사용하면 됨.
float samples_num = 3;     // 스파이크 제거를 위한 부분필터에 샘플을 몇개 측정할 것인지. 3개로 충분함! 가능하면 수정하지 말 것.

// 기존에 제공된 ir_distance 함수. 변경사항 없음.
// ================
float under_noise_filter(void){ // 아래로 떨어지는 형태의 스파이크를 제거해주는 필터
  int currReading;
  int largestReading = 0;
  for (int i = 0; i < samples_num; i++) {
    currReading = ir_distance();
    if (currReading > largestReading) { largestReading = currReading; }
    // Delay a short time before taking another reading
    delayMicroseconds(DELAY_MICROS);
  }
  return largestReading;
}

float filtered_ir_distance(void){ // 아래로 떨어지는 형태의 스파이크를 제거 후, 위로 치솟는 스파이크를 제거하고 EMA필터를 적용함.
  // under_noise_filter를 통과한 값을 upper_nosie_filter에 넣어 최종 값이 나옴.
  int currReading;
  int lowestReading = 1024;
  for (int i = 0; i < samples_num; i++) {
    currReading = under_noise_filter();
    if (currReading < lowestReading) { lowestReading = currReading; }
  }
  // eam 필터 추가
  ema_dist = EMA_ALPHA*lowestReading + (1-EMA_ALPHA)*ema_dist;
  return ema_dist;
}


void setup() {
  // put your setup code here, to run once:
  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(_DUTY_NEU);
  pterm = iterm = dterm = 0;
//  while(1){}
  Serial.begin(57600);
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN)/(float)(_SERVO_ANGLE) * (_SERVO_SPEED /1000.0)*_INTERVAL_SERVO;
}

void loop() {
  // put your main code here, to run repeatedly:

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

    // Event handlers

     if(event_dist) {
        event_dist = false;
        dist_raw = filtered_ir_distance();
        dist_raw = 100 + 300.0 / (B - A) * (dist_raw - A);
//        if(dist_raw>250) dist_raw += 90;
        if (dist_raw > _DIST_MAX) dist_raw = _DIST_MAX;
 
        error_curr = _DIST_TARGET - dist_raw;
        pterm = _KP * error_curr;
        dterm = _KD * (error_curr - error_prev);
        iterm += _KI * error_curr;

        if(abs(iterm) > _ITERM_MAX) iterm = iterm/2;
        if(iterm > _ITERM_MAX) iterm = _ITERM_MAX;
        if(iterm < -_ITERM_MAX) iterm = -_ITERM_MAX;

        control = pterm + dterm + iterm;
        error_prev = error_curr;

        duty_target = _DUTY_NEU *(control/300+1);
        
        if(duty_target < _DUTY_MIN) duty_target = _DUTY_MIN;
        if(duty_target > _DUTY_MAX) duty_target = _DUTY_MAX;
    }
  
    if(event_servo) {
        event_servo = false; // [2635] 업데이트 대기
        if(duty_target > duty_curr) {
            duty_curr += duty_chg_per_interval;
            if(duty_curr > duty_target) duty_curr = duty_target;
        }
        else {
            duty_curr -= duty_chg_per_interval;
            if(duty_curr < duty_target) duty_curr = duty_target;
        }
        myservo.writeMicroseconds((int)duty_curr);
        event_servo = false;
    }

      if(event_serial) {
        event_serial = false;
        Serial.print("IR:");
        Serial.print(dist_raw);
        Serial.print(",T:");
        Serial.print(_DIST_TARGET);
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
        event_serial = false;
     }
}
