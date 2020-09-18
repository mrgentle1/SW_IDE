int duty;
float period;
int second = 0;
int ledPin = 9;

void setup() {
  pinMode(9, OUTPUT);
  Serial.begin(9600);
  duty = set_duty(100);
  period = set_period(100);
}

void loop() {
  while(second<10000){
    for (int i = 0; i <= duty; i += 1){
      analogWrite(ledPin, 255-i);
      delayMicroseconds(period);
      int times = period/100 < 1 ? 1 : period/100;
      second += times;
    }
    for (int i = duty; i >= 0; i -= 1){
      analogWrite(ledPin, 255-i);
      delayMicroseconds(period);
      int times = period/100 == 0 ? 1 : period/100;
      second += times;
    }
  }
}

float set_period(int period){
  return period/duty;
}

int set_duty(int duty){
  return 255*(float)duty/100;
}
