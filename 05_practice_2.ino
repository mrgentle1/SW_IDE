#define PIN_LED 7
unsigned int toggle;

void setup() {
  // put your setup code here, to run once:
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200);
  toggle = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(PIN_LED, toggle);
  delay(1000);
  for (int i=0;i<11;i++){
    toggle = !toggle;
    digitalWrite(PIN_LED, toggle);
    delay(100);
  }
  while(1){}
}
