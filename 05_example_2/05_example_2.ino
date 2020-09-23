#define PIN_LED 7
unsigned int count;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200); // Initialize serial port
  while(!Serial){
    ; //wait for serial port to connect
  }
  count = 0;
}

void loop() {
  digitalWrite(PIN_LED, 0);
  delay(1000);
  while(count<5){
    digitalWrite(PIN_LED, 1);
    delay(100);
    digitalWrite(PIN_LED, 0);
    delay(100);
    count +=1;
   digitalWrite(PIN_LED, 1);
  }
  while(1){
    ;
  }
}
