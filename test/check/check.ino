#define INTERVAL 100
unsigned long last_sampling_time;

void setup() {
  last_sampling_time = 0;
  Serial.begin(57600);
}

void loop() {
  if (millis() < last_sampling_time + INTERVAL) return;
  Serial.print("Hi Do you working?\n");
  last_sampling_time += INTERVAL;
}
