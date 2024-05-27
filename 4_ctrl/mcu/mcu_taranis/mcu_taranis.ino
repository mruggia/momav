
// TARANIS "External RF" must be set to SBUS and "not inverted" !!!

void setup() {
  Serial.begin(115200);
  Serial1.begin(100000, SERIAL_8E2);
}

void loop() {
  if(Serial1.available()) {
    Serial.write(Serial1.read());
  }
}
