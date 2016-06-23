void setup() {
  pinMode(0, OUTPUT);
}

void loop() {
  digitalWrite(0, HIGH);
  delay(2000);
  digitalWrite(0, LOW);
  delay(1000);
}
