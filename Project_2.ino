void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100000000);
  delay(1000);

}

void loop() {
  \\ Prompt the user for input.
  Serial.printf("Enter the distance.\n");
  \\ Read the value.
  int i = Serial.parseInt();

}
