void setup() {

  // Configure LED ports
  int led1 = 17;
  int led2 = 18;
  int led3 = 19;
  int led4 = 20;

  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  pinMode(led4, OUTPUT);

  // configure switch (port 33-35) 

  // configure LEDS (ports 20-17)

}

void loop() {
  // enter condition if switch is pressed("if (pressed == true) { }")

  // start pattern (GPIOx_PDOR = (GPIOx_PDOR & ~mask) | new_pattern;)

  analogWrite(led1, 200);
  delay(500);
  analogWrite(led2, 200);
  delay(500);
  analogWrite(led3, 200);
  delay(500);
  analogWrite(led4, 200);

    
  



}
