#define LED_PIN1 32 // define the LED pin
#define LED_PIN2 31
#define LED_PIN3 30
#define LED_PIN4 29

void setup() {
  // Configure LED ports
  pinMode(LED_PIN1, OUTPUT);// set pin 32 as an ouput
  pinMode(LED_PIN2, OUTPUT);
  pinMode(LED_PIN3, OUTPUT);
  pinMode(LED_PIN4, OUTPUT);
}

void loop() {

  analogWrite(LED_PIN1, 200);
  delay(500);
  analogWrite(LED_PIN1, 0);
  delay(500);

  analogWrite(LED_PIN2, 200);
  delay(500);
  analogWrite(LED_PIN2, 0);
  delay(500);

  analogWrite(LED_PIN3, 200);
  delay(500);
  analogWrite(LED_PIN3, 0);
  delay(500);

  analogWrite(LED_PIN4, 200);
  delay(500);
  analogWrite(LED_PIN4, 0);
  delay(500);
 

}
