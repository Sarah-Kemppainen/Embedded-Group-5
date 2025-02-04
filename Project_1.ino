// define LED pin ports (output)
#define LED_PIN1 32
#define LED_PIN2 31
#define LED_PIN3 30
#define LED_PIN4 29

// define switch port (input)
#define SwitchIn A0

int switchState = 0;  // stores if switch is in the true or false position

void setup() {
  // Configure LED ports
  pinMode(LED_PIN1, OUTPUT);  // set pin 32 as an ouput
  pinMode(LED_PIN2, OUTPUT);  // set pin 31 as an output
  pinMode(LED_PIN3, OUTPUT);  // set pin 30 as an output
  pinMode(LED_PIN4, OUTPUT);  // set pin 29 as an output

  // Configure switch port
  pinMode(SwitchIn, INPUT); // set port () as an input

} // void setup()

void loop() {
  // read switch state
  switchState = digitalRead(SwitchIn);

  // enter pattern if switch state is true
  if (switchState == HIGH) {

    // turn pin1 on and off
    analogWrite(LED_PIN1, 200); // turn pin1 on
    delay(500);                 // wait 500 ms
    analogWrite(LED_PIN1, 0);   // turn pin1 off
    delay(500);                 // wait 500 ms

    // turn pin2 on and off
    analogWrite(LED_PIN2, 200);
    delay(500);
    analogWrite(LED_PIN2, 0);
    delay(500);

    // turn pin3 on and off
    analogWrite(LED_PIN3, 200);
    delay(500);
    analogWrite(LED_PIN3, 0);
    delay(500);

    // turn pin4 on and off
    analogWrite(LED_PIN4, 200);
    delay(500);
    analogWrite(LED_PIN4, 0);
    delay(500);

  } // if (switchState == HIGH)
} // void loop()
