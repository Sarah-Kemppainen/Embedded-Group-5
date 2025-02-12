void display_distance(float dist);

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100000000);
  delay(1000);

  // setup graph bar
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  //pinMode(8, OUTPUT);

}

void display_distance(float dist) {
  // write display_distance function here
  //dist_in_meters = dist / 1000

  int pins_turned_on = dist / 100;
  Serial.printf("%d:\n", pins_turned_on);

  int pin_states[5];
  for (int i = 0; i < 5; i++) {
    if (i <= pins_turned_on){
      pin_states[i] = 1;
      digitalWrite(i, HIGH);
    } else {
      pin_states[i] = 0;
      digitalWrite(i, LOW);
    }
    Serial.printf("%d ", pin_states[i]);
  }
  Serial.printf("\n");


/*
  for (int i = 2; i <= pins_turned_on; i++) {
    Serial.printf("Turning on pin %d\n", i);
    digitalWrite(i, HIGH);
  }
  for (int i = 7; i > pins_turned_on; i--) {
    Serial.printf("Turning off pin %d\n", i+1);
    digitalWrite(i, LOW);
  }
*/

}

void loop() {
  Serial.printf("\n\n***** NEW RUN *****\n");
  // Prompt the user for input.
  Serial.printf("Enter the distance (in milimeters) within the range [100 mm, 800 mm]:\n");
  // Read the value.
  int i = Serial.parseInt();


  display_distance(i);

}
