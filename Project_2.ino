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
  //pinMode(7, OUTPUT);
  //pinMode(8, OUTPUT);

}

void display_distance(float dist) {
  // write display_distance function here

  // Case 1: if dist = [0.0, 0.1) { grid = 10000 }
  // Case 2: if dist = [0.1, 0.2) { grid = 11000 }
  // Case 3: if dist = [0.2, 0.3) { grid = 01000 }
  // Case 4: if dist = [0.3, 0.4) { grid = 01100 }
  // Case 5: if dist = [0.4, 0.5) { grid = 00100 }
  // Case 6: if dist = [0.5, 0.6) { grid = 00110 }
  // Case 7: if dist = [0.6, 0.7) { grid = 00010 }
  // Case 8: if dist = [0.7, 0.8) { grid = 00001 }

  String grid = "";
  if ((dist >= 0.0) && (dist < 0.1)) { grid = "10000"; Serial.printf("grid set to 10000\n"); }
  else if ((dist >= 0.1) && (dist < 0.2)) { grid = "11000"; Serial.printf("grid set to 11000\n"); }
  else if ((dist >= 0.2) && (dist < 0.3)) { grid = "01000"; Serial.printf("grid set to 01000\n"); }
  else if ((dist >= 0.3) && (dist < 0.4)) { grid = "01100"; Serial.printf("grid set to 01100\n"); }
  else if ((dist >= 0.4) && (dist < 0.5)) { grid = "00100"; Serial.printf("grid set to 00100\n"); }
  else if ((dist >= 0.5) && (dist < 0.6)) { grid = "00110"; Serial.printf("grid set to 00110\n"); }
  else if ((dist >= 0.6) && (dist < 0.7)) { grid = "00010"; Serial.printf("grid set to 00010\n"); }
  else if ((dist >= 0.7) && (dist < 0.8)) { grid = "00011"; Serial.printf("grid set to 00011\n"); }
  else if (dist == 0.8) { grid = "00001"; Serial.printf("grid set to 00001\n"); }
  else { Serial.printf("Distance not within range. No statement entered\n"); }

  Serial.println(grid);

  // i = 0 ==> pin = 2
  char ON = '1';

  for (int i = 0; i < 5; i++) {
    if (grid[i] == ON) {
      // turn LED on
      digitalWrite(i+2, HIGH);
    }
    else {
      // turn LED off
      digitalWrite(i+2, LOW);
    }

  }

}

void loop() {
  Serial.printf("\n\n***** NEW RUN *****\n");
  // Prompt the user for input.
  Serial.printf("Enter the distance (in milimeters) within the range [100 mm, 800 mm]:\n");
  // Read the value.
  int i = Serial.parseInt();
  Serial.printf("distance recieved: %d\n", i);

  float dist = i * 0.001;  // convert distance to meters
  Serial.printf("distance in meters: %f\n", dist);

  display_distance(dist);

  // Grid Test
  /*
  digitalWrite(2, HIGH);
  delay(250);
  digitalWrite(2, LOW);
  delay(250);

  digitalWrite(3, HIGH);
  delay(250);
  digitalWrite(3, LOW);
  delay(250);

  digitalWrite(4, HIGH);
  delay(250);
  digitalWrite(4, LOW);
  delay(250);

  digitalWrite(5, HIGH);
  delay(250);
  digitalWrite(5, LOW);
  delay(250);

  digitalWrite(6, HIGH);
  delay(250);
  digitalWrite(6, LOW);
  delay(250);
  */



}
