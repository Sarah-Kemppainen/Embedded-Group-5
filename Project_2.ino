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

  float min_dist = 0.1;
  float max_dist = 0.8;
  float range = (max_dist - min_dist) / 9;
  String grid = "";
  
  const String states[] = {"10000", "11000", "01000", "01100", "00100", "00110", "00010", "00011", "00001"};

  Serial.printf("min distance: %0.1f m; max distance: %0.1f m; dist range of each light state: %f\n", min_dist, max_dist, range);

  for (int i = 0; i < 9; i++) {
    float lower_bound = min_dist + (range * i);
    float upper_bound = min_dist + (range * (i+1));

    if (dist >= lower_bound && dist < upper_bound) { 
      grid = states[i]; 
      Serial.printf("range is [%1.2f, %1.2f)\n", lower_bound, upper_bound);
      break;
    }

  }

  if (dist == max_dist) {
    grid = "00001";
    Serial.printf("dist is 0.8 m\n");
  }

  if (grid.length() == 0) {
    Serial.printf("Distance not within range. No statement entered\n");
  }

  Serial.printf("Grid: ");
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
