//Project2
//Group5
//Angel Alcantara, Brandon Aparicio, , Diego Perez, Sarah Kemppainen:
// Febuary, 14, 2025


void display_distance(float dist);

/*
 * Function: setup
 * Description: This function initializes the serial communication and sets the pin modes
 * for the output pins that control the LEDs.
 *
 * Parameters: None
 *
 * Return: None (void)
 *
 * External Effects: Initializes serial communication with a baud rate of 115200
 * and sets specific pins as OUTPUT to control LEDs.
 */
void setup() {
  Serial.begin(115200); // Start serial communication at 115200 baud rate
  Serial.setTimeout(100000000);// Set the serial timeout
  delay(1000);// Wait for the system to settle
  // setup graph bar
  pinMode(2, OUTPUT);// Set pin 2 as output
  pinMode(3, OUTPUT);// Set pin 3 as output
  pinMode(4, OUTPUT);// Set pin 4 as output
  pinMode(5, OUTPUT);// Set pin 5 as output
  pinMode(6, OUTPUT);// Set pin 6 as output
  //pinMode(7, OUTPUT);
  //pinMode(8, OUTPUT);
}
/*
 * Function: display_distance
 * Description: This function takes a distance as input and controls multiple LEDs
 * according to predefined distance ranges. The LED states are represented in binary form.
 *
 * Parameters:
 * - float dist: The distance in meters which determines the LED state.
 *
 * Return: None (void)
 *
 * External Effects: This function modifies the states of LEDs connected to pins 2 to 6.
 */
void display_distance(float dist) {

  float min_dist = 0.1; // Minimum distance threshold
  float max_dist = 0.8; // Maximum distance threshold
  float range = (max_dist - min_dist) / 9; // Calculate the range of each segment
  String grid = "";  // Initialize grid string to store LED states

// Define possible states for reference
  const String states[] = {"10000", "11000", "01000", "01100", "00100", "00110", "00010", "00011", "00001"};

  // Log distance range and segment details
  Serial.printf("min distance: %0.1f m; max distance: %0.1f m; dist range of each light state: %f\n", min_dist, max_dist, range);

 // Determine which range the distance falls into
  for (int i = 0; i < 9; i++) {
    float lower_bound = min_dist + (range * i); // Calculate lower bound for segmentation
    float upper_bound = min_dist + (range * (i+1));// Calculate upper bound for segmentation

// Check if the distance falls within the current segment
    if (dist >= lower_bound && dist < upper_bound) {
      grid = states[i];
      Serial.printf("range is [%1.2f, %1.2f)\n", lower_bound, upper_bound);
      break; // Exit loop once the right range is found
    }

  }

// Special handling if distance is exactly the maximum
  if (dist == max_dist) {
    grid = "00001";
    Serial.printf("dist is 0.8 m\n");
  }

// Log if no valid grid was set
  if (grid.length() == 0) {
    Serial.printf("Distance not within range. No statement entered\n");
  }

// Print the final grid state for debugging
  Serial.printf("Grid: ");
  Serial.println(grid);


  // i = 0 ==> pin = 2
  // Control LEDs based on the grid state
  char ON = '1';// Define ON state character
  for (int i = 0; i < 5; i++) {
    if (grid[i] == ON) {
      // Turn LED on if the corresponding bit is '1'
      digitalWrite(i+2, HIGH);
    }
    else {
       // Turn LED off if the corresponding bit is '0'
      digitalWrite(i+2, LOW);
    }
  }
}

/*
 * Function: loop
 * Description: This function runs in a continuous loop, prompting the user for
 * distance input and calling the display_distance function with the input value.
 *
 * Parameters: None
 *
 * Return: None (void)
 *
 * External Effects: Reads user input from serial and displays LED states based on the distance.
 */
void loop() {
  Serial.printf("\n\n***** NEW RUN *****\n");// Indicate a new run
  // Prompt the user for input.
  Serial.printf("Enter the distance (in milimeters) within the range [100 mm, 800 mm]:\n");
  // Read the value.
  int i = Serial.parseInt();// Parse the integer input
  Serial.printf("distance recieved: %d\n", i);// Log the received distance
  float dist = i * 0.001;  // convert distance to meters
  Serial.printf("distance in meters: %f\n", dist);// Log the distance in meters
  display_distance(dist); // Call display_distance to update LED states based on distance
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