//Project2
//Group5
//Angel Alcantara, Brandon Aparicio, , Diego Perez, Sarah Kemppainen:
// Febuary, 26, 2025

//The data collection program

// Define pin mappings
const int sharpPin1 = A7;  // First sensor
const int sharpPin2 = A6;  // Second sensor
const int sharpPin3 = A14;  // Third sensor

// Global variables for time tracking
unsigned long current_time, last_sensor_time;
const unsigned long sensor_interval = 2500; // Interval in milliseconds

void setup() {
    Serial.begin(115200); // Initialize serial at 115200 baud
}


void loop() {
    current_time = millis();
    if (current_time - last_sensor_time > sensor_interval) {
        Serial.printf("Entering sensor_display_task()\n");
        sensor_display_task();
        last_sensor_time = current_time;
    }
}


/*
 * Function: read_distance
 * Description: This function takes an analog sensor value as input and 
 * calibrates it to a distance in cm using a calibration equation.
 *
 * Parameters:
 * - int sensorValue: The analog sensor value that can be converted
 *
 * Return: 
 * - float: the distance sensed in cm
 *
 * External Effects: None
 */
float read_distance(int sensorValue) {
    // Initialize distance variable
    float distance = 0.0;

    // ***** CALIBRATION EQUATION *****
    distance = 19946 * pow(sensorValue, -1.155);
    Serial.printf("sensorValue = %d, distance = %f\n", sensorValue, distance);

    // ***** CLAMP DISTANCE to avoid unrealistic values *****
    // Check that distance is within (8 cm, 80 cm)
    //
    if (distance < 8) {
      Serial.printf("minimum distance sensed, setting distance = 8\n");
      distance = 8; // minimum distance 
    } else if (distance > 80) {
        Serial.printf("maximum distance sensed, setting distance = 80\n");
        distance = 80; // Maximum distance
    }

   return distance; // Return calibrated distance
}


//Testing the New Sensor Function
//Modify the sensor_display_task() function to print calibrated distances in centimeters. Use the read_distance() function you implemented.

/*
 * Function: sensor_display_task
 * Description: This function is called to read the inputs of each sensor, and
 * print the distances read in cm 
 *
 * Parameters: None (void)
 *
 * Return: None (void)
 *
 * External Effects: Reads the analog values of sensors
 */
void sensor_display_task() {

    //Serial.printf("Reading distance 1...");
    int val1 = analogRead(sharpPin1); // read analog sensor value for sensor 1 (black)
    float distance1 = read_distance(val1);  // convert distance to cm
    //Serial.printf("Distance 1 calcuated\n");
   
    //Serial.printf("Reading distance 2...\t");
    int val2 = analogRead(sharpPin2); // read analog sensor value for sensor 2 (orange)
    float distance2 = read_distance(val2);  // convert distance to cm
    //Serial.printf("Distance 2 calcuated\n");
   
    Serial.printf("Reading distance 3...\t\t");
    int val3 = analogRead(sharpPin3); // read analog sensor value for sensor 3 (green)
    float distance3 = read_distance(val3);  // convert distance to cm
    Serial.printf("Distance 3 calcuated\n");
   
    // Print calibrated distances
    Serial.printf("Distance 1: %.2f cm, Distance 2: %.2f cm, Distance 3: %.2f cm\n", distance1, distance2, distance3);
  
}

// devolping the sensor module You may want to create a model that prioritizes accuracy at 8 cm while still fitting other points.
/*
float read_distance(int sensorValue) {
    // Implement your calibration algorithm here
    float distance = 0.0;

    // Example model (linear or polynomial fit can be used):
    // You need to replace this with your fitted model based on collected data.
   
   // Determine if the sensorValue is within range [0, 1023]
    if ((sensorValue >= 0 && sensorValue <= 1023)) {
        // ***** CALIBRATION EQUATIONS *****
        //
        // Sensor 2 (orange)
        distance = (-7.9985 * sensorValue) + 646.8;
        Serial.printf("distance calculated = %f", distance);
    } else {
      Serial.printf("read_distance given sensorValue that exceeds [0, 1023]");
      return -999;  // Indicates that there is an error
    }
   
    // Clamp to avoid unrealistic values
    if (distance < 8) {
        Serial.printf("minimum distance sensed, returning distance = 8\n");
        return 8; // Minimum distance
    } else if (distance > 80) {
        Serial.printf("maximum distance sensed, returning distance = 80\n");
        return 80; // Maximum distance
    }
   
    return distance; // Return calibrated distance
}
*/



