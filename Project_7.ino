//Project7
//Group5
//Angel Alcantara, Brandon Aparicio, Diego Perez, Sarah Kemppainen:
// April, 16, 2025
// Include necessary libraries for BNO-055
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>          // Using standard Servo library
#include <cassert>          // For testing

Servo fan; // create servo object to control the fan

// Define pins
#define SWITCH_PIN 31        // Switch input pin
#define CENTRAL_FAN_PIN 2   // Central fan control pin

// Define pins for thrusters (update these to match your actual connections)
#define RIGHT_THRUSTER_PIN 23       // BLUE (right)                           //FIXME: old pwmPin1
#define LEFT_THRUSTER_PIN 22        // BLACK (left)                           //FIXME: old pwmPin2
#define BACK_THRUSTER_PIN 38       // RED (rear)                              //FIXME: old pwmPin3

// Controller parameters
const float KP = 0.2;  // Start with a small value and adjust during testing
const float KD = 0.1;  // Start with a small value and adjust during testing
const float DEADBAND = 5.0;  // 5 degrees deadband
const float SATURATION = 90.0;  // 90 degrees saturation

int orientationCalled = 0;


// Define duty parameters
#define CENTRAL_FAN_DUTY 100
#define SIDE_MOTOR_DUTY 175

float motors_vals[3] = {0,0,0};

const int NA1 = 37; // BLUE motor
const int NB1 = 14; // BLUE motor
const int NA2 = 36; // BLACK motor
const int NB2 = 35; // BLACK motor
const int NA3 = 8;  // RED motor
const int NB3 = 7;  // RED motor


// Controller parameters
#define Kd 0.5               // Damping coefficient for rotation control

// State machine states
enum CentralFanState {
  STATE_WAITING,
  STATE_FAN_ON,
  STATE_FAN_OFF
};


float goal_heading = 0.0;

/**
 * PeriodicAction class - Used to schedule and manage periodic tasks
 * This replaces the need for PeriodicAction.h
 */
class PeriodicAction {
private:
  unsigned long period;      // Period in milliseconds
  unsigned long lastRunTime; // Time of last run

public:
 
   //Constructor for PeriodicAction class
   // @param periodMs: Time period in milliseconds at which the action should occur
   // @return None
   // External Effects: Initializes a new PeriodicAction object
   
  PeriodicAction(unsigned long periodMs) {
    period = periodMs;
    lastRunTime = 0;
  }

  /*
   * Checks if it's time to run the periodic action
   *
   * @param None
   * @return bool: True if period has elapsed since last run, False otherwise
   * External Effects: Updates lastRunTime if period has elapsed
   */
  bool isReady() {
    unsigned long currentTime = millis();
    if (currentTime - lastRunTime >= period) {
      lastRunTime = currentTime;
      return true;
    }
    return false;
  }

  /*
   * Resets the timer for the periodic action
   *
   * @param None
   * @return None
   * External Effects: Updates lastRunTime to current time
   */
  void reset() {
    lastRunTime = millis();
  }
};

// Global variables
CentralFanState currCentralFanState = STATE_WAITING;
unsigned long stateStartTime = 0;
const unsigned long FAN_RUN_TIME = 30000; // 30 seconds in milliseconds

// For BNO-055
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Task objects
PeriodicAction imuTask(5);       // IMU update task (5ms)
PeriodicAction pdTask(10);       // PD control task (10ms)
PeriodicAction reportTask(1000); // Reporting task (1s)
PeriodicAction fsmTask(100);     // FSM task (100ms)

/*
 * Sets forces to control hovercraft movement and rotation
 *
 * @param fx: Unit value that indicates force in the x-direction
 * @param fy: Unit value that indicates force in the y-direction
 * @param torque: Unit value that indicates rotational force
 * @return None
 * External Effects: Updates motor values and activates side motors through set_motors function
 */
const float R = 5.5;
void set_hovercraft_forces(float fx, float fy, float torque) {
  // Convert forces to motor commands
  // Map fx, fy, and torque to actual motor outputs
 
  // Example implementation (to be replaced with your Project 6 code)
  motors_vals[0] =  (1 / (2 * cos(30 * PI / 180))) * fx +
                   (1 / (2 + 2 * sin(30 * PI / 180))) * fy +
                   (1 / (2 * R * (1 + sin(30 * PI / 180)))) * torque; // Left motor (BLACK)

  motors_vals[1] = (1 / (2 * cos(30 * PI / 180))) * fx -
                   (1 / (2 + 2 * sin(30 * PI / 180))) * fy -
                   (1 / (2 * R * (1 + sin(30 * PI / 180)))) * torque; // Right motor (BLUE)

  motors_vals[2] = 0 -
                   (1 / (2 + 2 * sin(30 * PI / 180))) * fy +
                   (sin(30 * PI / 180) / (R * (1 + sin(30 * PI / 180)))) * torque; // Rear motor (RED)

  //Serial.printf("left: %.2f ----- right: %.2f ----- back: %.2f\n", motors_vals[0],  motors_vals[1],  motors_vals[2]);
 
  // Scale forces to maximize motor usage
  for (int i = 0; i < 3; i++) {
    motors_vals[i] = bound(motors_vals[i] * SIDE_MOTOR_DUTY, -255, 255);  // Using full 8-bit range
  }

  //Serial.printf("passing motors_vals to set_motors: %.2f, %.2f, %.2f\n", motors_vals[0],  motors_vals[1],  motors_vals[2]);
 
  set_motors(motors_vals); // Apply thrust
 
}

/*
 * Initializes the thruster pins and ensures they start in the off state
 *
 * @param None
 * @return None
 * External Effects: Configures thruster pins as outputs and sets initial values to 0
 */
void setup_thrusters() {
  // Initialize thruster pins
  pinMode(LEFT_THRUSTER_PIN, OUTPUT);
  pinMode(RIGHT_THRUSTER_PIN, OUTPUT);
  pinMode(BACK_THRUSTER_PIN, OUTPUT);
 
  // Initialize with thrusters off
  analogWrite(LEFT_THRUSTER_PIN, 0);
  analogWrite(RIGHT_THRUSTER_PIN, 0);
  analogWrite(BACK_THRUSTER_PIN, 0);
}

/*
 * Constrains a value within a specified range
 *
 * @param value: The value to check and bound
 * @param min_value: The minimum allowable value
 * @param max_value: The maximum allowable value
 * @return float: The bounded value, ensuring it stays within [min_value, max_value]
 * External Effects: None
 */
float bound(float value, float min_value, float max_value) {
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

/*
 * Sets thrust and direction for a single motor
 *
 * @param motor: Index of the motor to control (0 = BLUE/Right, 1 = BLACK/Left, 2 = RED/Back)
 * @param val: PWM value for controlling the motor (-255 to 255)
 *             Positive values drive the motor forward, negative values drive it backward
 * @return None
 * External Effects: Modifies the state of the control pins (NA and NB)
 *                  for the specified motor, setting its direction and speed
 */
void set_motor(int motor, float val) {
  val = bound(val, -255, 255);  // Using full 8-bit range for more power
 
  if (motor == 0) {
    if (val > 0) {
      digitalWrite(NA1, HIGH);  // Set direction for forward
      digitalWrite(NB1, LOW);   // Ensure opposite control is low
      analogWrite(RIGHT_THRUSTER_PIN, val); // Set PWM value
    } else if (val < 0) {
      digitalWrite(NA1, LOW);   // Set direction for backward
      digitalWrite(NB1, HIGH);  // Ensure opposite control is high
      analogWrite(RIGHT_THRUSTER_PIN, -val); // Set PWM value (use -val as value cannot be negative)
    } else {
      // For val == 0, ensure motor is stopped
      digitalWrite(NA1, LOW);
      digitalWrite(NB1, LOW);
      analogWrite(RIGHT_THRUSTER_PIN, 0);
    }
  } else if (motor == 1) {
    if (val > 0) {
      digitalWrite(NA2, HIGH);  // Set direction for forward
      digitalWrite(NB2, LOW);   // Ensure opposite control is low
      analogWrite(LEFT_THRUSTER_PIN, val); // Set PWM value
    } else if (val < 0) {
      digitalWrite(NA2, LOW);   // Set direction for backward
      digitalWrite(NB2, HIGH);  // Ensure opposite control is high
      analogWrite(LEFT_THRUSTER_PIN, -val); // Set PWM value (use -val as value cannot be negative)
    } else {
      // For val == 0, ensure motor is stopped
      digitalWrite(NA2, LOW);
      digitalWrite(NB2, LOW);
      analogWrite(LEFT_THRUSTER_PIN, 0);
    }
  } else if (motor == 2) {
    if (val > 0) {
      digitalWrite(NA3, HIGH);  // Set direction for forward
      digitalWrite(NB3, LOW);   // Ensure opposite control is low
      analogWrite(BACK_THRUSTER_PIN, val); // Set PWM value
    } else if (val < 0) {
      digitalWrite(NA3, LOW);   // Set direction for backward
      digitalWrite(NB3, HIGH);  // Ensure opposite control is high
      analogWrite(BACK_THRUSTER_PIN, -val); // Set PWM value (use -val as value cannot be negative)
    } else {
      // For val == 0, ensure motor is stopped
      digitalWrite(NA3, LOW);
      digitalWrite(NB3, LOW);
      analogWrite(BACK_THRUSTER_PIN, 0);
    }
  }
}

/*
 * Sets thrust and direction for all three motors simultaneously
 *
 * @param val: Array of three PWM values (-255 to 255) for controlling each motor
 * @return None
 * External Effects: Calls set_motor for each of the three motors
 */
void set_motors(float val[3]) {
  for (int i = 0; i < 3; ++i) {
    set_motor(i, val[i]);
  }
}

/*
 * Sets up the central fan with a low initial throttle value
 *
 * @param None
 * @return None
 * External Effects: Attaches the fan to the specified pin and
 *                  sets it to a low throttle value for proper initialization
 */
void fan_setup() {
    fan.attach(CENTRAL_FAN_PIN); // attaches the fan to specified Arduino pin
    delay(100);
    fan.write(20); // write low throttle
    Serial.println("Setting up central fan");
    delay(3000);
}


 // Main setup function that initializes all components of the hovercraft
 // @param None
 // @return None
 // External Effects: Initializes serial communication, sets pin modes,
 //                  configures initial motor states, sets up fan and BNO-055 sensor
 
void setup() {
  Serial.begin(115200);             // FIXME 9600 (?)
  while (!Serial) delay(10);
  Serial.println("Hovercraft Controller with BNO-055");
 
  // Setup switch pin
  pinMode(SWITCH_PIN, INPUT_PULLUP);    // FIXME: In the past we've just used INPUT

  Serial.println("Hovercraft control starting up");

  // Configure motor pins
  pinMode(RIGHT_THRUSTER_PIN, OUTPUT);
  pinMode(NA1, OUTPUT);
  pinMode(NB1, OUTPUT);

  pinMode(LEFT_THRUSTER_PIN, OUTPUT);
  pinMode(NA2, OUTPUT);
  pinMode(NB2, OUTPUT);

  pinMode(BACK_THRUSTER_PIN, OUTPUT);
  pinMode(NA3, OUTPUT);
  pinMode(NB3, OUTPUT);

  // Initial state of motors - make sure they're OFF
  digitalWrite(NA1, LOW);
  digitalWrite(NB1, LOW);
  digitalWrite(NA2, LOW);
  digitalWrite(NB2, LOW);
  digitalWrite(NA3, LOW);
  digitalWrite(NB3, LOW);
  analogWrite(RIGHT_THRUSTER_PIN, 0);
  analogWrite(LEFT_THRUSTER_PIN, 0);
  analogWrite(BACK_THRUSTER_PIN, 0);
 
  // Setup central fan
  fan_setup();
  Serial.println("Central fan initialized");

  // Setup thrusters (from Project 6)
  setup_thrusters();
 
  // Initialize BNO-055
  if (!bno.begin()) {
    Serial.println("BNO055 initialization failed");
    while (1) {
      // Flash an LED or other indicator to show error
      digitalWrite(13, HIGH);
      delay(100);
      digitalWrite(13, LOW);
      delay(100);
    }
  }
  delay(1000); // Give the sensor time to stabilize
  bno.setExtCrystalUse(true);

  // Run tests
  run_tests_proj7();    // runs tests for compute_error, db_clip, pd_step() 
 
  Serial.println("System initialized. Press switch to start.");
}

/*
 * Main loop function that schedules and executes periodic tasks
 *
 * @param None
 * @return None
 * External Effects: Calls task functions when their respective periods have elapsed
 */
void loop() {
  // Check if it's time to run tasks
  if (imuTask.isReady()) imu_step();
  if (pdTask.isReady()) pd_step();
  //if (reportTask.isReady()) report_step();    // Function prints pitch roll and yaw
  if (fsmTask.isReady()) fsm_step();
}

/*
 * Updates BNO-055 IMU sensor data
 *
 * @param None
 * @return None
 * External Effects: Reads fresh data from the BNO-055 sensor
 */
void imu_step() {
  // Update BNO-055 data
  sensors_event_t event;
  bno.getEvent(&event);
  // This updates the sensor data but doesn't store it - the getVector()
  // calls in other functions will fetch the specific data we need
}

/*
 * Implements the proportional-derivative control step to stabilize the hovercraft
 *
 * @param None
 * @return None
 * External Effects: Reads gyroscope data from BNO-055 and applies
 *                  torque correction through the set_hovercraft_forces function
 */
void pd_step() {
   // Get current heading from sensor
  float theta = get_heading();
 
  // Calculate heading error
  float heading_error = compute_error(theta, goal_heading);
 
  // Apply deadband and saturation
  float modified_error = db_clip(heading_error, DEADBAND, SATURATION);
 
  // Get angular velocity for derivative term
  float omega = get_angular_velocity();
 
  // Calculate control output using PD control law
  float fx = 0.0;  // No forward/backward thrust for this task
  float fy = 0.0;  // No lateral thrust for this task
  float torque = KP * modified_error - KD * omega;  // PD control law
 
  // Apply forces to the hovercraft
  set_hovercraft_forces(fx, fy, torque);
}

/*
 * Reports system status data through serial communication
 *
 * @param None
 * @return None
 * External Effects: May print sensor data to Serial when uncommented
 */
void report_step() {
  // Get orientation and rotation data for reporting
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
 
  Serial.print("Rotation Rate (rad/s): ");
  Serial.print(gyro.z());
  Serial.print("\tOrientation: Yaw=");
  Serial.print(euler.x());
  Serial.print(" Pitch=");
  Serial.print(euler.y());
  Serial.print(" Roll=");
  Serial.println(euler.z());
}

/*
 * Implements the finite state machine that controls the hovercraft operation sequence
 *
 * @param None
 * @return None
 * External Effects: Reads switch state, changes currCentralFanState variable, and
 *                  turns the central fan on or off based on state transitions
 */
void fsm_step() {
  // Define states for central fan
  // Read switch state (active low with pull-up)
  bool switchPressed = (digitalRead(SWITCH_PIN) == LOW);
  unsigned long currentTime = millis();
 
  switch (currCentralFanState) {
    case STATE_WAITING:
      if (switchPressed) {
        // Switch pressed, turn on fan and transition to FAN_ON state
        fan.write(CENTRAL_FAN_DUTY);                                                  
        stateStartTime = currentTime;
        currCentralFanState = STATE_FAN_ON;
        Serial.println("Fan turned ON");
      }
      break;
     
    case STATE_FAN_ON:
      if (currentTime - stateStartTime >= FAN_RUN_TIME) {
        // 30 seconds elapsed, turn off fan and transition to FAN_OFF state
        fan.write(0);
        set_hovercraft_forces(0,0,0);
        currCentralFanState = STATE_FAN_OFF;
        Serial.println("Fan turned OFF");
      }
      break;
     
    case STATE_FAN_OFF:
      // Return to waiting state to complete the cycle
      currCentralFanState = STATE_WAITING;
      Serial.println("Ready for next cycle");
      break;
  }

}



/************************ PROJECT 7 FUNCTIONS *************************/
/**
 * Computes the heading error between current heading and goal
 * @param theta Current heading in degrees
 * @param heading_goal Goal heading in degrees
 * @return error in degrees (-180 to 180)
 */
float compute_error(float theta, float heading_goal) {
  // Calculate the raw difference
  float error = heading_goal - theta;
 
  // Normalize to the range [-180, 180]
  while (error > 180.0) {
    error -= 360.0;
  }
  while (error < -180.0) {
    error += 360.0;
  }
 
  return error;
}

/**
 * Applies deadband and saturation to the error
 * @param error Input error in degrees
 * @param deadband Deadband threshold in degrees
 * @param saturation Saturation threshold in degrees
 * @return Modified error with deadband and saturation applied
 */
float db_clip(float error, float deadband, float saturation) {         

  // Error is between +/- deadband
  if ((-deadband <= error) && (error <= deadband)) {
    return 0.0;
  } 

  else if (error >= saturation) {
    return saturation - deadband;
  }
  else if (error <= (-1 * saturation)) {
    return - (saturation - deadband);;
  }

  else if (deadband < error) { // error < saturation already guarenteed from prev elif
    return error - deadband;
  }
  else if (error < -deadband) {
    return error + deadband;
  }

  return 0.0;

}

/**
 * Gets the current heading from the BNO055 sensor
 * @return heading in degrees (-180 to 180)
 */
float get_heading() {
  // Get orientation data from the BNO055
  sensors_event_t event;
  bno.getEvent(&event);
 
  // Convert to range (-180, 180)
  float heading = event.orientation.x;
  if (heading > 180) {
    heading -= 360;
  }
 
  return heading;
}

/**
 * Gets the current angular velocity from the BNO055 sensor
 * @return angular velocity in degrees per second
 */
float get_angular_velocity() {
  // Get gyro data from the BNO055
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
 
  // Return the z-axis angular velocity (in degrees per second)
  return gyro.z();
}

/************************* TEST CODE ********************************/
// Runs tests for Proejct 7 Functions
void run_tests_proj7() {
  // compute_error() tests
  assert(compute_error(-90.0, 0.0) == 90.0);      // returns 90 deg CW (90.0) error from 0.0 heading
  assert(compute_error(90.0, 0.0) == -90.0);      // returns 90 deg CCW (-90.0) error from 0.0 heading
  assert(compute_error(270.0, 0.0) == 90.0);      // returns 90 deg CCW (90.0) error from 0.0 heading
  assert(compute_error(-1170.0, 0.0) == 90.0);    // returns 90 deg CCW (90.0) error from 0.0 heading

  assert(compute_error(-90.0, 78.0) == 168.0);    // returns a 168.0 deg CCW (168.0) error from 78.0 heading
  assert(compute_error(-180.0, 78.0) == -102.0);  // returns a 102.0 deg CW (-102.0) error from 78.0 heading

  // db_clip() tests
  // For an error that is +/- deadband, this function returns a zero.
  assert(db_clip(5, 5, 90) == 0.0);   // edge of deadband
  assert(db_clip(-5, 5, 90) == 0.0);  // other edge
  assert(db_clip(4.9, 5, 90) == 0.0); // inside deadband

  // For an error that is larger than saturation, this function returns saturation - deadband.
  assert(db_clip(100, 5, 90) == (90-5));   
  assert(db_clip(-100, 5, 90) == -(90-5));

  // For an error between deadband and saturation, the return value linearly increases with increasing error (gain=1).
  assert(db_clip(45, 5, 90) == 40);
  assert(db_clip(-45, 5, 90) == -40);
    
  Serial.println("compute_error function tests\t...........................\tpassed");

}

