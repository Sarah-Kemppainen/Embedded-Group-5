//Project8
//Group5
//Angel Alcantara, Brandon Aparicio, Diego Perez, Sarah Kemppainen:
// April, 21, 2025

#include <Servo.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Create IMU sensor instance
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// Servo objects
Servo fan; // Central lift fan

// Motor control pins
const int pwmPin1 = 23; // BLUE motor
const int pwmPin2 = 22; // BLACK motor
const int pwmPin3 = 38; // RED motor
const int pwmPin4 = 37; // CENTRAL motor

const int NA1 = 13; // BLUE motor
const int NB1 = 39; // BLUE motor
const int NA2 = 36; // BLACK motor
const int NB2 = 35; // BLACK motor
const int NA3 = 9;  // RED motor
const int NB3 = 8;  // RED motor

// IR sensor pins
const int sharpPin1 = A7;  // Left sensor
const int sharpPin2 = A6;  // Right sensor
const int sharpPin3 = A14; // Rear sensor

// Switch pin
#define SWITCH_PIN 31
int switchState = 0;

// Central fan settings
const int CENTRAL_MIN = 20;   // Minimum value for startup
const int CENTRAL_duty = 120; // Operational duty value

// Motor power settings
const int fsm_interval = 250; // FSM update interval in ms
const int duty = 200;         // Motor duty cycle max value

// Rangefinder settings
const int range_interval = 50; // Run rangefinder every 50ms

// Control parameters
const float Kp = 1.5;  // Proportional gain for orientation control (increased from 0.8)
const float Kd = 0.5;  // Derivative gain for orientation control (increased from 0.3)
const float Kpx = 1.2; // Proportional gain for x-direction obstacle avoidance
const float Kpy = 1.2; // Proportional gain for y-direction obstacle avoidance

// Global variables
float motors_vals[3] = {0, 0, 0}; // Motor values array
float yaw = 0;                    // Current yaw angle
float yaw_rate = 0;               // Current yaw rate
float yaw_des = 0;                // Desired yaw angle
float torque = 0;                 // Computed torque
float cfx = 0;                    // Computed force in x direction
float cfy = 0;                    // Computed force in y direction

// Rangefinder variables
float left_range = 80.0;  // Left sensor range (initialize to max)
float right_range = 80.0; // Right sensor range (initialize to max)
float rear_range = 80.0;  // Rear sensor range (initialize to max)

// IMU variables (reused from previous projects)
float accel_x = 0;
float accel_y = 0;
float accel_z = 0;
float gyro_x = 0;
float gyro_y = 0;
float gyro_z = 0;
float mag_x = 0;
float mag_y = 0;
float mag_z = 0;

// Time tracking variables
unsigned long current_time = 0;
unsigned long last_time = 0;
unsigned long last_range_time = 0;
unsigned long last_pd_time = 0;
unsigned long last_imu_time = 0;

// Project 6 FSM states
enum {
  STATE_IDLE,
  STATE_RUNNING
};

unsigned char state = STATE_IDLE; // Initial state

void setup() {
  Serial.begin(115200);  // Higher baud rate for faster serial communication
  delay(500);
  Serial.println("Hovercraft control with obstacle avoidance starting up");

  // Initialize I2C for IMU
  Wire.begin();
 
  // Configure rangefinder pins
  pinMode(sharpPin1, INPUT);
  pinMode(sharpPin2, INPUT);
  pinMode(sharpPin3, INPUT);
 
  // Set up motor control pins
  pinMode(pwmPin1, OUTPUT);
  pinMode(NA1, OUTPUT);
  pinMode(NB1, OUTPUT);
 
  pinMode(pwmPin2, OUTPUT);
  pinMode(NA2, OUTPUT);
  pinMode(NB2, OUTPUT);
 
  pinMode(pwmPin3, OUTPUT);
  pinMode(NA3, OUTPUT);
  pinMode(NB3, OUTPUT);
 
  // Initial state - all motors off
  digitalWrite(NA1, LOW);
  digitalWrite(NB1, LOW);
  digitalWrite(NA2, LOW);
  digitalWrite(NB2, LOW);
  digitalWrite(NA3, LOW);
  digitalWrite(NB3, LOW);
  analogWrite(pwmPin1, 0);
  analogWrite(pwmPin2, 0);
  analogWrite(pwmPin3, 0);
 
  // Switch setup
  pinMode(SWITCH_PIN, INPUT);
 
  // Initialize central fan
  Serial.println("Initializing central fan...");
  fan.attach(pwmPin4);
  delay(500);
 
  // Set up fan
  fan_setup();
  Serial.println("Central fan initialized");
 
  // Initialize and test rangefinders
  Serial.println("Testing rangefinders...");
  rangefinder_step();
  Serial.println("Rangefinders initialized");
 
  // Initialize IMU (would be integrated with your IMU code)
  // init_imu();
 
  // Test motors briefly
  Serial.println("Testing motors...");
  set_motor(0, 100);
  delay(250);
  set_motor(0, 0);
 
  set_motor(1, 100);
  delay(250);
  set_motor(1, 0);
 
  set_motor(2, 100);
  delay(250);
  set_motor(2, 0);
 
  Serial.println("System ready. Waiting for switch press.");
}

void loop() {
  current_time = millis();
 
  // Run IMU update at 100Hz (10ms interval)
  if (current_time - last_imu_time > 10) {
    read_imu_data();
    last_imu_time = current_time;
  }
 
  // Run rangefinder update at specified interval
  if (current_time - last_range_time > range_interval) {
    rangefinder_step();
    last_range_time = current_time;
  }
 
  // Run PD control update at specified interval
  if (current_time - last_pd_time > 20) {  // 50Hz control loop
    pd_step();
    last_pd_time = current_time;
  }
 
  // Run FSM update at specified interval
  if (current_time - last_time > fsm_interval) {
    fsm_step();
    last_time = current_time;
  }
}

/*
 * Function: read_distance
 * Description: Calibrates an analog sensor value to a distance in cm.
 *
 * Parameters:
 * - int sensorValue: The analog sensor value
 *
 * Return:
 * - float: the distance sensed in cm
 */
float read_distance(int sensorValue) {
  // Initialize distance variable
  float distance = 0.0;

  // Calibration equation from Project 3
  distance = 19946 * pow(sensorValue, -1.155);
 
  // Clamp distance to avoid unrealistic values (8 cm to 80 cm)
  if (distance < 8) {
    distance = 8; // minimum distance
  } else if (distance > 80) {
    distance = 80; // Maximum distance
  }

  return distance;
}

/*
 * Function: rangefinder_step
 * Description: Reads all three rangefinders and updates global range variables
 *
 * Parameters: None
 * Return: None
 * External Effects: Updates left_range, right_range, and rear_range global variables
 */
void rangefinder_step() {
  // Read analog values from sensors
  int val1 = analogRead(sharpPin1); // Left sensor
  int val2 = analogRead(sharpPin2); // Right sensor
  int val3 = analogRead(sharpPin3); // Rear sensor
 
  // Convert to distances and store in global variables
  left_range = read_distance(val1);
  right_range = read_distance(val2);
  rear_range = read_distance(val3);
 
  // Debug output (can be commented out for production)
  Serial.print("Ranges: Left=");
  Serial.print(left_range);
  Serial.print(" cm, Right=");
  Serial.print(right_range);
  Serial.print(" cm, Rear=");
  Serial.print(rear_range);
  Serial.println(" cm");
}

/*
 * Function: pd_step
 * Description: Implements PD control for yaw and obstacle avoidance
 *
 * Parameters: None
 * Return: None
 * External Effects: Updates global motor values based on control calculations
 */
void pd_step() {
  // Only perform control if in RUNNING state
  if (state == STATE_RUNNING) {
    // PD control for yaw
    float yaw_error = yaw_des - yaw;
   
    // Handle wrap-around for angles (ensuring error is between -180 and 180)
    if (yaw_error > 180) {
      yaw_error -= 360;
    } else if (yaw_error < -180) {
      yaw_error += 360;
    }
   
    // Compute torque using PD controller
    torque = Kp * yaw_error - Kd * yaw_rate;
   
    // Obstacle avoidance using proportional control
    // Compute forces to avoid obstacles detected by IR sensors
   
    // X-direction force (left/right)
    // Note: We're using (80-range) to create a repulsive force that increases as objects get closer
    // Using constrain to limit range values between 0 and 80 cm
    cfx = Kpx * ((80 - constrain(rear_range, 0, 80)) -
                (80 - 0.5 * constrain(left_range, 0, 80)) -
                (80 - 0.5 * constrain(right_range, 0, 80)));
   
    // Y-direction force (forward/backward)
    // Using sqrt(3)/2 coefficient to account for sensor alignment
    cfy = Kpy * ((80 - (sqrt(3)/2) * constrain(right_range, 0, 80)) -
                (80 - (sqrt(3)/2) * constrain(left_range, 0, 80)));
   
    // Debug output
    if (millis() % 1000 < 10) { // Print debug data once per second
      Serial.print("Control: yaw=");
      Serial.print(yaw);
      Serial.print(", yaw_des=");
      Serial.print(yaw_des);
      Serial.print(", error=");
      Serial.print(yaw_error);
      Serial.print(", torque=");
      Serial.print(torque);
      Serial.print(", cfx=");
      Serial.print(cfx);
      Serial.print(", cfy=");
      Serial.println(cfy);
    }
   
    // Apply the calculated forces and torque to control the hovercraft
    set_hovercraft_forces(cfx, cfy, torque);
  }
}

/*
 * Function: fan_setup
 * Description: Initializes the central lift fan
 */
void fan_setup() {
  fan.write(CENTRAL_MIN); // Set to minimum throttle
  Serial.println("CENTRAL fan setup");
  delay(2000);
}

/*
 * Function: bound
 * Description: Constrains a value within a specified range
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
 * Function: set_motor
 * Description: Controls a single motor's direction and speed
 */
void set_motor(int motor, float val) {
  val = bound(val, -255, 255);
 
  if (motor == 0) {  // BLUE motor
    if (val > 0) {
      digitalWrite(NA1, HIGH);
      digitalWrite(NB1, LOW);
      analogWrite(pwmPin1, val);
    } else if (val < 0) {
      digitalWrite(NA1, LOW);
      digitalWrite(NB1, HIGH);
      analogWrite(pwmPin1, -val);
    } else {
      digitalWrite(NA1, LOW);
      digitalWrite(NB1, LOW);
      analogWrite(pwmPin1, 0);
    }
  } else if (motor == 1) {  // BLACK motor
    if (val > 0) {
      digitalWrite(NA2, HIGH);
      digitalWrite(NB2, LOW);
      analogWrite(pwmPin2, val);
    } else if (val < 0) {
      digitalWrite(NA2, LOW);
      digitalWrite(NB2, HIGH);
      analogWrite(pwmPin2, -val);
    } else {
      digitalWrite(NA2, LOW);
      digitalWrite(NB2, LOW);
      analogWrite(pwmPin2, 0);
    }
  } else if (motor == 2) {  // RED motor
    if (val > 0) {
      digitalWrite(NA3, HIGH);
      digitalWrite(NB3, LOW);
      analogWrite(pwmPin3, val);
    } else if (val < 0) {
      digitalWrite(NA3, LOW);
      digitalWrite(NB3, HIGH);
      analogWrite(pwmPin3, -val);
    } else {
      digitalWrite(NA3, LOW);
      digitalWrite(NB3, LOW);
      analogWrite(pwmPin3, 0);
    }
  }
}

/*
 * Function: set_motors
 * Description: Controls all three motors simultaneously
 */
void set_motors(float val[3]) {
  for (int i = 0; i < 3; ++i) {
    set_motor(i, val[i]);
  }
}

/*
 * Function: set_hovercraft_forces
 * Description: Translates desired forces into motor commands using correct inverse kinematics
 *
 * @param fx: force in x-direction (left/right)
 * @param fy: force in y-direction (forward/backward)
 * @param torque: rotational torque around z-axis
 *
 * External Effects: calls set_motors function to apply thrust to individual motors
 */
void set_hovercraft_forces(float fx, float fy, float torque) {
  // Constants
  const float R = 0.15; // Radius of effect for torque (in meters), adjust as needed
 
  // Correct inverse kinematics equations
  motors_vals[0] = (1 / (2 * cos(30 * PI / 180))) * fx +
                  (1 / (2 + 2 * sin(30 * PI / 180))) * fy +
                  (1 / (2 * R * (1 + sin(30 * PI / 180)))) * torque; // Left motor (BLACK)
                 
  motors_vals[1] = (1 / (2 * cos(30 * PI / 180))) * fx -
                  (1 / (2 + 2 * sin(30 * PI / 180))) * fy -
                  (1 / (2 * R * (1 + sin(30 * PI / 180)))) * torque; // Right motor (BLUE)
                 
  motors_vals[2] = 0 -
                  (1 / (2 + 2 * sin(30 * PI / 180))) * fy +
                  (sin(30 * PI / 180) / (R * (1 + sin(30 * PI / 180)))) * torque; // Rear motor (RED)
 
  // Scale forces to maximize motor usage
  for (int i = 0; i < 3; i++) {
    motors_vals[i] = bound(motors_vals[i] * duty, -255, 255);
  }
 
  // Debug output
  if (millis() % 2000 < 10) {  // Print debug info every 2 seconds
    Serial.print("Motor values - Left (BLACK): ");
    Serial.print(motors_vals[0]);
    Serial.print(", Right (BLUE): ");
    Serial.print(motors_vals[1]);
    Serial.print(", Rear (RED): ");
    Serial.println(motors_vals[2]);
  }
 
  // Note that motor indices are swapped in these equations compared to the set_motor() function:
  // In equations: motors_vals[0] = LEFT = BLACK motor
  // In set_motor(): motor 0 = BLUE motor, motor 1 = BLACK motor
  // Therefore, we need to swap indices when applying to motors
  float temp_motors[3];
  temp_motors[0] = motors_vals[1]; // BLUE motor (index 0 in set_motor)
  temp_motors[1] = motors_vals[0]; // BLACK motor (index 1 in set_motor)
  temp_motors[2] = motors_vals[2]; // RED motor (index 2 in set_motor)
 
  // Apply thrust to motors with correct indices
  set_motors(temp_motors);
}

/*
 * Function: stop_central_fan
 * Description: Smoothly stops the central fan
 */
void stop_central_fan() {
  for (int i = CENTRAL_duty; i >= 0; i -= 5) {
    fan.write(i);
    delay(60);
  }
 
  fan.write(0);
  delay(300);
  fan.write(0);
  delay(100);
}

/*
 * Function: start_timer
 * Description: Initializes a timer for state timing
 */
void start_timer(unsigned long duration_ms) {
  state_timer = millis();
  wait_duration = duration_ms;
}

/*
 * Function: timer_expired
 * Description: Checks if a timer has expired
 */
bool timer_expired() {
  return (millis() - state_timer) >= wait_duration;
}

/*
 * Function: fsm_step
 * Description: Implements the simplified finite state machine for Project 6
 */
void fsm_step() {
  switch (state) {
    // Idle state - waiting for switch to be pressed
    case STATE_IDLE:
      // In idle state, ensure motors and fan are off
      set_hovercraft_forces(0, 0, 0);
      fan.write(0);
     
      // Check if switch is pressed to start
      if (digitalRead(SWITCH_PIN) == LOW) {
        Serial.println("Switch pressed, starting hovercraft");
       
        // Start fan gradually
        Serial.println("Starting lift fan");
        for (int i = CENTRAL_MIN; i <= CENTRAL_duty; i += 5) {
          fan.write(i);
          delay(50);
        }
       
        // Reset yaw desired to current yaw
        yaw_des = yaw;
        Serial.print("Setting yaw reference to current heading: ");
        Serial.println(yaw_des);
       
        // Transition to running state
        state = STATE_RUNNING;
      }
      break;
     
    // Running state - active obstacle avoidance and navigation
    case STATE_RUNNING:
      // Check if switch is pressed to stop
      if (digitalRead(SWITCH_PIN) == LOW) {
        Serial.println("Switch pressed, stopping hovercraft");
       
        // Stop motors first
        set_hovercraft_forces(0, 0, 0);
       
        // Then stop fan gradually
        Serial.println("Stopping lift fan");
        stop_central_fan();
       
        // Transition back to idle state
        state = STATE_IDLE;
      }
     
      // Check for IMU calibration issues
      if (system_cal < 1) {
        Serial.println("WARNING: System not calibrated! Wave the hovercraft in a figure-8 pattern.");
      }
     
      // Get new yaw reference if user presses alternate button (not implemented here)
      // This would be where you'd implement additional control inputs if needed
     
      break;
  }
}
