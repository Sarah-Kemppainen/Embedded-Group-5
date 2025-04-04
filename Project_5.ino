//Project5
//Group5
//Angel Alcantara, Brandon Aparicio, , Diego Perez, Sarah Kemppainen:
// April, 04, 2025

#include <PWMServo.h>
#include <Arduino.h>

PWMServo fan; // create servo object to control the fan

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

#define SWITCH_PIN 31
int switchState = 0;

// Central fan settings - MODIFIED FOR HIGHER POWER
const int CENTRAL_MIN = 20;      // Minimum value for startup
const int CENTRAL_duty = 90;    // Increased value for better power (~47% duty)

// Motor power settings - INCREASED FOR BETTER RESPONSE
const int fsm_interval = 250;    // Faster FSM updates
const int duty = 200;            // INCREASED from 168 to 240 for more power (94% of max)

float motors_vals[3] = {0,0,0}; // Declare float to store motor values; set as empty

// Set possible states
enum {
  STATE_WAIT,
  STATE_START_LIFT_1,
  STATE_WAIT_LIFT_1,
  STATE_POS_TORQUE,
  STATE_NEG_TORQUE,
  STATE_STOP_LIFT_1,
  STATE_WAIT_OFF_1,
  STATE_START_LIFT_2,
  STATE_WAIT_LIFT_2,
  STATE_FORWARD,
  STATE_BACKWARD,
  STATE_STOP_LIFT_2,
  STATE_WAIT_OFF_2,
  STATE_START_LIFT_3,
  STATE_WAIT_LIFT_3,
  STATE_RIGHT,
  STATE_LEFT,
  STATE_STOP_LIFT_3
};

unsigned char state = STATE_WAIT; // Initialize state to WAIT

// Time tracking
unsigned long last_time = 0;
unsigned long state_timer = 0;
unsigned long wait_duration = 0;

void setup() {
  Serial.begin(9600); // Initialize serial communication for debugging
  delay(500);         // Add delay to let serial initialize properly
  Serial.println("Hovercraft control starting up");

  // Configure motor pins
  pinMode(pwmPin1, OUTPUT);
  pinMode(NA1, OUTPUT);
  pinMode(NB1, OUTPUT);

  pinMode(pwmPin2, OUTPUT);
  pinMode(NA2, OUTPUT);
  pinMode(NB2, OUTPUT);

  pinMode(pwmPin3, OUTPUT);
  pinMode(NA3, OUTPUT);
  pinMode(NB3, OUTPUT);

  // Initial state of motors - make sure they're OFF
  digitalWrite(NA1, LOW);
  digitalWrite(NB1, LOW);
  digitalWrite(NA2, LOW);
  digitalWrite(NB2, LOW);
  digitalWrite(NA3, LOW);
  digitalWrite(NB3, LOW);
  analogWrite(pwmPin1, 0);
  analogWrite(pwmPin2, 0);
  analogWrite(pwmPin3, 0);

  pinMode(SWITCH_PIN, INPUT);
 
  // Initialize the central fan
  Serial.println("Initializing central fan...");
  //fan.attach(pwmPin4);    // Attach servo object to central fan pin
  //delay(500);             // Allow time for fan to initialize
 
  // Fan initialization sequence - helps with ESC calibration
  fan_setup();
  Serial.println("Central fan initialized");
 
  // Test each motor briefly to verify connections
  Serial.println("Testing motors...");
  // Left motor
  set_motor(0, 32);
  delay(250);
  set_motor(0, 0);
 
  // Right motor
  set_motor(1, 32);
  delay(250);
  set_motor(1, 0);
 
  // Rear motor
  set_motor(2, 32);
  delay(250);
  set_motor(2, 0);
 
  Serial.println("Testing completed.");
  Serial.println("System ready. Waiting for switch press.");
}

void loop() {
  // Call the FSM step function at regular intervals
  unsigned long current_time = millis();
  if (current_time - last_time > fsm_interval) {
    fsm_step();
    last_time = current_time;
  }
}

// Function to set up the fan with a low throttle
//
// @param NONE
// @return NONE
// External Effects: Ensures that the fan is set up on a low 
// throttle so that it properly turns on when called.
void fan_setup() {
    fan.attach(pwmPin4); // attaches the fan to specified Arduino pin
    delay(100);
    fan.write(CENTRAL_MIN); // write low throttle
    Serial.printf("CENTRAL fan setup\n");
    delay(3000);
}

 //* Function to constrain a value within a specified range.
 //*
 //* @param value: The value to check and bound.
 //* @param min_value: The minimum allowable value.
 //* @param max_value: The maximum allowable value.
 //*
 //* @return: The bounded value, ensuring it stays within [min_value, max_value].
 //* This function serves to protect against out-of-range values.
float bound(float value, float min_value, float max_value) {
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

// Function to set thrust and direction for a single motor
 //* Function to set thrust and direction for a single motor.
 //* @param motor: Index of the motor to control (0 = BLUE, 1 = BLACK, 2 = RED).
 //* @param val: The PWM value for controlling the selected motor.
// *             Positive values drive the motor forward, negative values drive it backward.
 
 //* External Effects: Modifies the state of the control pins (NA and NB)
 //* for the specified motor, influencing its direction and speed through PWM.
void set_motor(int motor, float val) {
  val = bound(val, -255, 255);  // Using full 8-bit range for more power
 
  if (motor == 0) {
    if (val > 0) {
      digitalWrite(NA1, HIGH);  // Set direction for forward
      digitalWrite(NB1, LOW);   // Ensure opposite control is low
      analogWrite(pwmPin1, val); // Set PWM value
    } else if (val < 0) {
      digitalWrite(NA1, LOW);   // Set direction for backward
      digitalWrite(NB1, HIGH);  // Ensure opposite control is high
      analogWrite(pwmPin1, -val); // Set PWM value (use -val as value cannot be negative)
    } else {
      // For val == 0, ensure motor is stopped
      digitalWrite(NA1, LOW);
      digitalWrite(NB1, LOW);
      analogWrite(pwmPin1, 0);
    }
  } else if (motor == 1) {
    if (val > 0) {
      digitalWrite(NA2, HIGH);  // Set direction for forward
      digitalWrite(NB2, LOW);   // Ensure opposite control is low
      analogWrite(pwmPin2, val); // Set PWM value
    } else if (val < 0) {
      digitalWrite(NA2, LOW);   // Set direction for backward
      digitalWrite(NB2, HIGH);  // Ensure opposite control is high
      analogWrite(pwmPin2, -val); // Set PWM value (use -val as value cannot be negative)
    } else {
      // For val == 0, ensure motor is stopped
      digitalWrite(NA2, LOW); 
      digitalWrite(NB2, LOW); 
      analogWrite(pwmPin2, 0); 
    }
  } else if (motor == 2) {
    if (val > 0) {
      digitalWrite(NA3, HIGH);  // Set direction for forward
      digitalWrite(NB3, LOW);   // Ensure opposite control is low
      analogWrite(pwmPin3, val); // Set PWM value
    } else if (val < 0) {
      digitalWrite(NA3, LOW);   // Set direction for backward
      digitalWrite(NB3, HIGH);  // Ensure opposite control is high
      analogWrite(pwmPin3, -val); // Set PWM value (use -val as value cannot be negative)
    } else {
      // For val == 0, ensure motor is stopped
      digitalWrite(NA3, LOW);
      digitalWrite(NB3, LOW);
      analogWrite(pwmPin3, 0);
    }
  }
}

// Function to set thrust and direction for multiple motors

 //* Function to set thrust and direction for a single motor.
 //* @param motor: Index of the motor to control (0 = BLUE, 1 = BLACK, 2 = RED).
 //* @param val: The PWM value for controlling the selected motor.
// *             Positive values drive the motor forward, negative values drive it backward.
 
 //* External Effects: Modifies the state of the control pins (NA and NB)
 //* for the specified motor, influencing its direction and speed through PWM.
void set_motors(float val[3]) {
  for (int i = 0; i < 3; ++i) {
    set_motor(i, val[i]);
  }
}

// Function to set hovercraft forces
//
// @param fx: unit value variable that indicates whether its moving in the x-dir
//        fy: unit value variable that indicates whether its moving in the y-dir
//        torque: unit value that indicates whether the hovercraft is rotating
// @return NONE
// External Effects: calls set_motors function to turn on side motors
void set_hovercraft_forces(float fx, float fy, float torque) {
  // Translate forces into thrust for motors
  motors_vals[0] = fy + torque;  // Left motor (BLUE)
  motors_vals[1] = fy - torque;  // Right motor (BLACK)
  motors_vals[2] = -fx;          // Rear motor (RED)
 
  // Scale forces to maximize motor usage
  for (int i = 0; i < 3; i++) {
    motors_vals[i] = bound(motors_vals[i] * duty, -255, 255);  // Using full 8-bit range
  }
 
  set_motors(motors_vals); // Apply thrust
}

// Function to stop the central fan
//
// @param NONE
// @return NONE
// External Effects: stops the central fan
void stop_central_fan() {
  // More gradual fan stop for smoother operation
  for (int i = CENTRAL_duty; i >= 0; i -= 5) {  // CHANGED: smaller steps
    fan.write(i);
    delay(60);  // CHANGED: slightly longer delay between steps
  }
 
  // Make sure fan stops completely with multiple zero commands
  fan.write(0);
  delay(300);
  fan.write(0);  // Second command to ensure stop
  delay(100);
}

// Function to start a timed wait period
void start_timer(unsigned long duration_ms) {
  state_timer = millis();
  wait_duration = duration_ms;
}

// Function to check if timer has expired
bool timer_expired() {
  return (millis() - state_timer) >= wait_duration;
}

// FSM logic implementation
void fsm_step() {
  switch (state) {
    // Wait for switch to be pressed
    case STATE_WAIT:
      set_hovercraft_forces(0, 0, 0); // Ensure all motors are off
      fan.write(0); // Ensure fan is off
     
      // If switch is pressed, change state
      if (digitalRead(SWITCH_PIN) == LOW) {
        Serial.println("Switch pressed, starting sequence");
        state = STATE_START_LIFT_1;
      }
      break;

    // 2. Set the central fan to lift the craft
    case STATE_START_LIFT_1:
      Serial.print("Starting lift fan (first time): ");
      Serial.println(CENTRAL_duty);
     
      // Progressive fan start for smoother operation
      for (int i = CENTRAL_MIN; i <= CENTRAL_duty; i += 5) {
        fan.write(i);
        delay(50);
      }
     
      start_timer(15000); // Wait for 15 seconds
      state = STATE_WAIT_LIFT_1;
      break;

    // 3. Wait for 15 seconds with fan on
    case STATE_WAIT_LIFT_1:
      if (!timer_expired()) {
        if (millis() % 2000 < 10) {  // Print status every 2 seconds
          Serial.print("Waiting with lift fan on (first time): ");
          Serial.print((millis() - state_timer) / 1000);
          Serial.println("s elapsed");
        }
      } else {
        Serial.println("15s elapsed, moving to positive torque state");
        start_timer(10000); // Set timer for next state
        state = STATE_POS_TORQUE;
      }
      break;

    // 4. Generate positive torque for 10 seconds
    case STATE_POS_TORQUE:
      set_hovercraft_forces(0, 0, 1.0);
     
      if (!timer_expired()) {
        if (millis() % 2000 < 10) {  // Print status every 2 seconds
          Serial.print("Applying positive torque: ");
          Serial.print((millis() - state_timer) / 1000);
          Serial.println("s elapsed");
        }
      } else {
        Serial.println("10s of positive torque elapsed, moving to negative torque");
        start_timer(10000); // Set timer for next state
        state = STATE_NEG_TORQUE;
      }
      break;
   
    // 5. Generate negative torque for 10 seconds
    case STATE_NEG_TORQUE:
      set_hovercraft_forces(0, 0, -1.0);
     
      if (!timer_expired()) {
        if (millis() % 2000 < 10) {  // Print status every 2 seconds
          Serial.print("Applying negative torque: ");
          Serial.print((millis() - state_timer) / 1000);
          Serial.println("s elapsed");
        }
      } else {
        Serial.println("10s of negative torque elapsed, stopping lift fan");
        state = STATE_STOP_LIFT_1;
      }
      break;

    // 6. Turn off the central fan
    case STATE_STOP_LIFT_1:
      Serial.println("Stopping lift fan (first time)");
      set_hovercraft_forces(0, 0, 0);  // Stop thrust motors first
     
      // Use the improved fan stop function
      stop_central_fan();
     
      start_timer(15000); // Wait for 15 seconds
      state = STATE_WAIT_OFF_1;
      break;

    // 7. Wait for 15 seconds with fan off
    case STATE_WAIT_OFF_1:
      if (!timer_expired()) {
        if (millis() % 2000 < 10) {  // Print status every 2 seconds
          Serial.print("Waiting with fan off (first time): ");
          Serial.print((millis() - state_timer) / 1000);
          Serial.println("s elapsed");
         
          // ADDED: Double-check fan is still off
          fan.write(0);
        }
      } else {
        Serial.println("15s with fan off elapsed, starting lift fan again");
        state = STATE_START_LIFT_2;
      }
      break;

    // 8. Turn on the central fan again
    case STATE_START_LIFT_2:
      Serial.println("Starting lift fan (second time)");
     
      // Progressive fan start for smoother operation
      for (int i = CENTRAL_MIN; i <= CENTRAL_duty; i += 5) {
        fan.write(i);
        delay(50);
      }
     
      start_timer(10000); // Wait for 10 seconds
      state = STATE_WAIT_LIFT_2;
      break;

    // 9. Wait for 10 seconds with fan on
    case STATE_WAIT_LIFT_2:
      if (!timer_expired()) {
        if (millis() % 2000 < 10) {  // Print status every 2 seconds
          Serial.print("Waiting with lift fan on (second time): ");
          Serial.print((millis() - state_timer) / 1000);
          Serial.println("s elapsed");
        }
      } else {
        Serial.println("10s with fan on elapsed, moving forward");
        start_timer(10000); // Set timer for next state
        state = STATE_FORWARD;
      }
      break;

    // 10. Generate a forward force for 10 seconds
    case STATE_FORWARD:
      set_hovercraft_forces(0, 1.0, 0);
     
      if (!timer_expired()) {
        if (millis() % 2000 < 10) {  // Print status every 2 seconds
          Serial.print("Moving forward: ");
          Serial.print((millis() - state_timer) / 1000);
          Serial.println("s elapsed");
        }
      } else {
        Serial.println("10s moving forward elapsed, moving backward");
        start_timer(10000); // Set timer for next state
        state = STATE_BACKWARD;
      }
      break;

    // 11. Generate a backward force for 10 seconds
    case STATE_BACKWARD:
      set_hovercraft_forces(0, -1.0, 0);
     
      if (!timer_expired()) {
        if (millis() % 2000 < 10) {  // Print status every 2 seconds
          Serial.print("Moving backward: ");
          Serial.print((millis() - state_timer) / 1000);
          Serial.println("s elapsed");
        }
      } else {
        Serial.println("10s moving backward elapsed, stopping lift fan");
        state = STATE_STOP_LIFT_2;
      }
      break;

    // 12. Turn off the central fan
    case STATE_STOP_LIFT_2:
      Serial.println("Stopping lift fan (second time)");
      set_hovercraft_forces(0, 0, 0);
     
      // Use the improved fan stop function
      stop_central_fan();
     
      start_timer(15000); // Wait for 15 seconds
      state = STATE_WAIT_OFF_2;
      break;

    // 13. Wait for 15 seconds with fan off
    case STATE_WAIT_OFF_2:
      if (!timer_expired()) {
        if (millis() % 2000 < 10) {  // Print status every 2 seconds
          Serial.print("Waiting with fan off (second time): ");
          Serial.print((millis() - state_timer) / 1000);
          Serial.println("s elapsed");
         
          // ADDED: Double-check fan is still off
          fan.write(0);
        }
      } else {
        Serial.println("15s with fan off elapsed, starting lift fan again");
        state = STATE_START_LIFT_3;
      }
      break;

    // 14. Turn on the central fan for the third time
    case STATE_START_LIFT_3:
      Serial.println("Starting lift fan (third time)");
     
      // Progressive fan start for smoother operation
      for (int i = CENTRAL_MIN; i <= CENTRAL_duty; i += 5) {
        fan.write(i);
        delay(50);
      }
     
      start_timer(10000); // Wait for 10 seconds
      state = STATE_WAIT_LIFT_3;
      break;

    // 15. Wait for 10 seconds with fan on
    case STATE_WAIT_LIFT_3:
      if (!timer_expired()) {
        if (millis() % 2000 < 10) {  // Print status every 2 seconds
          Serial.print("Waiting with lift fan on (third time): ");
          Serial.print((millis() - state_timer) / 1000);
          Serial.println("s elapsed");
        }
      } else {
        Serial.println("10s with fan on elapsed, moving right");
        start_timer(10000); // Set timer for next state
        state = STATE_RIGHT;
      }
      break;

    // 16. Generate a rightward force for 10 seconds
    case STATE_RIGHT:
      set_hovercraft_forces(1.0, 0, 0);
     
      if (!timer_expired()) {
        if (millis() % 2000 < 10) {  // Print status every 2 seconds
          Serial.print("Moving right: ");
          Serial.print((millis() - state_timer) / 1000);
          Serial.println("s elapsed");
        }
      } else {
        Serial.println("10s moving right elapsed, moving left");
        start_timer(10000); // Set timer for next state
        state = STATE_LEFT;
      }
      break;

    // 17. Generate a leftward force for 10 seconds
    case STATE_LEFT:
      set_hovercraft_forces(0, 1, 0);
     
      if (!timer_expired()) {
        if (millis() % 2000 < 10) {  // Print status every 2 seconds
          Serial.print("Moving left: ");
          Serial.print((millis() - state_timer) / 1000);
          Serial.println("s elapsed");
        }
      } else {
        Serial.println("10s moving left elapsed, stopping lift fan");
        state = STATE_STOP_LIFT_3;
      }
      break;

    // 18. Turn off the central fan
    case STATE_STOP_LIFT_3:
      Serial.println("Stopping lift fan (third time)");
      set_hovercraft_forces(0, 0, 0);
     
      // Use the improved fan stop function
      stop_central_fan();
     
      // 19. Return to waiting for the switch
      state = STATE_WAIT;
      break;
  }
}
