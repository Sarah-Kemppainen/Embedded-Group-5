#include <Arduino.h>

// Motor control pins
const int pmwPin1 = 18;
const int pmwPin2 = 15;
const int pmwPin3 = 37; // New motor pin for the rear motor

#define SWITCH_PIN 31

const int fsm_interval = 50; // FSM interval in milliseconds
const int duty = 64; // Duty cycle for motors (0 - 64 corresponds to 25%)

float motors_vals[3] = {0,0,0}; // Declare float to store motor values; set as empty

// Set possible states
enum {STATE_WAIT, STATE_LEFT25, STATE_RIGHT25, STATE_REAR25, STATE_ALL25, STATE_LEFT0, STATE_RIGHT0, STATE_REAR0, STATE_START};
unsigned char state = STATE_WAIT; // Initialize state to WAIT

// Time tracking
unsigned long last_time = 0;

void setup() {
  Serial.begin(9600); // Initialize serial communication for debugging
  pinMode(pmwPin1, OUTPUT);
  pinMode(pmwPin2, OUTPUT);
  pinMode(pmwPin3, OUTPUT); // Set the third motor pin as output
  pinMode(SWITCH_PIN, INPUT);
}

void loop() {
  // Call the FSM step function at regular intervals
  unsigned long current_time = millis();
  if (current_time - last_time > fsm_interval) {
    Serial.printf("calling fsm_step()........\n");
    fsm_step();
    last_time = current_time;
  }
}

// Function to bound a value within a specified range
float bound(float value, float min_value, float max_value) {
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  Serial.printf("bound function ran. Returning bounded value.......\n");
  return value;
}

// Function to set thrust and direction for a single motor
void set_motor(int motor, float val) {
  if (val < -64 || val > 64) {
    Serial.printf("val %f is out of range [-64, 64]. Calling bound.\n", val);
    val = bound(val, -64, 64);
  }

  // Map the thrust value to PWM range (0 to 64)
  Serial.printf("mapping the thrust value to PWM range (0 to 64).....\n");
  int pwm_value = (int)map(val, -64, 64, 0, 64);
  if (motor == 0) {
    analogWrite(pmwPin1, pwm_value); // Left motor
  } else if (motor == 1) {
    analogWrite(pmwPin2, pwm_value); // Right motor
  } else if (motor == 2) {
    analogWrite(pmwPin3, pwm_value); // Rear motor
  }
}

// Function to set thrust and direction for multiple motors
void set_motors(float val[3]) {
  const float negative_gain[3] = {1.0, 1.0, 1.0};

  Serial.printf("set_motors function called. Setting thrust and diraction for all motors.....\n");
  for (int i = 0; i < 3; ++i) {
    if (val[i] < 0) {
      val[i] *= negative_gain[i];
    }
    val[i] = bound(val[i], -64, 64);
    set_motor(i, val[i]);
  }
}

// FSM logic implementation
void fsm_step() {
  switch (state) {
    case STATE_WAIT:
      // If switch is pressed, change state to LEFT25
      if (digitalRead(SWITCH_PIN) == HIGH) {
        state = STATE_LEFT25;
      }
      break;

    case STATE_LEFT25:
      set_motor(0, 64); // Ramp left motor to 25%
      state = STATE_RIGHT25; // Transition to the next state
      break;

    case STATE_RIGHT25:
      set_motor(1, 64); // Ramp right motor to 25%
      state = STATE_REAR25; // Transition to the next state
      break;

    case STATE_REAR25:
      set_motor(2, 64); // Ramp rear motor to 25%
      state = STATE_ALL25; // Transition to the next state
      break;

    case STATE_ALL25:
      for (int i=0; i<3; i++) {
        motors_vals[i] = -64;
      }
     
      set_motors(motors_vals); // Ramp all motors down
      state = STATE_LEFT0; // Transition to the next state
      break;

    case STATE_LEFT0:
      set_motor(0, 0); // Ramp left motor to 0%
      state = STATE_RIGHT0; // Transition to the next state
      break;

    case STATE_RIGHT0:
      set_motor(1, 0); // Ramp right motor to 0%
      state = STATE_REAR0; // Transition to the next state
      break;

    case STATE_REAR0:
      set_motor(2, 0); // Ramp rear motor to 0%
      state = STATE_START; // Transition to start
      break;

    case STATE_START:
      // After reaching zero, return to WAIT state
      state = STATE_WAIT;
      break;
  }
}