#include <Arduino.h>

// Motor control pins
const int pwmPin1 = 23; // BLUE motor
const int pwmPin2 = 22; // BLACK motor
const int pwmPin3 = 38; // RED motor

const int NA1 = 13; // BLUE motor
const int NB1 = 39; // BLUE motor
const int NA2 = 36; // BLACK motor
const int NB2 = 35; // BLACK motor
const int NA3 = 9;  // RED motor
const int NB3 = 8;  // RED motor

#define SWITCH_PIN 31
int switchState = 0;

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

  pinMode(pwmPin1, OUTPUT);
  pinMode(NA1, OUTPUT); // Set NA1 as output
  pinMode(NB1, OUTPUT); // Set NB1 as output

  pinMode(pwmPin2, OUTPUT);
  pinMode(NA2, OUTPUT); // Set NA2 as output
  pinMode(NB2, OUTPUT); // Set NB2 as output

  pinMode(pwmPin3, OUTPUT); // Set the third motor pin as output
  pinMode(NA3, OUTPUT); // Set NA2 as output
  pinMode(NB3, OUTPUT); // Set NB2 as output

  pinMode(SWITCH_PIN, INPUT);
}

void loop() {
  // Call the FSM step function at regular intervals
  unsigned long current_time = millis();
  if (current_time - last_time > fsm_interval) {
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

  if (motor == 0) {
    if (val > 0) {
      digitalWrite(NA1, HIGH); // Set direction for one motor forward
      digitalWrite(NB1, LOW);   // Ensure the opposite control is low
      analogWrite(pwmPin1, val); // Set PWM value
    } else {
      digitalWrite(NA1, LOW);   // Set direction for one motor backward
      digitalWrite(NB1, HIGH);  // Ensure the opposite control is high
      analogWrite(pwmPin1, -val); // Set PWM value (use -val as value cannot be negative)
    }
  } else if (motor == 1) {
    if (val > 0) {
      digitalWrite(NA2, HIGH); // Set direction for one motor forward
      digitalWrite(NB2, LOW);   // Ensure the opposite control is low
      analogWrite(pwmPin2, val); // Set PWM value
    } else {
      digitalWrite(NA2, LOW);   // Set direction for one motor backward
      digitalWrite(NB2, HIGH);  // Ensure the opposite control is high
      analogWrite(pwmPin2, -val); // Set PWM value (use -val as value cannot be negative)
    }
  } else if (motor == 2) {
    if (val > 0) {
      digitalWrite(NA3, HIGH); // Set direction for one motor forward
      digitalWrite(NB3, LOW);   // Ensure the opposite control is low
      analogWrite(pwmPin3, val); // Set PWM value
    } else {
      digitalWrite(NA3, LOW);   // Set direction for one motor backward
      digitalWrite(NB3, HIGH);  // Ensure the opposite control is high
      analogWrite(pwmPin3, -val); // Set PWM value (use -val as value cannot be negative)
    }
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
    val[i] = bound(val[i], -64, 64);    // Should this only be called if out of bounds?
    set_motor(i, val[i]);
  }
}

// FSM logic implementation
void fsm_step() {
  switch (state) {
    case STATE_WAIT:
      Serial.printf("current state: STATE_WAIT\n");
      switchState = digitalRead(SWITCH_PIN);
      Serial.printf("Switch State: %d\n", switchState);
      // If switch is pressed, change state to LEFT25
      if (digitalRead(SWITCH_PIN) == LOW) {
        Serial.printf(".....high digital read from switch pin. Setting state to STATE_LEFT25\n");
        state = STATE_LEFT25;
      }
      break;

    case STATE_LEFT25:
      Serial.printf("current state: STATE_LEFT25\n");
      set_motor(0, 64); // Ramp left motor to 25%
      Serial.printf(".....set motor 0 (left/BLUE) to 64\n");
      Serial.printf(".....setting state to STATE_RIGHT25\n");
      state = STATE_RIGHT25; // Transition to the next state
      break;

    case STATE_RIGHT25:
      Serial.printf("current state: STATE_RIGHT25\n");
      set_motor(1, 64); // Ramp right motor to 25%
      Serial.printf(".....set motor 1 (right/RED) to 64\n");
      Serial.printf(".....setting state to STATE_REAR25\n");
      state = STATE_REAR25; // Transition to the next state
      break;

    case STATE_REAR25:
      Serial.printf("current state: STATE_REAR25\n");
      set_motor(2, 64); // Ramp rear motor to 25%
      Serial.printf(".....set motor 2 (rear/BLACK) to 64\n");
      Serial.printf(".....setting state to STATE_ALL25\n");
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