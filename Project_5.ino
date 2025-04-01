#include <PWMServo.h>

#include <Arduino.h>

// Project4
// Group5
// Angel Alcantara, Brandon Aparicio, Diego Perez, Sarah Kemppainen:
// March 24, 2025

PWMServo fan; // create servo object to control the fan

// Motor control pins
const int pwmPin1 = 23; // BLUE motor
const int pwmPin2 = 22; // BLACK motor
const int pwmPin3 = 38; // RED motor
const int pwmPin4 = 10; // CENTRAL fan

const int NA1 = 13; // BLUE motor
const int NB1 = 39; // BLUE motor
const int NA2 = 36; // BLACK motor
const int NB2 = 35; // BLACK motor
const int NA3 = 9;  // RED motor
const int NB3 = 8;  // RED motor

#define SWITCH_PIN 31


int switchState = 0;

const int fsm_interval = 50; // FSM interval in milliseconds
const int duty = 128; // Duty cycle for central fan (50%)

float motors_vals[3] = {0, 0, 0}; // Declare float to store motor values; set as empty

// Set possible states
enum {
    STATE_WAIT,
    STATE_START_LIFT,
    STATE_LIFT,
    STATE_TORQUE_POS,
    STATE_TORQUE_NEG,
    STATE_STOP_LIFT,
    STATE_WAIT_SHORT,
    STATE_START_FORWARD,
    STATE_FORWARD,
    STATE_BACKWARD,
    STATE_TURN_RIGHT,
    STATE_TURN_LEFT,
    STATE_DONE
};
unsigned char state = STATE_WAIT; // Initialize state to WAIT

// Time tracking variables
unsigned long last_time = 0;
unsigned long state_start_time = 0;
unsigned long state_duration = 0;

void fan_setup() {
    fan.attach(pwmPin4); // attaches the fan to specified Arduino pin
    delay(100);
    fan.write(20); // write low throttle
    Serial.printf("*************fan_setup called. fan written to 20*******\n");
    delay(3000);
}

void setup() {
    Serial.begin(9600); // Initialize serial communication for debugging

    pinMode(pwmPin1, OUTPUT);
    pinMode(NA1, OUTPUT);
    pinMode(NB1, OUTPUT);
   
    pinMode(pwmPin2, OUTPUT);
    pinMode(NA2, OUTPUT);
    pinMode(NB2, OUTPUT);
   
    pinMode(pwmPin3, OUTPUT);
    pinMode(NA3, OUTPUT);
    pinMode(NB3, OUTPUT);
   
    pinMode(SWITCH_PIN, INPUT);

    fan_setup(); // Setup the fan
}

void loop() {
    // Call the FSM step function at regular intervals
    unsigned long current_time = millis();
    if (current_time - last_time > fsm_interval) {
        fsm_step();
        last_time = current_time;
    }
}

// Function to constrain a value within a specified range
float bound(float value, float min_value, float max_value) {
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

// Function to set thrust and direction for a single motor
void set_motor(int motor, float val) {
    if (val < -64 || val > 64) {
        val = bound(val, -64, 64);
    }

    // Motor control logic remains unchanged
    if (motor == 0) {
        if (val > 0) {
            digitalWrite(NA1, HIGH); // Forward
            digitalWrite(NB1, LOW);
            analogWrite(pwmPin1, val);
        } else {
            digitalWrite(NA1, LOW); // Backward
            digitalWrite(NB1, HIGH);
            analogWrite(pwmPin1, -val);
        }
    } else if (motor == 1) {
        if (val > 0) {
            digitalWrite(NA2, HIGH); // Forward
            digitalWrite(NB2, LOW);
            analogWrite(pwmPin2, val);
        } else {
            digitalWrite(NA2, LOW); // Backward
            digitalWrite(NB2, HIGH);
            analogWrite(pwmPin2, -val);
        }
    } else if (motor == 2) {
        if (val > 0) {
            digitalWrite(NA3, HIGH); // Forward
            digitalWrite(NB3, LOW);
            analogWrite(pwmPin3, val);
        } else {
            digitalWrite(NA3, LOW); // Backward
            digitalWrite(NB3, HIGH);
            analogWrite(pwmPin3, -val);
        }
    }
}

// Function to set thrust and direction for multiple motors
void set_motors(float val[3]) {
    const float negative_gain[3] = {1.0, 1.0, 1.0};
    for (int i = 0; i < 3; ++i) {
        if (val[i] < 0) {
            val[i] *= negative_gain[i];
        }
        val[i] = bound(val[i], -64, 64);
        set_motor(i, val[i]);
    }
}

// Function to set hovercraft forces
void set_hovercraft_forces(float fx, float fy, float torque) {
    // Translate forces into thrust for motors (you will define your own logic here)
    motors_vals[0] = fy + torque; // Example logic for left motor
    motors_vals[1] = fy - torque; // Example logic for right motor
    motors_vals[2] = -fx;          // Example logic for rear motor
    set_motors(motors_vals); // Apply thrust
}

// Non-blocking state transition with timing
void enter_state(unsigned char new_state, unsigned long duration) {
    state = new_state;
    state_start_time = millis();
    state_duration = duration;
   
    // Actions to take upon entering the state
    switch(state) {
        case STATE_START_LIFT:
            Serial.printf("current state: STATE_START_LIFT\n");
            fan.write(duty); // Start the central fan
            break;
           
        case STATE_TORQUE_POS:
            Serial.printf("current state: STATE_TORQUE_POS\n");
            set_hovercraft_forces(0, 0, 1.0); // Apply positive torque
            break;
           
        case STATE_TORQUE_NEG:
            Serial.printf("current state: STATE_TORQUE_NEG\n");
            set_hovercraft_forces(0, 0, -1.0); // Apply negative torque
            break;
           
        case STATE_STOP_LIFT:
            Serial.printf("current state: STATE_STOP_LIFT\n");
            fan.write(0); // Turn off the central fan
            set_hovercraft_forces(0, 0, 0); // Stop all motors
            break;
           
        case STATE_START_FORWARD:
            Serial.printf("current state: STATE_START_FORWARD\n");
            fan.write(duty); // Turn on the fan again
            break;
           
        case STATE_FORWARD:
            Serial.printf("current state: STATE_FORWARD\n");
            set_hovercraft_forces(1.0, 0, 0); // Apply forward force
            break;
           
        case STATE_BACKWARD:
            Serial.printf("current state: STATE_BACKWARD\n");
            set_hovercraft_forces(-1.0, 0, 0); // Apply backward force
            break;
           
        case STATE_TURN_RIGHT:
            Serial.printf("current state: STATE_TURN_RIGHT\n");
            set_hovercraft_forces(0, 0, 1.0); // Apply right torque
            break;
           
        case STATE_TURN_LEFT:
            Serial.printf("current state: STATE_TURN_LEFT\n");
            set_hovercraft_forces(0, 0, -1.0); // Apply left torque
            break;
           
        case STATE_DONE:
            Serial.printf("current state: STATE_DONE\n");
            fan.write(0); // Ensure central fan is off
            set_hovercraft_forces(0, 0, 0); // Stop all motors
            break;
           
        default:
            Serial.printf("current state: %d\n", state);
            break;
    }
}

// FSM logic implementation (non-blocking)
void fsm_step() {
    unsigned long current_time = millis();
   
    switch (state) {
        case STATE_WAIT:
            //Serial.printf("current state: STATE_WAIT\n");
            switchState = digitalRead(SWITCH_PIN);
            //Serial.printf("Switch State: %d\n", switchState);
            // If switch is pressed, transition to lift state
            if (switchState == LOW) {
                enter_state(STATE_START_LIFT, 0);
            }
            break;

        case STATE_START_LIFT:
            // Transition after entering the state immediately
            enter_state(STATE_LIFT, 15000);
            break;

        case STATE_LIFT:
            // Check if state duration has elapsed
            if (current_time - state_start_time >= state_duration) {
                enter_state(STATE_TORQUE_POS, 10000);
            }
            break;

        case STATE_TORQUE_POS:
            if (current_time - state_start_time >= state_duration) {
                enter_state(STATE_TORQUE_NEG, 10000);
            }
            break;

        case STATE_TORQUE_NEG:
            if (current_time - state_start_time >= state_duration) {
                enter_state(STATE_STOP_LIFT, 0);
            }
            break;

        case STATE_STOP_LIFT:
            // Transition after stopping lift
            enter_state(STATE_WAIT_SHORT, 15000);
            break;

        case STATE_WAIT_SHORT:
            if (current_time - state_start_time >= state_duration) {
                enter_state(STATE_START_FORWARD, 10000);
            }
            break;

        case STATE_START_FORWARD:
            if (current_time - state_start_time >= state_duration) {
                enter_state(STATE_FORWARD, 10000);
            }
            break;

        case STATE_FORWARD:
            if (current_time - state_start_time >= state_duration) {
                enter_state(STATE_BACKWARD, 10000);
            }
            break;

        case STATE_BACKWARD:
            if (current_time - state_start_time >= state_duration) {
                enter_state(STATE_TURN_RIGHT, 10000);  // Added transition to turn right
            }
            break;

        case STATE_TURN_RIGHT:
            if (current_time - state_start_time >= state_duration) {
                enter_state(STATE_TURN_LEFT, 10000);
            }
            break;

        case STATE_TURN_LEFT:
            if (current_time - state_start_time >= state_duration) {
                enter_state(STATE_DONE, 5000);  // Transition to DONE state
            }
            break;

        case STATE_DONE:
            if (current_time - state_start_time >= state_duration) {
                enter_state(STATE_WAIT, 0);  // Return to WAIT state
            }
            break;
    }
}
