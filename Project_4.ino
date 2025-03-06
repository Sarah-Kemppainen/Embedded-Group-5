int fsm_interval = 50;

const int pmwPin1 = 18; 
const int pmwPin2 = 15;

#define SWITCH_PIN 31

// NA1 = 13
// NB1 = 39
// NA2 = 36 
// NB2 = 35
// switch = 31

const int duty = 64; // integer in the range [0, 255]

// Set possible states
enum {STATE_WAIT, STATE_LEFT25, STATE_RIGHT25, STATE_REAR25, STATE_ALL25, STATE_LEFT0, STATE_RIGHT0, STATE_REAR0, STATE_START};
unsigned char state; 


void setup() {
  pinMode(pmwPin1, OUTPUT);
  pinMode(pmwPin2, OUTPUT);

  pinMode(SWITCH_PIN, INPUT);
  
}

void loop() {
  //if (current_time - last_fsm_time > fsm_interval) {
  //  fsm_task();
  //  last_fsm_time = current_time;
  //}

  //analogWrite(pmwPin1, duty);

  fsm_step();

  // Switch Test

}


float bound(float value, float min_value, float max_value) {
  // return min_value
  if (value < min_value) {
    return min_value;
  }

  // return max_value
  if (value > max_value) {
    return max_value;
  }

  // return value
  else {
    return value;
  }
}

void set_motor(int motor, float val) {
  // check that val is an element of [-64, 64]
  if ((val >= -64) && (val <= 64)) {
    // bound value to this range
    Serial.printf("val %f is out of range [-64, 64]. Calling def bound.", val);
    
    bound(val, -64, 64);

  }

  // set thrust mag and dir for each motor
  //
  // ***** code here *****
  //
  //


}

void set_motors(float val[3]) {
  const float negative_gain[3] = {1.0, 1.0, 1.0};

  // if val is negative, multiply by the corresponding negative_gain
  // bound and convert to integer
  //
  // ***** code here *****
  //
  //
  
  set_motor(0, val[0]);
  set_motor(1, val[1]);
  set_motor(2, val[2]);
  

}

void fsm_step() {
  //static State state = STATE_START;

  switch (state) {

    // wait for the switch to be pressed
    case STATE_WAIT:
      // if switch is pressed, change state
      // 
      // ***** code here *****
      //

      break;
    
    // ramp L motor to 25%
    case STATE_LEFT25:
      // 
      // ***** code here *****
      //
      break;

    // ramp R motor to 25%
    case STATE_RIGHT25:
      // 
      // ***** code here *****
      //
      break;

    // ramp rear motor to 25%
    case STATE_REAR25:
      // 
      // ***** code here *****
      //
      break;

    // ramp all motors down 25%
    case STATE_ALL25:
      // 
      // ***** code here *****
      //
      break;

    // ramp L motor to 0%
    case STATE_LEFT0:
      // 
      // ***** code here *****
      //
      break;

    // ramp R motor to 0%
    case STATE_RIGHT0:
      // 
      // ***** code here *****
      //
      break;

    // ramp rear motor to 0%
    case STATE_REAR0:
      // 
      // ***** code here *****
      //
      break;

    // return to start state
    case STATE_START:
      // 
      // ***** code here *****
      //
      break;
    
  }


}
