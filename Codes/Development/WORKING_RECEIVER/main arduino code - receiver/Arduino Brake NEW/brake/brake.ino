/*
 * Code to receive commands from remote controller and the CART computer,
 * although the latter is CURRENTLY NOT IMPLEMENTED.
 * Signal received is sent to the switch which will flick solenoid on, braking the CART.
 * 
 * Uses the "pwmread_rcfailsafe" library instead of pulseIn().
 * pulseIn() might be used in place.
 */

// const int solenoid = 4; // SHOULD be OUT_LOW

/*
// Constants you might change (calibration)
const int DEADZONE = 100;  // Deadzone (in PWM [units?]) for actuation  
const int DEAD_VAL = -108; // Inactive value, note that
const int LOW_VAL = -800;  // This should be the same as in the "acceleration" file
const int HIGH_VAL = 1640;
*/

// Constants you might change (calibration)
const int DEADZONE = 300;  // Deadzone (in PWM [units?]) for actuation  
const int DEAD_VAL = -116; // Inactive value, note that
const int LOW_VAL = -761;  // This should be the same as in the "acceleration" file
const int HIGH_VAL = 1240;

// Calibration for the button (very simple)
const int BUTTON_LOW_VAL = -977;  // Low, or inactive, value
const int BUTTON_HIGH_VAL = 1643; // High, or pressed, value

// Pins
const int OUT_LOW = 4;        // Output pin to actuate the low-pressure solenoid
const int OUT_HIGH = 7;       // Output pin to actuate the high-pressure solenoid
const int RECEIVER_LOW = 10;  // Button, binary
const int RECEIVER_HIGH = 11; // Trigger, - to + range
    

// Other variables you shouldn't change
const int REC_F = 25;      // Receiver frame rate [ms]

int speedVal = 0;          // Speed of the motor, [unitless]
int motorSpeed = 0;        // Speed of the motor, [unitless]

unsigned long now;         // Timing variable in order to keep data updated
unsigned long rc_update;   // (same as above)
float RC_button = 0;       // From button
float RC_in;               // From trigger
                           // Raw input. This is length of the pulse, in MS, 
                           // BUT it's written in seconds (I think), as float
                           // I think (look at Arduino standard pulseIn() function)

void setup() {
  Serial.begin(9600);

  setup_pwmRead();
  
  pinMode(OUT_LOW, OUTPUT);
  pinMode(OUT_HIGH, OUTPUT);
}

void loop() {
  // Get signal from receiver
  int triggerVal = readReceiverSignal();

  /*
  // Check if the button is pressed, for hard braking
  if (RC_button > 0) {
    // Button is pressed, so brake hard
    digitalWrite(OUT_HIGH, HIGH);
  } else {
    digitalWrite(OUT_HIGH, LOW);
  }

  // Check if the trigger is pushed up, for soft braking
  // Note that if the button is pressed also, hard brake engages due to the way pneumatics are set up
  if (-triggerVal > DEADZONE) {
    // Brake softly
    digitalWrite(OUT_LOW, HIGH);
  } else {
    digitalWrite(OUT_LOW, LOW);
  }
  */
  
  // Old, state-machine approach. Figured it might be good because both solenoids can't be opened at the same time
  // However, I don't think there's anything wrong with opening both solenoids at once.
  // Are we braking hard? This is our first check, because we want fast emergency brake
  if (RC_button > 0) { // TODO - do something different? I just compare to 0, since low and high are different sides
                       // I don't even look at the BUTTON_LOW_VAL value lol
    // Button is pressed, so brake hard
    digitalWrite(OUT_LOW, HIGH);
    digitalWrite(OUT_HIGH, HIGH);
  } else if (-triggerVal > DEADZONE) {
    // Brake softly
    digitalWrite(OUT_LOW, HIGH);
    digitalWrite(OUT_HIGH, LOW);
  } else {
    // No brakes
    digitalWrite(OUT_LOW, LOW);
    digitalWrite(OUT_HIGH, LOW);
  }
}


int readReceiverSignal() {
  // I know this function is updating 2 global variables and returns one
  // This is ugly, but functional. I'll fix this later if I get a chance
  // and make it cleaner
  
  now = millis();
  if (RC_avail() || now - rc_update > REC_F) { // If RC data available, or time to update
    rc_update = now;
    //print_RCpwm(); // For debugging
    RC_button = RC_decode(1) * 1000;
    RC_in = RC_decode(2) * 1000;
  }

  int res = ((int) RC_in) - DEAD_VAL;
  if (abs(res) > DEADZONE) {
    return res;
  } else {
    return 0;
  }
}
