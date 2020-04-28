/*
 * Code to receive commands from remote controller and the CART computer,
 * although the latter is CURRENTLY NOT IMPLEMENTED.
 * Signal received is sent to the motor driver, which in turn actuates the gas pedal via motor.
 * Feedback from optoencoder within motor is currently not used.
 * 
 * Uses the "pwmread_rcfailsafe" library instead of pulseIn().
 * pulseIn() might be used in place.
 */

// TODO - change angle conversion to "count / 8", as Ike had it, and then
// re-calibrate the motor FULLY_PUSHED and PEDAL_DEADZONE values

// Constants you might change (calibration)

/*
// FOR RC RECEIVER 
const int DEADZONE = 100;  // Deadzone (in PWM [units?]) for actuation  
const int DEAD_VAL = -108; // Inactive value
const int LOW_VAL = -800;
const int HIGH_VAL = 1640;
*/

// FOR PIXHAWK
const int DEADZONE = 100;  // Deadzone (in PWM [units?]) for actuation  
const int DEAD_VAL = -116; // Inactive value
const int LOW_VAL = -761;
const int HIGH_VAL = 1240;


/*
// FOR PIXHAWK
const int DEADZONE = 100;  // Deadzone (in PWM [units?]) for actuation  
const int DEAD_VAL = 300; // Inactive value
const int LOW_VAL = 0;
const int HIGH_VAL = 800;
*/

// Calibration for motor
const int FULLY_PUSHED = -125; // Value of "angle" at full push, TOOD should be higher, like 116
const int NEUTRAL = 0; // TODO - PEDAL_DEADZONE was 15 with old PID gains
const int PEDAL_DEADZONE = 15; // Basically, how much you care about error
                               // This avoids a jittering motor pulling on the pedal

// Pins
const int PWM_OUT = 5;     // Output pin to drive the motor
const int PUSH = 6;        // Pin to drive motor to pull pedal
const int UNPUSH = 7;      // Pin to drive motor to UNPUSH pedal
const int CH_A = 2;        // Encoder pin A
const int CH_B = 3;        // Encoder pin B
const int RECEIVER = 9;    // Pin to receive signal from receiver, 
                           // MUST CHANGE IN pwmread_rcfailsafe also! (pwmPIN[])

// Other variables yuo shouldn't change
const int REC_F = 25;      // Receiver frame rate [ms]

int speedVal = 0;          // Speed of the motor, [unitless]
int motorSpeed = 0;        // Speed of the motor, [unitless]

unsigned long now;         // Timing variable in order to keep data updated
unsigned long rc_update;   // (same as above)
float RC_in;               // Raw input. This is length of the pulse, in MS, 
                           // BUT it's written in seconds (I think), as float
                           // I think (look at Arduino standard pulseIn() function)

// PID variables
double count = 1;
double angle = 0;
boolean A, B;
byte state, statep;

double desiredPosition = 0;
// OLD PID gains
const double Kp = 0.5; // Tuned by Ike
const double Ki = 1.0;
const double Kd = 3.0;
/*
const double Kp = 7.0; // Tuned by Jesse, using https://robotics.stackexchange.com/questions/167/what-are-good-strategies-for-tuning-pid-loops
const double Ki = 0.0;
const double Kd = 10.0; // NOT DONE tuning
*/

float lastError = 0;
float error = 0;
float changeError = 0;
float totalError = 0;
float pidTerm = 0;
float pidTermScaled = 0;

void setup() {
  Serial.begin(9600);

  setup_pwmRead();
  pinMode(PWM_OUT, OUTPUT);
  pinMode(PUSH, OUTPUT);
  pinMode(UNPUSH, OUTPUT);

  // Interrupt pins for encoder
  pinMode(CH_A, INPUT); // Encoders
  pinMode(CH_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(CH_A), Achange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH_B), Bchange, CHANGE);
}

void loop() {
  PIDcalculation();     // Update PID values
  int joystickValue = readReceiverSignal(); // Get signal from RC receiver

  // Get the desired position
  desiredPosition = map(joystickValue, DEAD_VAL, HIGH_VAL, NEUTRAL, FULLY_PUSHED);
  Serial.print(angle); // TODO - find range of angle
  Serial.print("\t");
  Serial.print(desiredPosition);
  Serial.print("\t");
  Serial.println(joystickValue);
   
   // FORWARD
  if (abs(desiredPosition - angle) >= PEDAL_DEADZONE && desiredPosition < angle) {
    analogWrite(PWM_OUT, pidTermScaled);
    digitalWrite(UNPUSH, LOW);
    digitalWrite(PUSH, HIGH);
    //Serial.println("RIGHT ");

    // Serial.println(speedVal);
    // Serial.println(joystickValue);
  }

  // Release the pedal
  else if (abs(desiredPosition - angle) >= PEDAL_DEADZONE && desiredPosition > angle) {
    analogWrite(PWM_OUT, pidTermScaled);
    digitalWrite(UNPUSH, HIGH);
    digitalWrite(PUSH, LOW);
    //Serial.println("LEFT");

    // Serial.println(speedVal);
    // Serial.println(joystickValue);
  }

  // Centered, so we're fine
  else {
    //Serial.println("CENTER");
    //Serial.println(joystickValue);
    analogWrite(PWM_OUT, 0);
    digitalWrite(UNPUSH, HIGH); // TODO - changed to be HIGH instead of LOW
    digitalWrite(PUSH, HIGH);
  }
}

void PIDcalculation() {
  angle = (count / 4); //count to angle conversion
  error = desiredPosition - angle;

  changeError = error - lastError; // derivative term
  totalError += error; //accumalate errors to find integral term
  pidTerm = (Kp * error) + (Ki * totalError) + (Kd * changeError);//total gain
  pidTerm = constrain(pidTerm, -255, 255);//constraining to appropriate value
  pidTermScaled = abs(pidTerm);//make sure it's a positive value

  lastError = error;
}

void Achange() //these functions are for finding the encoder counts
{
  A = digitalRead(CH_A); // INTERRUPT PINS HERE
  B = digitalRead(CH_B); // INTERRUPT PINS HERE

  if ((A == HIGH) && (B == HIGH)) state = 1;
  if ((A == HIGH) && (B == LOW)) state = 2;
  if ((A == LOW) && (B == LOW)) state = 3;
  if ((A == LOW) && (B == HIGH)) state = 4;
  switch (state)
  {
    case 1:
      {
        if (statep == 2) count++;
        if (statep == 4) count--;
        break;
      }
    case 2:
      {
        if (statep == 1) count--;
        if (statep == 3) count++;
        break;
      }
    case 3:
      {
        if (statep == 2) count --;
        if (statep == 4) count ++;
        break;
      }
    default:
      {
        if (statep == 1) count++;
        if (statep == 3) count--;
      }
  }
  statep = state;

}

void Bchange()
{
  A = digitalRead(CH_A);
  B = digitalRead(CH_B);

  if ((A == HIGH) && (B == HIGH)) state = 1;
  if ((A == HIGH) && (B == LOW)) state = 2;
  if ((A == LOW) && (B == LOW)) state = 3;
  if ((A == LOW) && (B == HIGH)) state = 4;
  switch (state)
  {
    case 1:
      {
        if (statep == 2) count++;
        if (statep == 4) count--;
        break;
      }
    case 2:
      {
        if (statep == 1) count--;
        if (statep == 3) count++;
        break;
      }
    case 3:
      {
        if (statep == 2) count --;
        if (statep == 4) count ++;
        break;
      }
    default:
      {
        if (statep == 1) count++;
        if (statep == 3) count--;
      }
  }
  statep = state;
}

int readReceiverSignal() {
  now = millis();
  if (RC_avail() || now - rc_update > REC_F) { // If RC data available, or time to update
    rc_update = now;
    //print_RCpwm(); // For debugging
    RC_in = RC_decode(1) * 1000; // TODO - don't necessarily need
  }

  int res = ((int) RC_in) - DEAD_VAL;
  if(abs(res) > DEADZONE) {
    return res;
  } else {
    return 0;
  }
}
