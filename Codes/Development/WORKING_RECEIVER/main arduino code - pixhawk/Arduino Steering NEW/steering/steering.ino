/*
   Code adapted from Dwight Theriot.
*/

/*
// Constants you might change (calibration) for receiver
const int DEADZONE = 100;  // Deadzone (in PWM [units?]) for actuation
const int DEAD_VAL = 340;  // Inactive value
const int LOW_VAL = -700 - DEAD_VAL;  // Receiver low val (turning to left)
const int HIGH_VAL = 1600 - DEAD_VAL; // Receiver high val (turning wheel to right)
*/

// Constants you might change (calibration) for receiver
// Pixhawk
const int DEADZONE = 150;  // Deadzone (in PWM [units?]) for actuation
const int DEAD_VAL = 0;  // Inactive value
const int LOW_VAL = -1300;  // Receiver low val (turning to left)
const int HIGH_VAL = 2230; // Receiver high val (turning wheel to right)


// Calibration for motor
const int LEFT_WHEEL_LOCK = 150;   // Value of "angle" at left wheel lock
const int RIGHT_WHEEL_LOCK = -150; // Value of "angle" at right wheel lock
const double STEERING_DEADZONE = 20; // Basically, how much you care about error
                                     // This avoids a jittering steering wheel

// Pins
const int PWM_OUT = 5;     // Output pin to drive the motor
const int LEFT = 7;        // Motor driver pin: to go left
const int RIGHT = 6;       // Motor driver pin: to go right
const int CH_A = 2;        // Encoder pin A
const int CH_B = 3;        // Encoder pin B
const int RECEIVER = 9;    // Pin to receive signal from receiver,
// MUST CHANGE IN pwmread_rcfailsafe also! (pwmPIN[])

// Other variables you shouldn't change
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
const double Kp = 0.1; // Tuned by Ike
const double Ki = 1.0;
//const double Kd = 5.0;
const double Kd = 1.0;

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
  pinMode(LEFT, OUTPUT);
  pinMode(RIGHT, OUTPUT);

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
  desiredPosition = map(joystickValue, LOW_VAL, HIGH_VAL, LEFT_WHEEL_LOCK, RIGHT_WHEEL_LOCK);
  Serial.print(angle); // TODO - find range of angle
  Serial.print("\t");
  Serial.print(desiredPosition);
  Serial.print("\t");
  Serial.println(joystickValue);

  //RIGHT
  if (abs(desiredPosition - angle) >= STEERING_DEADZONE && desiredPosition < angle) {
    analogWrite(PWM_OUT, pidTermScaled);
    digitalWrite(LEFT, LOW);
    digitalWrite(RIGHT, HIGH);
    //Serial.println("RIGHT ");

    // Serial.println(speedVal);
    // Serial.println(joystickValue);
  }

  //LEFT
  else if (abs(desiredPosition - angle) >= STEERING_DEADZONE && desiredPosition > angle) {
    analogWrite(PWM_OUT, pidTermScaled);
    digitalWrite(LEFT, HIGH);
    digitalWrite(RIGHT, LOW);
    //Serial.println("LEFT");

    // Serial.println(speedVal);
    // Serial.println(joystickValue);
  }

  //CENTER
  else {
    //Serial.println("CENTER");
    //Serial.println(joystickValue);
    
    // NEW very safe analogWrite with analogWrite
    analogWrite(PWM_OUT, 0); // TODO - added this to be safe
    digitalWrite(RIGHT, HIGH);
    digitalWrite(LEFT, HIGH);
//    digitalWrite(RIGHT, LOW);
//    digitalWrite(LEFT, LOW);
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
