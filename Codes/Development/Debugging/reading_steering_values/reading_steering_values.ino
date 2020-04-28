/*
 * Used purely to read the steering angle of the optoencoder.
 */

// Pins
const int CH_A = 2;        // Encoder pin A
const int CH_B = 3;        // Encoder pin B
// MUST CHANGE IN pwmread_rcfailsafe also! (pwmPIN[])

// Other variables you shouldn't change
const int REC_F = 25;      // Receiver frame rate [ms]

int speedVal = 0;          // Speed of the motor, [unitless]
int motorSpeed = 0;        // Speed of the motor, [unitless]

// PID variables
double count = 1;
double angle = 0;
boolean A, B;
byte state, statep;

double desiredPosition = 0;
double Kp = 0.1; // Tuned by Ike
double Ki = 1.0;
double Kd = 5.0;

float lastError = 0;
float error = 0;
float changeError = 0;
float totalError = 0;
float pidTerm = 0;
float pidTermScaled = 0;

void setup() {
  Serial.begin(9600);
  
  // Interrupt pins for encoder
  pinMode(CH_A, INPUT); // Encoders
  pinMode(CH_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(CH_A), Achange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH_B), Bchange, CHANGE);
}

void loop() {
  PIDcalculation();     // Update PID values

  // Get the desired position
  //desiredPosition = map(joystickValue, LOW_VAL, HIGH_VAL, -2700, 2700);
  desiredPosition = 0;
  Serial.println(angle);
  //Serial.print("\t");
  //Serial.println(desiredPosition);

  //RIGHT
  if (abs(desiredPosition - angle) >= 30 && desiredPosition < angle) {
    //analogWrite(PWM_OUT, pidTermScaled);
    //digitalWrite(LEFT, LOW);
    //digitalWrite(RIGHT, HIGH);
    //Serial.println("RIGHT ");

    // Serial.println(speedVal);
    // Serial.println(joystickValue);
  }

  //LEFT
  else if (abs(desiredPosition - angle) >= 30 && desiredPosition > angle) {
    //analogWrite(PWM_OUT, pidTermScaled);
    //digitalWrite(LEFT, HIGH);
    //digitalWrite(RIGHT, LOW);
    //Serial.println("LEFT");

    // Serial.println(speedVal);
    // Serial.println(joystickValue);
  }

  //CENTER
  else {
    //Serial.println("CENTER");
    //Serial.println(joystickValue);
    //digitalWrite(RIGHT, LOW);
    //digitalWrite(LEFT, LOW);
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
