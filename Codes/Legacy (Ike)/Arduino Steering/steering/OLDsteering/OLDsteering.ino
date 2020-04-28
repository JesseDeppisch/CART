int pwm = 5;
const int RIGHT = 6;
const int LEFT = 7;
int speedVal = 0;
int motorSpeed = 0;

void setup() {
  Serial.begin(9600);
  pinMode(pwm, OUTPUT);
  pinMode(RIGHT, OUTPUT);
  pinMode(LEFT, OUTPUT);

}

void loop() {
  int joystickValue = analogRead(A4);

  //TURNING RIGHT
  if(joystickValue < 510){ 
    speedVal = map(joystickValue,0,510,255,0);
    motorSpeed = speedVal*4;
    analogWrite(pwm, motorSpeed);
    digitalWrite(LEFT, LOW);
    digitalWrite(RIGHT, HIGH);
    Serial.println("TURN RIGHT");Serial.println(speedVal);Serial.println(joystickValue);
  }
  //TURNING LEFT
  else if(joystickValue > 520){
    speedVal = map(joystickValue,520,1023,0,255);
    motorSpeed = speedVal*4;
    analogWrite(pwm, motorSpeed);
    digitalWrite(LEFT, HIGH);
    digitalWrite(RIGHT, LOW);
    Serial.println("TURN LEFT");
    Serial.println(speedVal);
    Serial.println(joystickValue);
  }
  else{
    Serial.println("CENTER");
    Serial.println(joystickValue);
    digitalWrite(RIGHT, HIGH);
    digitalWrite(LEFT, HIGH);
    }
    

}
