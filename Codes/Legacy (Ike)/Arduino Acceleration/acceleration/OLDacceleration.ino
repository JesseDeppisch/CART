int pwm = 5;
const int FORWARD = 6;
const int BACK = 7;
int speedVal = 0;
int motorSpeed = 0;

void setup() {
  Serial.begin(9600);
  pinMode(pwm, OUTPUT);
  pinMode(FORWARD, OUTPUT);
  pinMode(BACK, OUTPUT);

}

void loop() {
  int joystickValue = analogRead(A4);
  
  //GO FORWARD
  if(joystickValue > 530){ 
    speedVal = map(joystickValue,512,660,0,50);
    // motorSpeed = speedVal*4;
    motorSpeed = speedVal*1.5;
    analogWrite(pwm, motorSpeed);
    digitalWrite(FORWARD, HIGH);
    digitalWrite(BACK, LOW);
    Serial.println("GO FORWARD");
    Serial.println(motorSpeed);
    Serial.println(joystickValue);
  }
  //GO BACK/RELEASE
  else{
    Serial.println("CENTER");
    Serial.println(joystickValue);
    digitalWrite(FORWARD, HIGH);
    digitalWrite(BACK, HIGH);
    }
    

}
