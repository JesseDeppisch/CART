const int solenoid = 4;

void setup() {
  Serial.begin(9600);
  pinMode(solenoid, OUTPUT);
}

void loop() {
  int joystickValue = analogRead(A4);
  Serial.println(joystickValue);
  
  if(joystickValue > 480){
    Serial.println("SOLENOID OFF");
    digitalWrite(solenoid, LOW);
  }
  else{
    Serial.println("BRAKE");
    digitalWrite(solenoid, HIGH);
  }
    

}
