/*
 * Calculates MPH based on odometer
 */

const float RADIUS = (17.0 / 2); // Radius of tire [inch] // TODO - actually measure
const int NUM_MAGNETS = 24;      // Number of magnets, assuming equally distributed around the circumference
const long REFRESH_RATE = 300;   // Period of speed updates, in milliseconds (i.e. every X ms, speed is calculated)

float mph;             // Miles per hour

volatile byte count;   // Every magnet pass
unsigned int rpm;      // Rotations per minute
unsigned long timeold; // Old time value

void setup_odometer() {
  attachInterrupt(0, magnet_detect, RISING); //Initialize the intterrupt pin (Arduino digital pin 2) and attach function
  count = 0;
  rpm = 0;
  timeold = 0;
}

void calculateSpeed() {
  // TODO - might want to check count instead of time, but that would only update if moving
  if (millis() - timeold > REFRESH_RATE) {
    // Measure RPM // TODO - consolidate extra variables into one-liner
    float circum = 2*M_PI*RADIUS;                  // Circumference
    float sectionDist = circum / NUM_MAGNETS;      // "Section" distance, i.e. arc length between magnets
 
    float dist = sectionDist * count;              // Arc length travelled
    float seconds = (millis() - timeold) / 1000.0; // Time, [s]

    float inps = dist / seconds;                   // Inches per second

    // Calculate speed in mph
    mph = inps * (0.056818); // 0.056818 = 3600 / (12*5280)
                             // Necessary becuase otherwise I was overflowing variable

    // Print the speed
    Serial.println(mph);

    // Update the time and reset the counter
    timeold = millis();
    count = 0;
  }
}

//This function is called whenever a magnet/interrupt is detected by the arduino
void magnet_detect() {
  count++;
//  Serial.println(count);
}
