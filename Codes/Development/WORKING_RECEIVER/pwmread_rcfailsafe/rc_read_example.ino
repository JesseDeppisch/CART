#include <math.h>

unsigned long now;                        // timing variables to update data at a regular interval                  
unsigned long rc_update;
const int channels = 1;                   // specify the number of receiver channels
float RC_in[channels];                    // an array to store the calibrated input from receiver 

const int DEAD_VAL = 0;

void setup() {
    setup_pwmRead();                      
    Serial.begin(9600);
    //Serial.begin(115200);
}

void loop() {  
    
    now = millis();
    
    if(RC_avail() || now - rc_update > 25){   // if RC data is available or 25ms has passed since last update (adjust to be equal or greater than the frame rate of receiver)
      
      rc_update = now;                           
      
      //print_RCpwm();                        // uncommment to print raw data from receiver to serial
      
      for (int i = 0; i<channels; i++){       // run through each RC channel
        int CH = i+1;
        
        RC_in[i] = RC_decode(CH);             // decode receiver channel and apply failsafe
        Serial.print(RC_in[i]); // -1 to 1 (float)
        Serial.print("\t");
        Serial.print((int) (RC_in[i] * 1000));
        Serial.print("\t");
        Serial.println(((int) (RC_in[i] * 1000)) - DEAD_VAL);

        //Serial.println(RC_in[i] - test);
        //Serial.println(round(RC_in[i] - test));
        
        //print_decimal2percentage(RC_in[i]);   // uncomment to print calibrated receiver input (+-100%) to serial       
      }
      //Serial.println();                       // uncomment when printing calibrated receiver input to serial.
    }
}
