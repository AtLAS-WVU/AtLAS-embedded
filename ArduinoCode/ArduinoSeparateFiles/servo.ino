#include <Servo.h>

#define SERVO_PIN 13  //This pin may need changed
Servo servo;
int angle = 0;

void setup() {
  servo.attach(SERVO_PIN);
}

//This loop should continually change the servo positiona full 180 degrees
//with having 5 second pauses when facing upward, frontward, or downward
void loop() {
  //Loop to rotate from 0-180 degrees
  for(int angle=0;angle<180;++angle)
  {
    //Add pause when upward or frontward
    if(angle == 0 || angle == 90)
    {
      delay(5000); //if facing forward, delay 5 seconds 
    }
    servo.write(angle); //writes angle to servo
    delay(45);  //add delay to slow servo down
  }
  //Loop to rotate back from 180-0 degrees
  for(int angle=180;angle>0;--angle)
  {
    //Add pause when downward or frontward
    if(angle == 180 || angle == 90)
    {
      delay(5000);
    }
    servo.write(angle); //writes angle to servo
    delay(45);  //add delay to slow servo done
  }
}
