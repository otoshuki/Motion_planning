#include <AccelStepper.h>
#define HALFSTEP 8

//Driver pin connections
#define d1_in1  2    
#define d1_in2  3    
#define d1_in3  4    
#define d1_in4  5    

#define d2_in1  6    // IN1 on the ULN2003 driver 2
#define d2_in2  7    // IN2 on the ULN2003 driver 2
#define d2_in3  8    // IN3 on the ULN2003 driver 2
#define d2_in4  9    // IN4 on the ULN2003 driver 2

//Stepper initialization - IN1 - IN3 - IN2 - IN$
AccelStepper stepper1(HALFSTEP, d1_in1, d1_in3, d1_in2, d1_in4);
AccelStepper stepper2(HALFSTEP, d2_in1, d2_in3, d2_in2, d2_in4);

void setup() {
  stepper1.setMaxSpeed(2000.0);
  stepper1.setAcceleration(1000.0);
  stepper1.setSpeed(600);
  stepper1.moveTo(2048);

  stepper2.setMaxSpeed(2000.0);
  stepper2.setAcceleration(1000.0);
  stepper2.setSpeed(600);
  stepper2.moveTo(-2048);
}

void loop() {
  //Change direction when the stepper reaches the target position
  if (stepper1.distanceToGo() == 0&& stepper2.distanceToGo() == 0) {
    stepper1.moveTo(-stepper1.currentPosition());
    stepper2.moveTo(-stepper2.currentPosition());
  }
  stepper1.run();
  stepper2.run();

}
