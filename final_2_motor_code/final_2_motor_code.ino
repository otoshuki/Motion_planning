#include <AccelStepper.h>

long receivedDistance = 0;//distance to move
long receivedSpeed = 0;
long receivedAcceleration = 0;
char receivedCommand;

bool newData, canMove= false;

AccelStepper stepper1(8, 3, 5, 4, 6);
AccelStepper stepper2(8, 8, 10, 9, 11);

void setup() {
  Serial.begin(9600);
  
  stepper1.setMaxSpeed(2000);
  stepper1.setAcceleration(1000);
  stepper1.disableOutputs(); 
  stepper2.setMaxSpeed(2000);
  stepper2.setAcceleration(1000);
  stepper2.disableOutputs(); 
}

void loop() {
  checkInput();
  checkMove();  

}


void checkMove()
{
  if(canMove==true)
  {
    if(abs(stepper1.currentPosition())<receivedDistance && (stepper2.currentPosition())<receivedDistance)
    {
      stepper1.enableOutputs();
      stepper2.enableOutputs();
      stepper1.run();
      stepper2.run();
    }
    else
    {
      canMove = false;
      stepper1.disableOutputs();
      stepper2.disableOutputs();
      Serial.print("Current Pos:");
      Serial.println(stepper1.currentPosition());
      Serial.println(stepper2.currentPosition());
      stepper1.setCurrentPosition(0);
      stepper2.setCurrentPosition(0);
      Serial.print("Zero Pos:");
      Serial.println(stepper1.currentPosition());
      Serial.println(stepper2.currentPosition());
    }
  }
  else{
    return;
  }
}
void checkInput()
{
  if (Serial.available()>0)
  {
    receivedCommand = Serial.read();
    newData = true;
  }
  if(newData == true)
  {
    canMove = true;
    receivedDistance = Serial.parseFloat();
    receivedSpeed = Serial.parseFloat();
    stepper1.setMaxSpeed(receivedSpeed);
    stepper2.setMaxSpeed(receivedSpeed);
    switch(receivedCommand){
      case 'b' :
        stepper1.move(receivedDistance);
        stepper2.move(-1*receivedDistance);
        Serial.println("b");
        break;
      case 'f':
        stepper1.move(-1*receivedDistance);
        stepper2.move(receivedDistance);
        Serial.println("f");
        break;
      case 'r':
         stepper1.move(receivedDistance);
         stepper2.move(receivedDistance);
         Serial.println("r");
         break;
      case 'l':
        stepper1.move(-1*receivedDistance);
        stepper2.move(-1*receivedDistance);
        Serial.println("l");
        break;
      default:
        return;
    }
    newData = false;
  }
  
}
