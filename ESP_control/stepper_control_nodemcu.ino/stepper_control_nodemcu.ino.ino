//Robotics Club - Motion Planning System
//Guining Pertin - 05-06-2019
//NodeMCU control code

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <AccelStepper.h>

const char* ssid = "ROBO_IITG_2.4G";
const char* password = "iamthefuture";
//Motor driver connections

//LForward - in1 - LOW & in2 - HIGH
//RForward - in3 - LOW & in4 - HIGH


long receivedDistance = 0;//distance to move
long receivedSpeed = 0;
long receivedAcceleration = 0;
char receivedCommand;

bool newData, canMove= false;

AccelStepper stepper2(8, D1, D3, D2, D4);
AccelStepper stepper1(8, D5, D7, D6, D8);

void setup() {
  Serial.begin(9600);
  WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting..");
  }
  Serial.println("Connected");

  stepper1.setMaxSpeed(2000);
  stepper1.setAcceleration(1000);
  stepper1.disableOutputs(); 
  stepper2.setMaxSpeed(2000);
  stepper2.setAcceleration(1000);
  stepper2.disableOutputs(); 
}

void loop() {
  String dist;
  String angle;
  String dir;
  if (WiFi.status() == WL_CONNECTED){
    HTTPClient http;
    http.begin("http://192.168.0.5:8081/data.txt");
    int httpCode = http.GET();
  
    if (httpCode > 1) {
      String payload = http.getString();
      Serial.println(payload);  
      dist = payload.substring(0, payload.indexOf('+'));
      angle = payload.substring(payload.indexOf('+')+1, payload.indexOf('-'));
      dir = payload.substring(payload.indexOf('-')+1, payload.indexOf('*'));
      Serial.print(dist);
      Serial.print(",");
      Serial.print(angle);
      Serial.print(",");
      Serial.println(dir);
    }
  
      if(abs(angle.toInt())>0)
      {
        receivedDistance = 13.66667*(angle.toInt());
        stepper1.enableOutputs();
        stepper2.enableOutputs();
   
          if(dir[0]=='r')
          {
            stepper1.move(receivedDistance);
            stepper2.move(receivedDistance);
          }
          else{
            stepper1.move(-1*receivedDistance);
            stepper2.move(-1*receivedDistance);
          }
          while(abs(stepper1.currentPosition())<receivedDistance && (stepper2.currentPosition())<receivedDistance)
          {
             stepper1.run();
             stepper2.run();
          }
          stepper1.setCurrentPosition(0);
          stepper2.setCurrentPosition(0);
         // delay(5000);
          //stepper1.disableOutputs();
          //stepper2.disableOutputs();
        }
     
      if(abs(dist.toInt())>0)
      {
        receivedDistance = ((dist.toInt())*4076)/21.98;//perimeter of wheel = 7*pi;number of revolution = dist/(7*pi);number of steps in one full rotation= 4076;
        Serial.println(receivedDistance);
        stepper1.move(-1*receivedDistance);
        stepper2.move(receivedDistance);
        while(abs(stepper1.currentPosition())<receivedDistance && (stepper2.currentPosition())<receivedDistance)
        {
            stepper1.run();
            stepper2.run();
        }
        stepper1.setCurrentPosition(0);
        stepper2.setCurrentPosition(0);
        //delay(5000);
//        stepper1.disableOutputs();
//        stepper2.disableOutputs();
        
      }
        stepper1.disableOutputs();
        stepper2.disableOutputs();
        //delay(10000);
        http.end();
  }
}
