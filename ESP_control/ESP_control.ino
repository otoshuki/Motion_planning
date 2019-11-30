//Robotics Club - Motion Planning System
//Guining Pertin - 05-06-2019
//NodeMCU control code

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
 
const char* ssid = "ROBO_IITG_2.4G";
const char* password = "iamthefuture";
//Motor driver connections
int enA = 12; //Left
int in1 = 5;
int in2 = 4;
int in3 = 0;
int in4 = 2;
int enB = 14; //Right
int counter = 0;

//LForward - in1 - LOW & in2 - HIGH
//RForward - in3 - LOW & in4 - HIGH

//PID coeffs
float kp_angle = 0.1;
float ki_angle = 5;
float kd_angle = 1000;
float kp_dist = 0.1;
float ki_dist = 15;
float kd_dist = 1000;
//PID values
float currTime_angle = 0;
float prevTime_angle = 0;
float error_angle = 0;
float prevError_angle = 0;
float p_angle = 0;
float i_angle = 0;
float d_angle = 0;
float pid_angle = 0;
float currTime_dist = 0;
float prevTime_dist = 0;
float error_dist = 0;
float prevError_dist = 0;
float p_dist = 0;
float i_dist = 0;
float d_dist = 0;
float pid_dist = 0;

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting..");
  }
  Serial.println("Connected");
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enB, OUTPUT);
  digitalWrite(enA, LOW);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(enB, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW); 
}

void loop() {
  String dist;
  String angle;
  String dir;
  if (WiFi.status() == WL_CONNECTED){
    HTTPClient http;
    http.begin("http://192.168.0.5:8080/data.txt");
    int httpCode = http.GET();
  
    if (httpCode > 0) {
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
      //Control system
      if (dist.length() > 0){
        //Check angular motion first
        if (angle.toInt() > 8){
          currTime_angle = millis();
          //Turn off linear motion PID
          //PID control for angle error
          error_angle = angle.toInt();
          p_angle = kp_angle * error_angle;
          //i_angle += ki_angle * error_angle*(currTime_angle-prevTime_angle);
          d_angle = kd_angle * (error_angle - prevError_angle)/(currTime_angle-prevTime_angle);
          pid_angle = p_angle + d_angle;
          //Set PID error limits
          if (pid_angle > 480) pid_angle = 480;
          else if (pid_angle < 300) pid_angle = 300;
          //Serial.println("Setting Angle");
          //Check direction
          if (dir == "l"){
            analogWrite(enA, pid_angle);
            analogWrite(enB, pid_angle);
            digitalWrite(in1, HIGH);
            digitalWrite(in2, LOW);
            digitalWrite(in3, LOW);
            digitalWrite(in4, HIGH);
          }
          else if (dir == "r"){
            analogWrite(enA, pid_angle);
            analogWrite(enB, pid_angle);
            digitalWrite(in1, LOW);
            digitalWrite(in2, HIGH);
            digitalWrite(in3, HIGH);
            digitalWrite(in4, LOW);
          }
          prevError_angle = error_angle;
          prevTime_angle = currTime_angle;
        }
        else if (dist.toInt() > 30){
          currTime_dist = millis();
          //Turn off angular motion PID
          //PID control for dist error
          error_dist = dist.toInt();
          p_dist = kp_dist * error_dist;
          //i_angle += ki_angle * error_angle*(currTime_angle-prevTime_angle);
          d_dist = kd_dist * (error_dist - prevError_dist)/(currTime_dist-prevTime_dist);
          pid_dist = p_dist + d_dist;
          //Set PID error limits
          if (pid_dist > 480) pid_dist = 480;
          else if (pid_dist < 300) pid_dist = 300;
          digitalWrite(enA, pid_dist);
          digitalWrite(enB, pid_dist);
          digitalWrite(in1, LOW);
          digitalWrite(in2, HIGH);
          digitalWrite(in3, LOW);
          digitalWrite(in4, HIGH);
          //Serial.println("Setting Distance");
          prevError_dist = error_dist;
          prevTime_dist = currTime_dist;
        }
        else{
          digitalWrite(enA, LOW);
          digitalWrite(enB, LOW);
          currTime_angle = 0;
          prevTime_angle = 0;
          error_angle = 0;
          prevError_angle = 0;
          p_angle = 0;
          i_angle = 0;
          d_angle = 0;
          pid_angle = 0;
          //Serial.println("Stopped");
        }
        if (dist.toInt() < 30){
          digitalWrite(enA, LOW);
          digitalWrite(enB, LOW);
          currTime_angle = 0;
          prevTime_angle = 0;
          error_angle = 0;
          prevError_angle = 0;
          p_angle = 0;
          i_angle = 0;
          d_angle = 0;
          pid_angle = 0;
          //Serial.println("Stopped");
        }
      }
      else{
        counter++;
        if (counter > 100){
          digitalWrite(enA, LOW);
          digitalWrite(enB, LOW);
          currTime_angle = 0;
          prevTime_angle = 0;
          error_angle = 0;
          prevError_angle = 0;
          p_angle = 0;
          i_angle = 0;
          d_angle = 0;
          pid_angle = 0;
          Serial.println("No data received");
        }
      }
    }
    else {
      Serial.println("Error in receiving information");
      digitalWrite(enA, LOW);
      digitalWrite(enB, LOW);
      currTime_angle = 0;
      prevTime_angle = 0;
      error_angle = 0;
      prevError_angle = 0;
      p_angle = 0;
      i_angle = 0;
      d_angle = 0;
      pid_angle = 0;
    }
    http.end();
  }
}
