//Robotics Club - Motion Planning System
//Guining Pertin - 05-06-2019
//Arduino control code

//Motor driver connections
int enA = 5; //Left
int in1 = 6;
int in2 = 7;
int in3 = 8;
int in4 = 9;
int enB = 10; //Right
int counter = 0;

//LForward - in1 - LOW & in2 - HIGH
//RForward - in3 - LOW & in4 - HIGH

//PID coeffs
float kp_angle = 1.3;
float ki_angle = 5;
float kd_angle = 110;
float kp_dist = 1.8;
float ki_dist = 100;
float kd_dist = 50;
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
  //Set pin I/O mode
  for (int pin = 5; pin <= 10; pin++) {
    pinMode(pin, OUTPUT);
  }
  digitalWrite(enA, HIGH);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(enB, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  Serial.begin(9600); //Serial at 9600 baud rate  
}

void loop() {
  //Serial data
  String dist;
  String dir;
  String angle;
  //Read serial data as string
  while (Serial.available()){
    dist = Serial.readStringUntil(',');
    dir = Serial.readStringUntil(',');
    angle = Serial.readStringUntil('\n');
    counter = 0;
  }
  //Control system
  if (dist.length() > 0){
    //Check angular motion first
    if (angle.toInt() > 8){
      currTime_angle = millis();
      //Turn off linear motion PID
      int currTime_dist = 0;
      int prevTime_dist = 0;
      //PID control for angle error
      error_angle = angle.toInt();
      p_angle = kp_angle * error_angle;
      //i_angle += ki_angle * error_angle*(currTime_angle-prevTime_angle);
      d_angle = kd_angle * (error_angle - prevError_angle)/(currTime_angle-prevTime_angle);
      pid_angle = p_angle + d_angle;
      //Set PID error limits
      if (pid_angle > 255) pid_angle = 255;
      else if (pid_angle < 130) pid_angle = 130;
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
      int currTime_angle = 0;
      int prevTime_angle = 0;
      //PID control for dist error
      error_dist = dist.toInt();
      p_dist = kp_dist * error_dist;
      //i_angle += ki_angle * error_angle*(currTime_angle-prevTime_angle);
      d_dist = kd_dist * (error_dist - prevError_dist)/(currTime_dist-prevTime_dist);
      pid_dist = p_dist + d_dist;
      //Set PID error limits
      if (pid_dist > 255) pid_dist = 255;
      else if (pid_dist < 130) pid_dist = 130;
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
    if (counter > 2000){
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
      //Serial.println("No data received");
    }
  }
}
