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
    //Simple system now
    if (dist.toInt()> 20){
      digitalWrite(enA, HIGH);
      digitalWrite(enB, HIGH);
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      //Serial.println("Moving");
    }
    else{
      digitalWrite(enA, LOW);
      digitalWrite(enB, LOW);
      //Serial.println("Stopped");
    }
  }
  else{
    counter++;
    if (counter > 2000){
      digitalWrite(enA, LOW);
      digitalWrite(enB, LOW);
      //Serial.println("No data received");
    }
  }
}
