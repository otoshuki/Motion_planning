//Robotics Club - Motion Planning System
//Guining Pertin - 05-06-2019
//Arduino control code

//Motor driver connections
int enA = 5;
int in1 = 6;
int in2 = 7;
int in3 = 8;
int in4 = 9;
int enB = 10;

//LForward - in1 - LOW & in2 - HIGH
//RForward - in3 - LOW & in4 - HIGH
void setup() {
  //Set pin I/O mode
  for (int pin = 5; pin <= 10; pin++) {
    pinMode(pin, OUTPUT);
  }
  Serial.begin(9600); //Serial at 9600 baud rate  
}

void loop() {
  //Left forward
  digitalWrite(enA, HIGH);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  //Right forward
  digitalWrite(enB, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}
