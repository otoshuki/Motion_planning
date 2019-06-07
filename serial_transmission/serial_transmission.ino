void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  String dist;
  String dir;
  String angle;
  // put your main code here, to run repeatedly:
  while (Serial.available()){
    dist = Serial.readStringUntil(',');
    dir = Serial.readStringUntil(',');
    angle = Serial.readStringUntil('\n');
  }
  if (dist.length() > 0){
    Serial.print("Dist : ");
    Serial.print(dist);
    Serial.print("\tDire : ");
    Serial.print(dir);
    Serial.print("\tAngle : ");
    Serial.println(angle);
  }
}
