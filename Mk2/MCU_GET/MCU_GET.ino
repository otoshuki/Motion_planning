#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
 
const char* ssid = "duncan";
const char* password = "11111111";
 
void setup () {
 
Serial.begin(115200);
WiFi.begin(ssid, password);
 
while (WiFi.status() != WL_CONNECTED) {
 
  delay(1000);
  Serial.println("Connecting..");
 
}
Serial.println("Connected");
 
}
 
void loop() {
  String dist;
  String angle;
  String dir;
 
  if (WiFi.status() == WL_CONNECTED) { //Check WiFi connection status
   
    HTTPClient http;  //Declare an object of class HTTPClient
     
    http.begin("http://192.168.43.135:8081/data.txt");  //Specify request destination
    int httpCode = http.GET(); //Send the request
     
    if (httpCode > 0) { //Check the returning code
     
      String payload = http.getString();   //Get the request response payload
      if (payload == "") Serial.println("No data");
      else {
        //Serial.println(payload);                     //Print the response payload
        dist = payload.substring(0, payload.indexOf('+'));
        Serial.print(dist);
        Serial.print(",");
        angle = payload.substring(payload.indexOf('+')+1, payload.indexOf('-'));
        Serial.print(angle);
        Serial.print(",");
        dir = payload.substring(payload.indexOf('-')+1, payload.indexOf('*'));
        Serial.println(dir);
    
      }

    }
    else {
      Serial.println("No data");
    }
     
    http.end();   //Close connection
   
  }
   
  //delay(1000);    //Send a request every 30 seconds
   
}
