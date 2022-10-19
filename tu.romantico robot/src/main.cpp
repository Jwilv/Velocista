#include <Arduino.h>
#include <SoftwareSerial.h>

SoftwareSerial bt(11,10);

String in;
String hola = "h";
void setup() {
  Serial.begin(9600);
  Serial.println("listo");
  bt.begin(9600);
}

void loop() {
  if(bt.available()){
    Serial.write(bt.read());
     in = bt.readString();
      in.trim();
    if(in == "h") {
      bt.println("todo salio bien");
      Serial.println("todo salio bien");
    }
  } 
   
  if(Serial.available()) bt.write(Serial.read());
  
  

  

}
