
#include <SoftwareSerial.h>

SoftwareSerial Monitor(3, 4); //3 se conecta al rx de arduino y 4 a tx del arduino

void setup() {
  
Monitor.begin(9600); 
pinMode(0, OUTPUT);
pinMode(4, OUTPUT);
pinMode(3, INPUT);
}

void loop() {
  digitalWrite(0, HIGH);
  Monitor.println("LED on");
  delay(1000);
  digitalWrite(0, LOW);
  Monitor.println("LED off");
  delay(1000);
}
