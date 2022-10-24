// Connect SDA and SDC (ATTINY85 physical pins 5 & 7) to the MPU6050 and the SSD1306 OLED Display
// Connect Vcc/Gnd to MPU6050 and SSD1306 OLED Display  (if you are using the 128x64 display, comment out "setdisplay()"
// If you only want to use Serial Monitor OR OLED display, comment out calls to libraries, functions and routines which you don't require
//
// Tom Donnelly 2018

#include <TinyWireM.h>
//#include <TinyOzOLED.h>
#include <SoftwareSerial.h>
 #define DEBUG 1  // - uncomment this line to display accel/gyro values
#ifdef DEBUG
#endif

int accelX, accelY, accelZ;
int gyroX, gyroY, gyroZ;
int gyroXold, gyroYold, gyroZold;
char mpu = 0x68;  // I2C address of MPU.  Connect 5V to pin ADO to use 0x69 address instead

SoftwareSerial Monitor(-1, 4);  // We will only use Tx on PortB 4

void setup() {
  pinMode(1, OUTPUT);
  
  Monitor.begin(9600);
  TinyWireM.begin();
//  OzOled.init(); 
//  setDisplay();  // set display to 128x32px.  Comment out of you're using 128x64 display
//  OzOled.printString("Bond Gyro");
//  delay(1000); // Display title
//  OzOled.clearDisplay();
  // We need to do three things.  1. Disable sleep mode on the MPU (it activates on powerup).  2. Set the scale of the Gyro.  3. Set the scale of the accelerometer
  // We do this by sending 2 bytes for each:  Register Address & Value
  TinyWireM.beginTransmission(mpu); 
  TinyWireM.write(0x6B); //  Power setting address
  TinyWireM.write(0b00000000); // Disable sleep mode (just in case)
  TinyWireM.endTransmission();
  TinyWireM.beginTransmission(mpu); 
  TinyWireM.write(0x1B); // Config register for Gyro
  TinyWireM.write(0x00000000); // 250° per second range (default)
  TinyWireM.endTransmission();
  TinyWireM.beginTransmission(mpu); //I2C address of the MPU
  TinyWireM.write(0x1C); // Accelerometer config register
  TinyWireM.write(0b00000000); // 2g range +/- (default)
  TinyWireM.endTransmission();
}

void loop() {
  getAccel();
  getGyro();
//  if (shaken()) {
//    Monitor.print((long)millis());
//    Monitor.println(" Shaken..");
//    digitalWrite(1, HIGH);
//    delay(100);
//    digitalWrite(1, LOW);
////    OzOled.printNumber((long)millis(), 0, 1);    
////    OzOled.printString("Shaken..", 0, 2);
//    
//    #ifdef DEBUG
//    Monitor.print("accel X = ");
//    Monitor.println((long)accelX);
//    Monitor.print("accel Y = ");
//    Monitor.println(accelY);
//    Monitor.print("accel Z = ");
//    Monitor.println(accelZ);
////    OzOled.printNumber((long)accelX, 0, 0);
////    OzOled.printNumber(accelY, 0, 1);
////    OzOled.printNumber(accelZ, 0, 2);
////    OzOled.printNumber((long)millis(), 0, 3);
//    #endif
//    
//  }
//  if (stirred()) {
//    Monitor.print((long)millis());
//    Monitor.println(" Stirred..");
//    digitalWrite(1, HIGH);
//    delay(100);
//    digitalWrite(1, LOW);
//    
//    #ifdef DEBUG
//    Monitor.print("gyro X = ");
//    Monitor.println(gyroX);
//    Monitor.print("gyro Y = ");
//    Monitor.println(gyroY);
//    Monitor.print("gyro Z = ");
//    Monitor.println(gyroZ);
////    OzOled.printNumber((long)gyroX, 0, 0);
////    OzOled.printNumber(gyroY, 0, 1);
////    OzOled.printNumber(gyroZ, 0, 2);
////    OzOled.printNumber((long)millis(), 0, 3);
//    #endif
//    
//  }

if (x_axys_accel_pos()) {
    Monitor.print((long)millis());
    Monitor.println("x_axys_accel pos");
    digitalWrite(1, HIGH);
    delay(100);
    digitalWrite(1, LOW);
    
    #ifdef DEBUG
    Monitor.print("accel X = ");
    Monitor.println((long)accelX);
    #endif
}  

if (x_axys_accel_neg()) {
    Monitor.print((long)millis());
    Monitor.println("x_axys_accel neg");
    digitalWrite(1, HIGH);
    delay(100);
    digitalWrite(1, LOW);
    
    #ifdef DEBUG
    Monitor.print("accel X = ");
    Monitor.println((long)accelX);
    #endif
}  
  
//  delay(100);
  }

void getAccel() {
  TinyWireM.beginTransmission(mpu); //I2C address of the MPU
  TinyWireM.write(0x3B); //  Acceleration data register
  TinyWireM.endTransmission();
  TinyWireM.requestFrom(mpu, 6); // Get 6 bytes, 2 for each DoF
  accelX = TinyWireM.read() << 8; // Get X upper byte first
  accelX |= TinyWireM.read();     // lower
  accelY = TinyWireM.read() << 8; // Get Y upper byte first
  accelY |= TinyWireM.read();     // lower
  accelZ = TinyWireM.read() << 8; // Get Z upper byte first
  accelZ |= TinyWireM.read();     // lower
//    Monitor.print("accel X = ");
//    Monitor.println((long)accelX);
//    Monitor.print("accel Y = ");
//    Monitor.println(accelY);
//    Monitor.print("accel Z = ");
//    Monitor.println(accelZ);
}

void getGyro() {
  TinyWireM.beginTransmission(mpu); //I2C address of the MPU
  TinyWireM.write(0x43); // Gyro data register
  TinyWireM.endTransmission();
  TinyWireM.requestFrom(mpu, 6); // Get 6 bytes, 2 for each DoF
  while (TinyWireM.available() < 6);
  gyroX = TinyWireM.read() << 8; // Get X upper byte first
  gyroX |= TinyWireM.read();     // lower
  gyroY = TinyWireM.read() << 8; // Get Y upper byte first
  gyroY |= TinyWireM.read();     // lower
  gyroZ = TinyWireM.read() << 8; // Get Z upper byte first
  gyroZ |= TinyWireM.read();     // lower
}

bool shaken() {
  if ((abs(accelX) > 20000) || (abs(accelY) > 20000) ||  (abs(accelZ) > 32760)) {
  return true;
  }
  else return false;
  }

bool x_axys_accel_pos(){
  if(accelX > 5000){
    return true;
  }
  else return false;
}
bool x_axys_accel_neg(){
  if(accelX < -5000){
    return true;
  }
  else return false;
}

bool stirred() {
  gyroXold = gyroX;    // Save current Gyro settings...
  gyroYold = gyroY;
  gyroZold = gyroZ;
  getGyro();  // get a second reading to compare with the last to see if we're moving
  //  300 is just a number to filter noise-level fluxuations .. DYOR
  if (((gyroX - gyroXold) > 300) || ((gyroY - gyroYold) > 300) ||  ((gyroZ - gyroZold) > 300)) {
    return true;
  }
  else return false;
}

//void setDisplay () { // 128x32px OLED settings
//  OzOled.sendCommand(0xA8); // Multiplexer
//  OzOled.sendCommand(0x1F);
//  OzOled.sendCommand(0xDA); // Com Pins
//  OzOled.sendCommand(0x02);
//}
