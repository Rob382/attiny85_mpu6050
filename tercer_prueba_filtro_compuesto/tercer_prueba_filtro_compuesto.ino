// Connect SDA and SDC (ATTINY85 physical pins 5 & 7) to the MPU6050 and the SSD1306 OLED Display
// Connect Vcc/Gnd to MPU6050 and SSD1306 OLED Display  (if you are using the 128x64 display, comment out "setdisplay()"
// If you only want to use Serial Monitor OR OLED display, comment out calls to libraries, functions and routines which you don't require
//
// Tom Donnelly 2018

#include <TinyWireM.h>
#include <SoftwareSerial.h>
 #define DEBUG 1  // - uncomment this line to display accel/gyro values
#ifdef DEBUG
#endif

int accelX, accelY, accelZ;
int gyroX, gyroY, gyroZ;
int gyroXold, gyroYold, gyroZold;
char mpu = 0x68;  // I2C address of MPU.  Connect 5V to pin ADO to use 0x69 address instead

long tiempo_prev;
float dt;
float ang_x, ang_y;
float ang_x_prev = 0, ang_y_prev = 0;
float accel_ang_x; 
float accel_ang_y;

SoftwareSerial Monitor(-1, 4);  // We will only use Tx on PortB 4

void setup() {
  pinMode(1, OUTPUT);
  
  Monitor.begin(9600);
  TinyWireM.begin();

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
  updateFiltered();
   Monitor.print(F("Inclinacion en X: "));
   Monitor.print(accel_ang_x);
   Monitor.print(F("\tInclinacion en Y:"));
   Monitor.println(accel_ang_y);
   Monitor.print(F("Rotacion en X:  "));
   Monitor.print(ang_x);
   Monitor.print(F("\t Rotacion en Y: "));
   Monitor.println(ang_y);
   Monitor.print(F("gyro X:  "));
   Monitor.print(gyroX);
   Monitor.print(F("\t gyro Y: "));
   Monitor.println(gyroY);
   delay(10);

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

void updateFiltered()
{
   dt = (millis() - tiempo_prev) / 1000.0;
   tiempo_prev = millis();
   //Calcular los ángulos con acelerometro
   accel_ang_x = atan(accelY / sqrt(pow(accelX, 2) + pow(accelZ, 2)))*(180.0 / 3.14);
   accel_ang_y = atan(-accelX / sqrt(pow(accelY, 2) + pow(accelZ, 2)))*(180.0 / 3.14);
   //Calcular angulo de rotación con giroscopio y filtro complementario
   ang_x = (0.98*(ang_x_prev + (gyroX / 131)*dt) + (accel_ang_x / 50));
   ang_y = (0.98*(ang_y_prev + (gyroY / 131)*dt) + (accel_ang_y / 50));
   ang_x_prev = ang_x;
   ang_y_prev = ang_y;
}

bool shaken() {
  if ((abs(accelX) > 20000) || (abs(accelY) > 20000) ||  (abs(accelZ) > 32760)) {
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
