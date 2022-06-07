#include <Wire.h>
long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ,elapsedTime;
long gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;
double x,y,z,dx,dy,dz,ex,ey,ez,desx=0,desy=0,desz=0;
float PID, PWM, error, previous_error,timePrev,time1;
float pid_p;
float pid_i;
float pid_d;
float rad_to_deg = 180/3.141592654;
/////////////////PID CONSTANTS/////////////////
double kp=2;
double ki=0;
double kd=0;
///////////////////////////////////////////////
// motor one
int enA = 9;
int in1 = 7;
int in2 = 6;
// motor two
int enB = 10;
int in3 = 5;
int in4 = 4;
///////////////////////////////////////////////
int led = 9;           // the PWM pin the LED is attached to
int brightness = 0;    // how bright the LED is
int fadeAmount = 5;
void setup() 
{
  Serial.begin(115200);
  Wire.begin();
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  setupMPU();
  delay(500);
}


void loop() 
{
    timePrev = time1;  // the previous time is stored before the actual time read
    time1 = millis();  // actual time read/
    elapsedTime = (time1 - timePrev) / 1000; 
    recordAccelRegisters();
    recordGyroRegisters();
   processangle();
    Apply_PID();
    delay(100);
}
void right(int PWM)
{  
  PWM=map(PWM,0,-50,0,255);
  Serial.println(PWM);
  // turn on motor A
  analogWrite(enA, PWM);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  // set speed to 200 out of possible range 0~255
  analogWrite(enB, PWM); 
  // turn on motor B
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  // set speed to 200 out of possible range 0~255
 
  Serial.println("Backward Movement");
}

void left(int PWM)
{
  PWM=map(PWM,0,50,0,255);
  Serial.println(PWM);
  // turn on motor A
    analogWrite(enA, PWM);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  // set speed to 200 out of possible range 0~255
analogWrite(enB, PWM);
  // turn on motor B
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  // set speed to 200 out of possible range 0~255 
  Serial.println("Forward Movement");
}
void stopper()
{
   digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);  
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  }
void setupMPU()
{
  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();  
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s 
  Wire.endTransmission(); 
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5) 
  Wire.write(0b00000000); //Setting the accel to +/- 2g
  Wire.endTransmission(); 
}

void recordAccelRegisters() 
{
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
  while(Wire.available() < 6);
  accelX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  accelY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  accelZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processAccelData();
}

void processAccelData()
{
  gForceX = accelX / 16384.0;
  gForceY = accelY / 16384.0; 
  gForceZ = accelZ / 16384.0;
  x= RAD_TO_DEG * (atan2((-gForceZ),sqrt(square(-gForceX)+square(-gForceY))));
  y= RAD_TO_DEG * (atan2((-gForceX),sqrt(square(-gForceY)+square(-gForceZ))));
  z= RAD_TO_DEG * (atan(sqrt(square(-gForceY) + square(-gForceX)) / -gForceZ));
 /* x=map(x,340,11,-90,90);*/
  Serial.print("Raw AccelX=");
Serial.print(y);
}

void recordGyroRegisters() 
{
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 - 48)
  while(Wire.available() < 6);
  gyroX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  gyroY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  gyroZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processGyroData();
}

void processGyroData() 
{
  rotX = (gyroX / 131.0);
  rotY = gyroY / 131.0; 
  rotZ = gyroZ / 131.0;
  Serial.print("Raw Gyro=");
  Serial.println(rotY*elapsedTime);
}
void processangle()
{
  dx=0.98 *(dx + rotX*elapsedTime) + 0.02*x;
  dy=0.98 *(dy + rotY*elapsedTime) + 0.02*y;
  dz=0.98 *(dz + rotZ*elapsedTime) + 0.02*z;
  ex=dx-desx;
  ey=dy-desy;
  ez=dz-desz;
  Serial.print("Filter Angle=");
  Serial.println(ey);
}

void Apply_PID()
{ 
  float l=(float)ey;
  pid_p=kp*l;
  if(-5<l<5)
  pid_i = pid_i+(ki*l);  
pid_d = kd *((l - previous_error)/elapsedTime);
PID = pid_p + pid_i + pid_d;

/*Serial.print("I=");
Serial.println(pid_d);
Serial.print("p=");
Serial.println(pid_p);*/
Serial.print("PID");
/*Serial.println(PID);*/

    if(l<0)
    right(PID);
    else
    left(PID);
previous_error = l;
}
