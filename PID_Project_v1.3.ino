#include <Wire.h>
#include <Servo.h>
#include <LiquidCrystal_I2C.h>

Servo right_prop;
Servo left_prop; 

int manualOveridePin = 2,
  kpOnPin = 3,
  kiOnPin = 4,
  kdOnPin = 5;

int manualOverideState = 0,
  kpOnState = 0,
  kiOnState = 0,
  kdOnState = 0;

int potPinA = A0,
  potPinP = A2,
  potPinI = A3,
  potPinD = A1,
  potValueA, potValueP, potValueI, potValueD;

const int MPU_addr = 0x68;

float AcX, AcY, AcZ, GyX, GyY, GyZ;
float AccAngleRoll, RollAngle = 0;
double dt, now, PrevTime;

double error, previous = 0, pwmLeft, pwmRight, proportional, integral = 0, derivative, PID_output = 0;

double kp=1.0,
  ki=0.0040,
  kd=0.40;
int throttle=1300;
float desired_angle = 0;

LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup()
{
  Serial.begin(9600);

  pinMode(manualOveridePin, INPUT);
  pinMode(kpOnPin, INPUT);
  pinMode(kiOnPin, INPUT);
  pinMode(kdOnPin, INPUT);
  pinMode(potPinA, INPUT);
  pinMode(potPinP, INPUT);
  pinMode(potPinI, INPUT);
  pinMode(potPinD, INPUT);

  Wire.begin();
  setupMPU();

  right_prop.attach(10);
  left_prop.attach(9);
  
  left_prop.writeMicroseconds(1000); 
  right_prop.writeMicroseconds(1000);

  delay(7000);
  
  lcd.init();
  lcd.backlight();
}

void loop()
{
  now = millis();
  dt = (now - PrevTime) / 1000.0;
  PrevTime = now; 

  manualOverideState = digitalRead(manualOveridePin);
  if (manualOverideState == LOW)
  {
    ReadPotValues();
  }
  else
  {
    kp = 1.0;
    ki = 0.0040;
    kd = 0.40;
    desired_angle = 0;
  }

  ReadAccValues();
  ReadGyroValues();

  AccAngleRoll = -atan(AcX / sqrt(AcY * AcY + AcZ * AcZ)) * 180 / PI;

  RollAngle = 0.2 *(RollAngle + GyY*dt) + 0.8*AccAngleRoll;

  PrintLCD();

  error = RollAngle - desired_angle;
  PID_output = pid(error);

  PID_output = constrain(PID_output, -1000, 1000);
  
  pwmLeft = throttle - PID_output;
  pwmRight = throttle + PID_output;
  
  pwmLeft = constrain(pwmLeft, 1000, 2000);
  pwmRight = constrain(pwmRight, 1000, 2000);

  left_prop.writeMicroseconds(pwmLeft);
  right_prop.writeMicroseconds(pwmRight);
}

void setupMPU()
{
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
}

void ReadPotValues()
{
  potValueA = analogRead(potPinA);
  desired_angle = map(potValueA, 0, 1022, -80, 80);

  kpOnState = digitalRead(kpOnPin);
  if (kpOnState == LOW)
  {
    potValueP = analogRead(potPinP);
    kp = static_cast<double>(map(potValueP, 0, 1022, 0, 240));
    kp = round(kp) / 100.0;
  }
  else
  {
    kp = 0;
  }
  
  kiOnState = digitalRead(kiOnPin);
  if (kiOnState == LOW)
  {
    potValueI = analogRead(potPinI);
    ki = static_cast<double>(map(potValueI, 0, 1022, 0, 1000));
    ki = round(ki) / 100000.0;
  }
  else
  {
    ki = 0;
  }
  
  kdOnState = digitalRead(kdOnPin);
  if (kdOnState == LOW)
  {
    potValueD = analogRead(potPinD);
    kd = static_cast<double>(map(potValueD, 0, 1022, 0, 150));
    kd = round(kd) / 100.0;
  }
  else
  {
    kd = 0;
  }
}

void ReadAccValues()
{
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(MPU_addr,6);
  AcX=Wire.read()<<8|Wire.read();
  AcY=Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read();

  AcX = AcX / 4096.0-0.02;  
  AcY = AcY / 4096.0+0.035;
  AcZ = AcZ / 4096.0;
}

void ReadGyroValues()
{
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(MPU_addr,6);
  GyX=Wire.read()<<8|Wire.read();
  GyY=Wire.read()<<8|Wire.read();
  GyZ=Wire.read()<<8|Wire.read();

  GyX = GyX / 65.5;  
  GyY = GyY / 65.5;
  GyZ = GyZ / 65.5;
}

void PrintLCD()
{
  lcd.setCursor(0, 0);
  lcd.print("kp=");
  lcd.setCursor(3, 0);
  lcd.print(kp);

  lcd.setCursor(7, 0);
  lcd.print("ki=");
  lcd.setCursor(9, 0);
  lcd.print(ki, 5);
  lcd.setCursor(9, 0);
  lcd.print("=");

  lcd.setCursor(0, 1);
  lcd.print("kd=");
  lcd.setCursor(3, 1);
  lcd.print(kd);

  lcd.setCursor(7, 1);
  lcd.print("DA");
  lcd.setCursor(9, 1);
  lcd.print(desired_angle);
  lcd.setCursor(12, 1);
  lcd.print("/");
  lcd.setCursor(13, 1);
  lcd.print(RollAngle);
}

double pid(double error)
{
  proportional = error;
  integral = (error > -5 && error < 5) ? (integral + error * dt) : 0;
  derivative = (error - previous) / dt;
  previous = error;
  return (kp * proportional) + (ki * integral) + (kd * derivative);
}
