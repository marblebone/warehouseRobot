#include "HUSKYLENS.h"
#include "SoftwareSerial.h"
#include <AFMotor.h>
#include <Servo.h>
#include <NewPing.h>
#include <Wire.h>

AF_DCMotor motor1(1, MOTOR12_1KHZ);
AF_DCMotor motor2(2, MOTOR12_1KHZ);
AF_DCMotor motor3(3, MOTOR34_1KHZ);
AF_DCMotor motor4(4, MOTOR34_1KHZ);

HUSKYLENS huskylens;
void printResult(HUSKYLENSResult result);

Servo servo_left, servo_right, servo_base, servo_grip;
int angleX = 10; // Set the initial angle for x-axis servo
int angleY = 90; // Set the initial angle for y-axis servo
int angleZ = 180;
int angleG = 150;
int speed = 200;
int speedTurn = 255;

#define TRIGGER_PIN 2
#define ECHO_PIN 7
#define MAX_DISTANCE 350
NewPing sonar (TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
float duration, distance;

SoftwareSerial btSerial(0, 1); // RX, TX

struct PacketData 
{
  byte lxAxisValue;
  byte lyAxisValue;
  byte rxAxisValue;
  byte ryAxisValue;
};
PacketData data;

unsigned long lastRecvTime = 0;


void setup() {
    Serial.begin(115200);
    Wire.begin();
    while (!huskylens.begin(Wire))
    {
        Serial.println(F("Begin failed!"));
        Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protocol Type>>Serial 9600)"));
        Serial.println(F("2.Please recheck the connection."));
        delay(100);
    }
    huskylens.writeAlgorithm(ALGORITHM_LINE_TRACKING);
  
    
    servo_base.attach(8);
    servo_base.write(angleX);
    servo_right.attach(10);
    servo_right.write(angleY);
    servo_left.attach(9);
    servo_left.write(angleZ);
    servo_grip.attach(12);
    servo_grip.write(angleG);


    btSerial.begin(38400);

    pinMode(TRIGGER_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
}

void loop() {
  MeasureDistance();
  if(distance >= 10)
    { 
      if (!huskylens.request()) 
    { 
      Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));
    }
    else if(!huskylens.isLearned()) Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));
    else if(!huskylens.available()) 
    {
      Serial.println(F("No block or arrow appears on the screen!"));
      motor1.setSpeed(0);
      motor1.run(RELEASE);
      motor2.setSpeed(0);
      motor2.run(RELEASE);
      motor3.setSpeed(0);
      motor3.run(RELEASE);
      motor4.setSpeed(0);
      motor4.run(RELEASE);
     //SlaveCar();
    }
    else
    {
        Serial.println(F("###########"));
        if (huskylens.available())
        {
            HUSKYLENSResult result = huskylens.read();
            printResult(result);

            int xAxis = result.xCenter;
            int yAxis = result.yCenter;
            int distance = result.height;
    

    Serial.print("xAxis");
    Serial.println(xAxis);
    Serial.print("yAxis");
    Serial.println(yAxis);
    Serial.print("distance");
    Serial.println(distance);
   
    if (xAxis > 240) {
      //Accelerate();
      motor1.setSpeed(speedTurn);
      motor1.run(FORWARD);
      motor4.setSpeed(speedTurn);
      motor4.run(FORWARD);
      motor2.setSpeed(speedTurn);
      motor2.run(BACKWARD);
      motor3.setSpeed(speedTurn);
      motor3.run(BACKWARD);
      //delay(10);
      
    }else if (xAxis < 100) {
      //Accelerate();
      motor2.setSpeed(speedTurn);
      motor2.run(FORWARD);
      motor3.setSpeed(speedTurn);
      motor3.run(FORWARD);
      motor1.setSpeed(speedTurn);
      motor1.run(BACKWARD);
      motor4.setSpeed(speedTurn);
      motor4.run(BACKWARD);
      //delay(10);
      
    
    

    }else if ((xAxis >= 100)&&(xAxis <= 240)) {
      motor1.setSpeed(speed);
      motor1.run(FORWARD);
      motor2.setSpeed(speed);
      motor2.run(FORWARD);
      motor3.setSpeed(speed);
      motor3.run(FORWARD);
      motor4.setSpeed(speed);
      motor4.run(FORWARD);
      //delay(10);

    }else{
      motor1.setSpeed(0);
      motor1.run(RELEASE);
      motor2.setSpeed(0);
      motor2.run(RELEASE);
      motor3.setSpeed(0);
      motor3.run(RELEASE);
      motor4.setSpeed(0);
      motor4.run(RELEASE);
     
    }
  
    
  } else {
    Serial.println("vision body undetected.");
     motor1.setSpeed(0);
      motor1.run(RELEASE);
      motor2.setSpeed(0);
      motor2.run(RELEASE);
      motor3.setSpeed(0);
      motor3.run(RELEASE);
      motor4.setSpeed(0);
      motor4.run(RELEASE);
      }
    }
            
  }
  else if ((distance >=8)&&(distance <10)) {
      motor1.setSpeed(0);
      motor1.run(RELEASE);
      motor2.setSpeed(0);
      motor2.run(RELEASE);
      motor3.setSpeed(0);
      motor3.run(RELEASE);
      motor4.setSpeed(0);
      motor4.run(RELEASE);

      RoboticArm();
  }
  else if(distance < 3){
      motor1.setSpeed(0);
      motor1.run(RELEASE);
      motor2.setSpeed(0);
      motor2.run(RELEASE);
      motor3.setSpeed(0);
      motor3.run(RELEASE);
      motor4.setSpeed(0);
      motor4.run(RELEASE);
  }
}

void printResult(HUSKYLENSResult result){
    if (result.command == COMMAND_RETURN_BLOCK){
        Serial.println(String()+F("Block:xCenter=")+result.xCenter+F(",yCenter=")+result.yCenter+F(",width=")+result.width+F(",height=")+result.height+F(",ID=")+result.ID);
    }
    else if (result.command == COMMAND_RETURN_ARROW){
        Serial.println(String()+F("Arrow:xOrigin=")+result.xOrigin+F(",yOrigin=")+result.yOrigin+F(",xTarget=")+result.xTarget+F(",yTarget=")+result.yTarget+F(",ID=")+result.ID);
    }
    else{
        Serial.println("Object unknown!");

    }
}    

void SlaveCar(){
  String dataString;
  if (btSerial.available())
  {
    dataString = btSerial.readStringUntil('\n');
    sscanf(dataString.c_str(), "%d,%d,%d,%d", &data.lxAxisValue, &data.lyAxisValue, &data.rxAxisValue, &data.ryAxisValue);
    int throttle = map(data.lyAxisValue, 254, 0, -255, 255); //Left stick  - y axis - forward/backward car movement
    int steering = map(data.rxAxisValue, 0, 254, -255, 255); //Right stick - x axis - left/right car movement 
    int motorDirection = 1;
    
    if (throttle < 0)       //Move car backward
    {
      motorDirection = -1;    
    }
  
    int rightMotorSpeed, leftMotorSpeed;
    rightMotorSpeed =  abs(throttle) - steering;
    leftMotorSpeed =  abs(throttle) + steering;
    rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);
    leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
  
    rotateMotor(rightMotorSpeed * motorDirection, leftMotorSpeed * motorDirection);
    lastRecvTime = millis();      
  }
  else
  {
    unsigned long now = millis();
    if ( now - lastRecvTime > 1000 )       //Signal lost after 1 second. Reset the motor to stop
    {
      rotateMotor(0, 0);   
   }
 }      
}
void rotateMotor(int rightMotorSpeed, int leftMotorSpeed)
{
  if (rightMotorSpeed < 0)
  {
    motor1.run(BACKWARD);
    motor4.run(BACKWARD);
  }
  else if (rightMotorSpeed > 0)
  {      
    motor1.run(FORWARD);
    motor4.run(FORWARD);
  }
  else
  {    
    motor1.run(RELEASE);
    motor4.run(RELEASE); 
  }
  
  if (leftMotorSpeed < 0)
  {
    motor2.run(BACKWARD);
    motor3.run(BACKWARD);     
  }
  else if (leftMotorSpeed > 0)
  {     
    motor2.run(FORWARD);
    motor3.run(FORWARD);
  }
  else
  {  
    motor2.run(RELEASE);
    motor3.run(RELEASE);    
  }  
  
}

void MeasureDistance(){
  duration = sonar.ping();
  distance = duration * 0.034 /2;
  Serial.print("Distance = ");
  Serial.print(distance);
  Serial.println(" cm");

  delay(50);
  return distance;

}

void Accelerate(){
      motor1.setSpeed(speed);
      motor1.run(FORWARD);
      motor2.setSpeed(speed);
      motor2.run(FORWARD);
      motor3.setSpeed(speed);
      motor3.run(FORWARD);
      motor4.setSpeed(speed);
      motor4.run(FORWARD);
      delay(25);
}

void RoboticArm(){
  for(angleX=10; angleX<=180; angleX++)
  {
    servo_base.write(angleX);
    delay(15);
  }

  for(angleY=90; angleY<=150; angleY++)
  {
    servo_right.write(angleY);
    delay(15);
  }

  for(angleZ=180; angleZ>=110; angleZ--)
  {
    servo_left.write(angleZ);
    delay(15);
  }

  for(angleG=150; angleG>=20; angleG--)
  {
    servo_grip.write(angleG);
    delay(15);
  }

  for(angleZ=110; angleZ<=180; angleZ++)
  {
    servo_left.write(angleZ);
    delay(15);
  }

  for(angleY=150; angleY>=90; angleY--)
  {
    servo_right.write(angleY);
    delay(15);
  }

  for(angleX=180; angleX>=20; angleX--)
  {
    servo_base.write(angleX);
    delay(15);
  }

   for(angleZ=180; angleZ>=170; angleZ--)
  {
    servo_left.write(angleZ);
    delay(15);
  }

  for(angleG=20; angleG<=150; angleG++)
  {
    servo_grip.write(angleG);
    delay(15);
  }
   
  for(angleZ=170; angleZ<=180; angleZ++)
  {
    servo_left.write(angleZ);
    delay(15);
  }
   
}



