<p align="center">
  <img width="460" height="300" src="https://j.gifs.com/n5ZPN5.gif">
 <h1 align="center">Delta Arm Project</h1>
 
 ### Our idea for the robot arm project was to make a delta arm that is precise enough to play the game operation and as a reach goal, possibly faster and more  consistent than a human

# But what is a Delta arm?
A delta robot is a type of parallel robot that consists of three arms connected to universal joints at the base. The key design feature is the use of parallelograms in the arms, which maintains the orientation of the end effector ([According to Wikipedia](https://en.wikipedia.org/wiki/Delta_robot)). It really isn't all that, it's just:
<p align="center">
  <img height="200" src="https://thumbs.gfycat.com/ElasticUnderstatedErin-max-1mb.gif">
 
  and anything that sort of looks like that.
  
# CODE
## The original idea for the code was a three step process.
  
### ONE
The code recieves inputs and maps the potentiometer values.
  
### TWO
The code uses the DeltaKinematics library and calculates Theta 1, 2, and 3 from X, Y, and Z.
  
### THREE
The code turns Theta 1, 2 and 3 into steps for the stepper, then steps the difference between the previous and current output.
```C++
//I've included some serial.prints just in case you dont want to write it yourself, these are all the prints I had when evaluating the final project

#include <DeltaKinematics.h>//importing libraries
#include <Arduino.h>
#include <math.h>
#include <Stepper.h>

//if you try to use both 1 and 0 pins you'll get an error, "Attpmpt 10/10". Don't do that.
#define MOTOR_STEPS 200//amount of steps your motor can take(Base amount, if micro stepping, keep using this amt)

#define Pot1 A3//defining pins
#define Pot2 A4
#define Pot3 A5

#define AIN1A 2
#define AIN2A 3
#define BIN1A 12
#define BIN2A 13

Stepper stepper1(MOTOR_STEPS, AIN1A, AIN2A, BIN1A, BIN2A);//creating Stepper as an object

DeltaKinematics DK(200, 300, 25, 10);//measurments of the Delta arm, making it an object. Measurment order:  DeltaKinematics(double ArmLength,double RodLength,double BassTri,double PlatformTri)


int step1Rot = 0;//definings variables
int step2Rot = 0;
int step3Rot = 0;

int b1Amt = 0;
int b2Amt = 0;
int b3Amt = 0;

int a1Amt = 0;
int a2Amt = 0;
int a3Amt = 0;


void setup()
{
  Serial.begin(9600);//starting serial monitor
  stepper1.setSpeed(60);//setting the RotationsPerMinute on the stepper
}

void loop() {

  Serial.println("XYZ Print VALUES:");
  XYZ();

  Serial.println("reoundDegrees Print VALUES:");
  roundDegrees();

  Serial.println("stepperMove Print VALUES:");
  stepperMove();
  //moves stepper

  delay(1000);
}


void stepperMove() {
  Serial.println(String(b1Amt) + "," + String(a1Amt));

  int d1 = -1 * (b1Amt - a1Amt);//final math to find differnce in values and step the difference

  Serial.println(String(d1) + ",");

  stepper1.step(d1);//stepping motors
}



void XYZ() {

  int p1 = analogRead(Pot1);//input from potentiometers
  int p2 = analogRead(Pot2);
  int p3 = analogRead(Pot3);

  DK.x = map(p1, 0, 1024, -130, 130);//mapping inputs
  DK.y = map(p2, 0, 1024, -130, 140);
  DK.z = map(p3, 0, 1024, -430, -160);

  DK.inverse();//This function is part of the DK library, its whole job is to turn X, Y, and Z into THETA1 THETA2 and THETA3

  Serial.println(String(p1) + "," + String(p2) + "," + String(p3));
  Serial.println(String(DK.x) + "," + String(DK.y) + "," + String(DK.z));
  Serial.println(String(DK.a) + "," + String(DK.b) + "," + String(DK.c));

}



//idea for before and After amts comes from this exe code
void roundDegrees() {//this whole block of code is here because otherwise the motors dont know what to do with a valuse thats not on their list of allowed values(Whole numbers from 1 to MOTOR_STEPS)
  //therefore we multiply the variable by 200 and truncate its value. this makes it devisable by 200, then we divide it by 200.

  b1Amt = step1Rot;
  b2Amt = step2Rot;
  b3Amt = step3Rot;

  step1Rot = int((DK.a));
  step2Rot = int((DK.b));
  step3Rot = int((DK.c));
  
  a1Amt = step1Rot;
  a2Amt = step2Rot;
  a3Amt = step3Rot;
}
```
# Delta Arm Kinematics
The Delta arm uses kinematics to figure out how much the arms have to bend to locate the end arm position. I looked at this project optomistically by expecting myself to figure these out. I couldn't, but I found the useful library [Delta Kinematics](https://github.com/tinkersprojects/Delta-Kinematics-Library) which explains exactly how a delta arm works and how his calculations save you time.
# Wiring
Wiring involves three steppers(NEMA 17), three stepper motor drivers(DRV8833), three potentiometers, a switch, a button, and an LED if you want. We can hard wire a LED be attaching a 220 omh resistor from the short end of the LED to ground, and the long end to 3.3v or 5v. The potentiometers can have all of their grounds and 5v connected to save space and pins, it also decreases the possobility of a connection issue. The switch connects from a 9v battery to the  VM pin on the arduino. The steppers I used have four colors red, grey, green, and yellow. Look at image for reference.
