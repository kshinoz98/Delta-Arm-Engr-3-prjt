<p align="center">
  <img width="460" height="300" src="https://j.gifs.com/n5ZPN5.gif">
 <h1 align="center">Delta Arm Project</h1>
 
 ### Our idea for the robot arm project was to make a delta arm that is precise enough to play the game operation and as a reach goal, possibly faster and more  consistent than a human

# But what is a Delta arm?
A delta robot is a type of parallel robot that consists of three arms connected to universal joints at the base. The key design feature is the use of parallelograms in the arms, which maintains the orientation of the end effector ([According to Wikipedia](https://en.wikipedia.org/wiki/Delta_robot)). It really isn't all that, it's just:
<p align="center">
  <img height="200" src="https://thumbs.gfycat.com/ElasticUnderstatedErin-max-1mb.gif">
 
  and anything that sort of looks like that.

# **Build**
  (more like advanced copy and paste)
## CAD
 <p align="center">
  <img height="300" src="https://user-images.githubusercontent.com/113209502/218620330-2e0810e5-90f8-4238-860c-04ab3687f134.gif">

There's not too much to say on the CAD except that I would advise anyone building something like this pay a lot of attention to what joints have what freedoms(watch a few videos of a delta arm moving). Also, the platform should not be able rotate in any direction.
## Our Build
   ### The Base
   Our robot arm was obviously going to be too large to economically make out of laser cut or 3d printed pieces, so we used bar pieces from a robotics kit, and sliced up an old table to make the top.
   Insert image here of bar base and then table base
   Here is it without the top,                     and with the top
   
   Perhaps it was a bit excessive, but it was pretty fun doing some woodworking.
   
  ### Next, the Arms
   I messed up on the arms. Repeatedly. They still aren't the greatest things I've ever built, and I would honestly tell anybody interested in building a delta arm not to do it my way, because it causes too many issues. 
   * First, I messed up the joints **(remember to check interference in all positions)** and had to print different and objectively worse joints to make up for it.
   <p align="center">
  <img height="200" src="https://user-images.githubusercontent.com/113209502/218626945-7cb06c45-8562-42d0-8417-6a1eda3e8425.jpg">

   * Second, my design included the use of tape, which my teachers were not happy to see (In not happy I mean they made me change my design). 
     
   <p align="center">
  <img height="200" src="https://user-images.githubusercontent.com/113209502/218627678-261fce9e-642a-4f94-9718-0a46d1b63237.jpg">
       
   * Thirdly, I used a bad design making use of a wiggly joint, which is, well, too wiggly. And recently I learned that I didn't even need to use those bad joints at all. 
     
   <p align="center">
  <img height="200" src="https://user-images.githubusercontent.com/113209502/218628470-b8454481-36d0-4698-8cd9-ebe9539b9f6d.png">

       
   * Lastly, I didn't do the right fit on the connection to the stepper, which took me an appalling amount of time to fix.
   
   <p align="center">
  <img height="200" src="https://user-images.githubusercontent.com/113209502/218629013-4e0aa868-9c2f-4b7b-b0b8-40ae7e51b0fb.png">
     
# But... The Problem to end them all.
     
We did not finish our project. And it was not because the code didn't work, it was only partially because the build didn't work, but the main cause of the problem was that the stepper was too weak. Well yes, we could have anticipated this, and somebody had brought it up to us earlier, but we didn't really realize until everything was just about done and we could finally make it move. It moved, but then the motor didn't have enough power and it fell. :( And this was the last week before it was due, so though we do have a solution (add a gearbox to the stepper to give it torque), it's going to be overdue. 

# CODE
## The original idea for the code was a three step process.
  
### ONE
The code recieves inputs and maps the potentiometer values.
  
### TWO
The code uses the DeltaKinematics library and calculates Theta 1, 2, and 3 from X, Y, and Z.
  
### THREE
The code turns Theta 1, 2 and 3 into steps for the stepper, then steps the difference between the previous and current output.
```CPP
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
# Problems to prevent
### *Using pins 1 and 0 on the Arduino Uno
  The arduino uno is equipped with 14 pins, from 0 - 13, it also has 6 analog inputs. Pins 0 and 1 on the Arduino Uno are used for serial communication with the computer over the USB cable. Connecting things to these pins can cause interference between the use of serial and whatever other use you are making of them, that includes being able to upload sketches to the Arduino board.
### *H-Bridge shorting detection
  For me, my H-Bridge created some magic smoke. I burned about 4 or 5 H-Bridges this project. To tell more subtle changes, you can look at the bottom , the black square may have a few extruding dots. That means its done for. Another way to tell is always heat.
### *Converting Variables
  In the code, you'll notice lines like
  ```CPP
   Serial.println(String(p1) + "," + String(p2) + "," + String(p3));
  ```
  ```CPP
  step1Rot = int((DK.a));
  ```
  This is just converting variables. A string con be any combination of characters, the computer can print that easier on the serial monitor. An integer is always a whole number, that means that by converting DK.a to an integer we can assure that the stepper motor can step that many steps and not try a half step without micro stepping equipment.
### *Arm length vs Stepper strength
# The micro stepping fail
  ## If you're going to try micro stepping I recommend three things:
  ### [Use this library](https://github.com/uivc/A4982StepperDriver)
  ### [Use A4982's(Part of the library)](https://www.allegromicro.com/-/media/files/datasheets/a4982-datasheet.pdf)
  ### Don't short them(and how)
  I had access to four stepper A4982 stepper drivers. The first one happened to disapear... in a ball of smoke, but the others can be used to teach good lessons on what not to do. The second stepper driver said that the capacity was 12v for the driver, I cranked the power supply to 12v and nothing happened, slightly over and nothing happened. After the smoke fell, the problem was in the wiring of the motor, not the driver. Double check your wiring. The third was a code problem, I had ENBL and STEP mixed up on my code when I created my A4982 object, that means that the second the code tried to stop the motor, it sent a constant 12v into my STEP pin. Another one bites the dust. The last one taught me my final lesson on drivers and ultimatly forced me to drop micro stepping. I connected the blue and red rails on my breadboard backwards, initially it was a joke, until my teacher and I realized that 12v and GND dont mix very well. That not only ended my last A4982 but also an Arduino.
# Helpful sources
https://www.marginallyclever.com/other/samples/fk-ik-test.html
