//add servo library
#include <Servo.h>

#define LEFT 11   //  pin 11 is connected to left button
#define RIGHT 12  //  pin 12 is connected to right button
#define WRIST 9
#define SHOULDER 6
#define GRIPPER 3
#define GRIPPER_ROTATION 5
#define BASE_ROTATION 10

//define our servos
Servo gripper;
Servo griprot;
Servo shoulder;
Servo wrist;
Servo baserot;

int angle = 70;    // initial angle  for servo
int angleStep = 2;

//define our potentiometers
int pot1 = A0; //VRx joystick 1
int pot2 = A1; //VRy joystick 1
int pot3 = A2; //VRx joystick 2
int pot4 = A3; //VRy joystick 2

//variable to read the values from the analog pin (potentiometers)
int valPot1;
int valPot2;
int valPot3;
int valPot4;

void setup()
{
  Serial.begin(9600);  //  setup serial

  pinMode(LEFT, INPUT_PULLUP); // assign pin 11 ass input for Left button
  pinMode(RIGHT, INPUT_PULLUP); // assing pin 12 as input for right button
  pinMode(A5, INPUT_PULLUP); //center servo's pin

  //attaches our servos on pins PWM 11-10-9-6 to the servos
  gripper.attach(GRIPPER);
  griprot.attach(GRIPPER_ROTATION);
  shoulder.attach(SHOULDER);
  wrist.attach(WRIST);
  baserot.attach(BASE_ROTATION);
  baserot.write(angle);// send servo to the middle at 70 degrees
  Serial.println("Robo ARM VR800 Velleman for Makers");
  Serial.println("Ready to Use");

  //Centering servo's keep in loop untill pin is high
  while (digitalRead(A5) == LOW)
  {
    gripper.write(95);
    griprot.write(90);
    shoulder.write(45);
    wrist.write(180);
    baserot.write(90);
  }
}

void loop()
{
  //reads the value of potentiometers (value between 0 and 1023)

  valPot1 = analogRead(pot1);

  valPot1 = map (valPot1, 0, 1023, 0, 180);   //scale it to use it with the servo (value between 0 and 180)
  gripper.write(valPot1);                      //set the servo position according to the scaled value
  delay(100);

  valPot2 = analogRead(pot2);
  valPot2 = map (valPot2, 0, 1023, 0, 180);
  griprot.write(valPot2);
  delay(100);

  valPot3 = analogRead(pot3);
  if (valPot3 > 500 && valPot3 < 524) //check if joystick is centered, if yes then detach servo
  { //to prevent from burning out the servo.
    if (shoulder.attached())          //You can change the values to have a more faster response on the joystick
      shoulder.detach();
  }
  else {
    Serial.println(valPot3);
    valPot3 = map (valPot3, 0, 1023, 0, 80);
    if (!shoulder.attached())
      shoulder.attach(SHOULDER);
    shoulder.write(valPot3);
  }
  delay(100);

  valPot4 = analogRead(pot4);
  if (valPot4 > 500 && valPot4 < 524)//check if joystick is centered, if yes then detach servo
  { //to prevent from burning out the servo.
    if (wrist.attached())          //You can change the values to have a more faster response on the joystick
      wrist.detach();
  }
  else {

    valPot4 = map (valPot4, 0, 1023, 0, 90);
    if (!wrist.attached())
      wrist.attach(WRIST);
    wrist.write(valPot4);
  }
  delay(100);

  while (digitalRead(RIGHT) == LOW) {

    if (angle > 0 && angle <= 180) {
      angle = angle - angleStep;
      if (angle < 0) {
        angle = 0;
      } else {
        baserot.write(angle); // move the servo to desired angle
        Serial.print("Moved to: ");
        Serial.print(angle);   // print the angle
        Serial.println(" degree");
      }
    }

    delay(100); // waits for the servo to get there
  }// while

  while (digitalRead(LEFT) == LOW) {

    if (angle >= 0 && angle <= 180) {
      angle = angle + angleStep;
      if (angle > 180) {
        angle = 180;
      } else {
        baserot.write(angle); // move the servo to desired angle
        Serial.print("Moved to: ");
        Serial.print(angle);   // print the angle
        Serial.println(" degree");
      }
    }

    delay(100); // waits for the servo to get there
  }//
}
