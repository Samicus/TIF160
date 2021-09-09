#include <Servo.h>

Servo servothumb;
Servo servoindex;
Servo servomajeure;
Servo servoringfinger;
Servo servopinky;
Servo servowrist;
//Servo servowrist2;

void setup() { 
  pinMode(13, OUTPUT);
  servothumb.attach(2);
  servoindex.attach(3);
  servomajeure.attach(4);
  servoringfinger.attach(5);
  servopinky.attach(6);
  servowrist.attach(7);
//  servowrist2.attach(8);
} 

void loop() {

  fingersToZero(); // moves all finger servos to 0 (fully extended) and wrist to 90 (middle)
  delay(5000);//*/
/*  fingersToMax(); // moves all finger servos to max (fully flexed) and wrist to 90 (middle)
  delay(5000);//*/
/*  allToMiddle(); // moves all servos to the middle of the range
  delay(5000);//*/

  // simple test program that moves the hand in all positions
/*  digitalWrite(13, HIGH); // the LED is on while hand is in rest position
  allToRest(); // moves all finger servos to 0 (fully extended) and wrist to 90 (middle)
  delay(2000);
  digitalWrite(13, LOW); // LED is off while hand is in other positions
  allToZero(); // moves all servos to zero, including wrist
  delay(2000);
  allToMax(); // moves all servos to the maximum range, i.e. clench the fist and rotate wrist
  delay(2000);
  allToMiddle(); // moves all servos to the middle of the range
  delay(2000);//*/

}

void fingersToZero() {
  servothumb.write(0);
  delay(5);
  servoindex.write(0);
  delay(5);
  servomajeure.write(0);
  delay(5);
  servoringfinger.write(0);
  delay(5);
  servopinky.write(0);
  delay(5);
  servowrist.write(90);
/*  delay(5);
  servowrist2.write(90);//*/
}

void fingersToMax() {
  servothumb.write(180);
  delay(5);
  servoindex.write(180);
  delay(5);
  servomajeure.write(180);
  delay(5);
  servoringfinger.write(140);
  delay(5);
  servopinky.write(160);
  delay(5);
  servowrist.write(90);
/*  delay(5);
  servowrist2.write(90);//*/
}

void allToMiddle() {
  servothumb.write(90);
  delay(5);
  servoindex.write(90);
  delay(5);
  servomajeure.write(90);
  delay(5);
  servoringfinger.write(90);
  delay(5);
  servopinky.write(80);
  delay(5);
  servowrist.write(90);
/*  delay(5);
  servowrist2.write(90);//*/
}

void allToMax() {
  servothumb.write(180);
  delay(5);
  servoindex.write(180);
  delay(5);
  servomajeure.write(180);
  delay(5);
  servoringfinger.write(140);
  delay(5);
  servopinky.write(160);
  delay(5);
  servowrist.write(180);
/*  delay(5);
  servowrist2.write(180);//*/
}

void allToRest() {
  servothumb.write(0);
  delay(5);
  servoindex.write(0);
  delay(5);
  servomajeure.write(0);
  delay(5);
  servoringfinger.write(0);
  delay(5);
  servopinky.write(0);
  delay(5);
  servowrist.write(90);
/*  delay(5);
  servowrist2.write(90);//*/
}

void allToZero() {
  servothumb.write(0);
  delay(5);
  servoindex.write(0);
  delay(5);
  servomajeure.write(0);
  delay(5);
  servoringfinger.write(0);
  delay(5);
  servopinky.write(0);
  delay(5);
  servowrist.write(0);
/*  delay(5);
  servowrist2.write(0);//*/
}

void rotateWrist() {
  servowrist.write(180);
  delay(3000);
  servowrist.write(90);
  delay(3000);
  servowrist.write(0);
  delay(3000);
  servowrist.write(90);
}

