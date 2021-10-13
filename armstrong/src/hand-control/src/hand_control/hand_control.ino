#include <Servo.h> 
#include <ros.h>
#include <std_msgs/Float64MultiArray.h>

ros::NodeHandle  nh;

Servo THUMB, INDEX, MIDDLE, RING, PINKY, WRIST, ELBOW;

void servo_cb( const std_msgs::Float64MultiArray& cmd_msg){
  THUMB.write(  180 - cmd_msg.data[0]); //set servo angle, should be from 0-180
  delay(5);
  INDEX.write(  180 - cmd_msg.data[1]); //set servo angle, should be from 0-180
  delay(5);
  MIDDLE.write( 180 - cmd_msg.data[2]); //set servo angle, should be from 0-180
  delay(5);
  RING.write(   180 - cmd_msg.data[3]); //set servo angle, should be from 0-180
  delay(5);
  PINKY.write(  180 - cmd_msg.data[4]); //set servo angle, should be from 0-180
  delay(5);
  WRIST.write(  180 - cmd_msg.data[5]); //set servo angle, should be from 0-180
  delay(5);
  ELBOW.write(cmd_msg.data[6]); //set servo angle, should be from 0-180
}


ros::Subscriber<std_msgs::Float64MultiArray> sub("hand_tracking", servo_cb);

void setup(){
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
  
  /*
   * PIN 2: THUMB
   * PIN 3: INDEX
   * PIN 4: MIDDLE
   * PIN 5: RING
   * PIN 6: PINKY
   * PIN 7: WRIST
   * PIN 8: ELBOW
   */
  
  THUMB.attach(2);
  INDEX.attach(3);
  MIDDLE.attach(4);
  RING.attach(5);
  PINKY.attach(6);
  WRIST.attach(7);
  ELBOW.attach(8);
  
}

void loop(){
  nh.spinOnce();
  delay(1);
}
