#include <Servo.h> 
#include <ros.h>
#include <std_msgs/Float64MultiArray.h>

ros::NodeHandle  nh;

Servo finger1, finger2, finger3, finger4, finger5, thumb;

void servo_cb( const std_msgs::Float64MultiArray& cmd_msg){
  finger1.write(180 - cmd_msg.data[0]); //set servo angle, should be from 0-180
  delay(5);
  finger2.write(180 - cmd_msg.data[1]); //set servo angle, should be from 0-180
  delay(5);
  finger3.write(180 - cmd_msg.data[2]); //set servo angle, should be from 0-180
  delay(5);
  finger4.write(180 - cmd_msg.data[3]); //set servo angle, should be from 0-180
  delay(5);
  finger5.write(180 - cmd_msg.data[4]); //set servo angle, should be from 0-180
  delay(5);
  thumb.write(  180 - cmd_msg.data[5]); //set servo angle, should be from 0-180
}


ros::Subscriber<std_msgs::Float64MultiArray> sub("hand_tracking", servo_cb);

void setup(){
  //pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
  
  finger1.attach(2);
  finger2.attach(3);
  finger3.attach(4);
  finger4.attach(5);
  finger5.attach(6);
  thumb.attach(7);
  
}

void loop(){
  nh.spinOnce();
  delay(1);
}
