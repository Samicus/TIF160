#include <Servo.h> 
#include <ros.h>
#include <std_msgs/Float64MultiArray.h>

ros::NodeHandle  nh;

Servo servo;

void servo_cb( const std_msgs::Float64MultiArray& cmd_msg){
  servo.write(180 - cmd_msg.data[2]); //set servo angle, should be from 0-180   
}


ros::Subscriber<std_msgs::Float64MultiArray> sub("hand_tracking", servo_cb);

void setup(){
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
  
  servo.attach(9); //attach it to pin 9
}

void loop(){
  nh.spinOnce();
  delay(1);
}
