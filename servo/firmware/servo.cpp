#include <Servo.h>
#include <ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Bool.h>

#include <Arduino.h>

ros::NodeHandle hubert;

//Servos
Servo shoulder;
Servo elbow;
Servo headRotation;
Servo headTilt;
Servo body;

//Init position of all servos
int pos_init[] = {50, 50, 90, 90, 90};

int servo_pins[] = {9, 10, 11, 12, 13};



//ROS-setup
void servo_neck_rot(const std_msgs::UInt16& cmd_msg) {

	headRotation.write(cmd_msg.data);

	//if (sizeof(cmd_msg.data) >= 2) {
	//	headRotation.write(cmd_msg.data[0]);
	//	headTilt.write(cmd_msg.data[1]);
	//}
}

void servo_neck_tilt(const std_msgs::UInt16& cmd_msg) {

	headTilt.write(cmd_msg.data);

	//if (sizeof(cmd_msg.data) >= 2) {
	//	headRotation.write(cmd_msg.data[0]);
	//	headTilt.write(cmd_msg.data[1]);
	//}
}

void servo_arm_shoulder(const std_msgs::UInt16& cmd_msg) {
//Take in array with all angles for the arm
	shoulder.write(cmd_msg.data);
	//if (sizeof(cmd_msg.data) >= 2) {
	//	shoulder.write(cmd_msg.data[0]);
	//	elbow.write(cmd_msg.data[1]);
	//}
}

void servo_arm_elbow(const std_msgs::UInt16& cmd_msg) {
	shoulder.write(cmd_msg.data);
}
void servo_body_ex(const std_msgs::UInt16& cmd_msg) {
//Take in value for body position
	body.write(cmd_msg.data);
}

void fire_gun_ex(const std_msgs::Bool& cmd_msg) {
//Check if gun should be fired
	if (cmd_msg.data) {
		digitalWrite(8,HIGH);
	} else {
		digitalWrite(8,LOW);
	}
}

ros::Subscriber<std_msgs::UInt16> sub_rot("servo_neck_rot", servo_neck_rot);
ros::Subscriber<std_msgs::UInt16> sub_tilt("servo_neck_tilt", servo_neck_tilt);

ros::Subscriber<std_msgs::UInt16> sub_shoulder("servo_shoulder", servo_arm_shoulder);
ros::Subscriber<std_msgs::UInt16> sub_elbow("servo_elbow", servo_arm_elbow);

ros::Subscriber<std_msgs::UInt16> sub_body("servo_body", servo_body_ex);
ros::Subscriber<std_msgs::Bool> fire_gun("fire_gun", fire_gun_ex);

void setup() {
	hubert.initNode();
	hubert.subscribe(sub_rot);
	hubert.subscribe(sub_tilt);

	hubert.subscribe(sub_shoulder);
	hubert.subscribe(sub_elbow);	
	
	hubert.subscribe(sub_body);
	hubert.subscribe(fire_gun);	
	
	//Attach fire pin
	pinMode(8, OUTPUT);

	//Attach each joint servo
	//and write each init position
	shoulder.attach(servo_pins[0]);
	shoulder.write(pos_init[0]);

	elbow.attach(servo_pins[1]);
	elbow.write(pos_init[1]);

	headRotation.attach(servo_pins[2]);
	headRotation.write(pos_init[2]);

	headTilt.attach(servo_pins[3]);
	headTilt.write(pos_init[3]);

	body.attach(servo_pins[4]);
	body.write(pos_init[4]);
	
	digitalWrite(8, LOW);

	delay(250);
}

void loop() {
	hubert.spinOnce();
	delay(1);
}
