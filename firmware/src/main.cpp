// ===========================================================================
// src/main.cpp
//
// Minimal ROS Serial Template for Teensy 4.1
// ===========================================================================

#include <Arduino.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <wheelchair_msgs/EncoderHeading.h>

// ---------------------------------------------------------------------------
// ROS Node Handle, Messages, Publishers, and Subscribers
// ---------------------------------------------------------------------------
ros::NodeHandle nh;

// Message instance to publish
wheelchair_msgs::EncoderHeading encoderMsg;
ros::Publisher encoderPub("encoder_heading", &encoderMsg);

// Callback function for /cmd_vel subscriber
void cmdVelCallback(const geometry_msgs::Twist &msg) {
    // TODO: Process incoming Twist commands (msg.linear.x, msg.angular.z)
}

ros::Subscriber<geometry_msgs::Twist> cmdVelSub("cmd_vel", &cmdVelCallback);

void setup() {
    // Initialize ROS node handle and register pub/sub
    nh.initNode();
    nh.subscribe(cmdVelSub);
    nh.advertise(encoderPub);
}

void loop() {
    // TODO: Populate encoderMsg fields before publishing
    // encoderMsg.left_ticks = ...;
    // encoderMsg.right_ticks = ...;
    // encoderMsg.heading = ...;
    // encoderMsg.stamp = nh.now();

    encoderPub.publish(&encoderMsg);

    // Handle incoming communication and maintenance
    nh.spinOnce();
    delay(10); // Adjust loop frequency/rate as needed
}