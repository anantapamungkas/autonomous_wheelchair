#!/usr/bin/env python
import rospy
import tf
import math
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, Vector3
from nav_msgs.msg import Odometry

class BaseController:
    def __init__(self):
        rospy.init_node('base_controller')
        
        # Physical parameters of the wheelchair
        self.wheelbase = 0.60  # Distance between wheels in meters
        self.wheel_radius = 0.15  # Wheel radius in meters
        
        # Internal state for odometry calculation
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = rospy.Time.now()
        
        # ROS Publishers and Subscribers
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.cmd_sub = rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)
        
    def cmd_vel_callback(self, msg):
        # 1. Extract requested linear (v) and angular (omega) velocities
        v = msg.linear.x
        omega = msg.angular.z
        
        # 2. Convert to left and right wheel speeds (m/s)
        # Formula: v_wheel = v +/- (omega * wheelbase / 2)
        v_right = v + (omega * self.wheelbase / 2.0)
        v_left = v - (omega * self.wheelbase / 2.0)
        
        # 3. Send these speeds to the motor hardware
        self.send_motor_commands(v_left, v_right)
        
    def send_motor_commands(self, v_left, v_right):
        # TODO: Implement the hardware interface here.
        # Convert m/s into the specific format your motor drivers expect 
        # (e.g., PWM values, RPM, or raw byte commands via a serial port).
        pass

    def read_encoders(self):
        # TODO: Implement reading from your hardware encoders here.
        # Calculate the *actual* linear and angular velocity based on wheel ticks.
        # For this template, we return 0.0 to prevent errors if run as-is.
        actual_v = 0.0
        actual_omega = 0.0
        return actual_v, actual_omega
        
    def publish_odometry(self, v, omega, dt):
        # Calculate changes in position (Euler integration)
        delta_x = (v * math.cos(self.th)) * dt
        delta_y = (v * math.sin(self.th)) * dt
        delta_th = omega * dt
        
        self.x += delta_x
        self.y += delta_y
        self.th += delta_th
        
        # Convert yaw angle to a quaternion for ROS messages
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)
        current_time = rospy.Time.now()
        
        # Publish the transform over tf (odom -> base_link)
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0.),
            odom_quat,
            current_time,
            "base_link",
            "odom"
        )
        
        # Construct and publish the Odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))
        
        # Add velocity information to the odometry message
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega
        
        self.odom_pub.publish(odom)

    def loop(self):
        rate = rospy.Rate(50)  # Standard 50Hz control loop
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            dt = (current_time - self.last_time).to_sec()
            
            # Request actual movement data from hardware
            actual_v, actual_omega = self.read_encoders()
            
            # Publish the calculated position and transform
            self.publish_odometry(actual_v, actual_omega, dt)
            
            self.last_time = current_time
            rate.sleep()

if __name__ == '__main__':
    try:
        controller = BaseController()
        controller.loop()
    except rospy.ROSInterruptException:
        pass