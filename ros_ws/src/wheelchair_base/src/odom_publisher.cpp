// ===========================================================================
// odom_publisher.cpp
//
// Subscribes to raw drivetrain telemetry (wheelchair_msgs/EncoderHeading)
// published by the Teensy firmware, integrates differential-drive
// kinematics to produce wheel odometry, and:
//   1. Publishes nav_msgs/Odometry on /odom
//   2. Broadcasts the odom -> base_link tf transform
//
// Kinematic model: standard differential-drive odometry using wheel
// encoder deltas. IMU heading (from the firmware) is fused in as the
// authoritative yaw source (encoders alone drift on wheel-slip-prone
// surfaces), while translational displacement still comes from the
// average of the two wheel encoders.
// ===========================================================================

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <wheelchair_msgs/EncoderHeading.h>

class OdomPublisher {
public:
    OdomPublisher(ros::NodeHandle &nh, ros::NodeHandle &pnh) {
        // ---- Kinematic parameters (override via rosparam / launch args) ----
        pnh.param("wheel_radius_m", wheelRadiusM_, 0.155);
        pnh.param("encoder_cpr",    encoderCpr_,   2048.0);
        pnh.param("odom_frame_id",  odomFrameId_,  std::string("odom"));
        pnh.param("base_frame_id",  baseFrameId_,  std::string("base_link"));
        pnh.param("publish_tf",     publishTf_,    true);

        odomPub_ = nh.advertise<nav_msgs::Odometry>("odom", 50);
        encoderSub_ = nh.subscribe("encoder_heading", 10,
                                     &OdomPublisher::encoderCallback, this);

        x_ = 0.0;
        y_ = 0.0;
        theta_ = 0.0;
        haveLastReading_ = false;
    }

private:
    // ---- ROS interfaces ----
    ros::Publisher odomPub_;
    ros::Subscriber encoderSub_;
    tf2_ros::TransformBroadcaster tfBroadcaster_;

    // ---- Parameters ----
    double wheelRadiusM_;
    double encoderCpr_;
    std::string odomFrameId_;
    std::string baseFrameId_;
    bool publishTf_;

    // ---- Integrated pose state ----
    double x_, y_, theta_;

    // ---- Previous reading, for delta computation ----
    bool haveLastReading_;
    int32_t lastLeftTicks_ = 0;
    int32_t lastRightTicks_ = 0;
    ros::Time lastStamp_;

    void encoderCallback(const wheelchair_msgs::EncoderHeading &msg) {
        if (!haveLastReading_) {
            lastLeftTicks_  = msg.left_ticks;
            lastRightTicks_ = msg.right_ticks;
            lastStamp_      = msg.stamp;
            theta_          = msg.heading;  // seed yaw from IMU on first message
            haveLastReading_ = true;
            return;
        }

        double dt = (msg.stamp - lastStamp_).toSec();
        if (dt <= 0.0) {
            // Guard against out-of-order or duplicate timestamps from the
            // microcontroller clock; skip this integration step.
            lastLeftTicks_  = msg.left_ticks;
            lastRightTicks_ = msg.right_ticks;
            lastStamp_      = msg.stamp;
            return;
        }

        // ---- Per-wheel distance traveled since the last message ----
        double leftDelta  = static_cast<double>(msg.left_ticks - lastLeftTicks_);
        double rightDelta = static_cast<double>(msg.right_ticks - lastRightTicks_);

        double metersPerTick = (2.0 * M_PI * wheelRadiusM_) / encoderCpr_;
        double leftDistanceM  = leftDelta * metersPerTick;
        double rightDistanceM = rightDelta * metersPerTick;

        // ---- Differential-drive forward kinematics ----
        double linearDistanceM = (leftDistanceM + rightDistanceM) * 0.5;

        // Use the IMU-reported heading directly as the authoritative yaw
        // (more robust to wheel slip than encoder-only heading estimation).
        double newTheta = msg.heading;
        double deltaTheta = angleDiff(newTheta, theta_);

        // Integrate position using the midpoint heading between the two
        // samples for better accuracy on curved trajectories.
        double midTheta = theta_ + deltaTheta * 0.5;
        x_ += linearDistanceM * cos(midTheta);
        y_ += linearDistanceM * sin(midTheta);
        theta_ = newTheta;

        double vx = linearDistanceM / dt;
        double vth = deltaTheta / dt;

        publishOdometry(msg.stamp, vx, vth);

        lastLeftTicks_  = msg.left_ticks;
        lastRightTicks_ = msg.right_ticks;
        lastStamp_      = msg.stamp;
    }

    // Wrap an angle difference into (-pi, pi] to avoid discontinuity jumps
    // when the heading crosses the +/-pi boundary.
    static double angleDiff(double newAngle, double oldAngle) {
        double diff = newAngle - oldAngle;
        while (diff > M_PI) diff -= 2.0 * M_PI;
        while (diff < -M_PI) diff += 2.0 * M_PI;
        return diff;
    }

    void publishOdometry(const ros::Time &stamp, double vx, double vth) {
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        geometry_msgs::Quaternion odomQuat = tf2::toMsg(q);

        // ---- Broadcast odom -> base_link tf ----
        if (publishTf_) {
            geometry_msgs::TransformStamped tfMsg;
            tfMsg.header.stamp = stamp;
            tfMsg.header.frame_id = odomFrameId_;
            tfMsg.child_frame_id = baseFrameId_;
            tfMsg.transform.translation.x = x_;
            tfMsg.transform.translation.y = y_;
            tfMsg.transform.translation.z = 0.0;
            tfMsg.transform.rotation = odomQuat;
            tfBroadcaster_.sendTransform(tfMsg);
        }

        // ---- Publish nav_msgs/Odometry ----
        nav_msgs::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = odomFrameId_;
        odom.child_frame_id = baseFrameId_;

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odomQuat;

        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = vth;

        // Conservative diagonal covariance; tune against ground-truth once
        // the platform is characterized (e.g. via a motion-capture bench).
        for (int i = 0; i < 36; i++) {
            odom.pose.covariance[i] = 0.0;
            odom.twist.covariance[i] = 0.0;
        }
        odom.pose.covariance[0]  = 0.01;   // x
        odom.pose.covariance[7]  = 0.01;   // y
        odom.pose.covariance[35] = 0.02;   // yaw
        odom.twist.covariance[0]  = 0.02;  // vx
        odom.twist.covariance[35] = 0.04;  // vth

        odomPub_.publish(odom);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "odom_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    OdomPublisher odomPublisher(nh, pnh);

    ros::spin();
    return 0;
}
