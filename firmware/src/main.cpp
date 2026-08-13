// ===========================================================================
// src/main.cpp
//
// Teensy 4.1 firmware for an autonomous wheelchair drive base.
//
// Responsibilities:
//   1. Subscribe to /cmd_vel (geometry_msgs/Twist) from the ROS navigation
//      stack running on the onboard computer.
//   2. Convert (linear.x, angular.z) into per-wheel target surface speeds
//      via differential-drive kinematics.
//   3. Close a velocity loop per wheel using FuzzyPID, driving each BTS7960
//      half-bridge motor driver's PWM/direction pins.
//   4. Read quadrature encoders via hardware interrupts.
//   5. Read an IMU for heading (yaw) and publish encoder ticks + heading to
//      ROS on wheelchair_msgs/EncoderHeading, consumed by wheelchair_base's
//      odom_publisher node to compute /odom and the odom->base_link tf.
//
// Safety notes (read before flashing to a real chair):
//   - A hardware/software watchdog on /cmd_vel timeout is MANDATORY. Loss of
//     comms with the nav computer must stop the drive motors, not coast
//     them indefinitely. See CMD_VEL_TIMEOUT_MS below.
//   - BTS7960 R_EN/L_EN are wired active-high enable pins; they are held
//     LOW (disabled) until a valid heartbeat is received.
// ===========================================================================

#include <Arduino.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <wheelchair_msgs/EncoderHeading.h>   // generated via rosserial make_libraries.py
#include "FuzzyPID.h"

// ---------------------------------------------------------------------------
// Robot-specific kinematic constants -- MEASURE THESE on the physical chair.
// ---------------------------------------------------------------------------
static const float WHEEL_RADIUS_M      = 0.155f;   // drive wheel radius (m)
static const float WHEEL_BASE_M        = 0.560f;   // track width, wheel-center to wheel-center (m)
static const float ENCODER_CPR         = 2048.0f;  // counts per revolution (quadrature, post x4)
static const float MAX_WHEEL_SPEED_MPS = 1.60f;    // conservative indoor max linear wheel speed

// ---------------------------------------------------------------------------
// Pin map -- adjust to match the actual wiring harness.
// BTS7960: each half-bridge module needs RPWM, LPWM, R_EN, L_EN.
// Encoders: both channels on interrupt-capable pins for full quadrature decode.
// ---------------------------------------------------------------------------
// Left motor driver (BTS7960 #1)
static const uint8_t LEFT_RPWM_PIN = 2;
static const uint8_t LEFT_LPWM_PIN = 3;
static const uint8_t LEFT_R_EN_PIN = 4;
static const uint8_t LEFT_L_EN_PIN = 5;

// Right motor driver (BTS7960 #2)
static const uint8_t RIGHT_RPWM_PIN = 6;
static const uint8_t RIGHT_LPWM_PIN = 7;
static const uint8_t RIGHT_R_EN_PIN = 8;
static const uint8_t RIGHT_L_EN_PIN = 9;

// Quadrature encoders (A/B channels), interrupt-capable pins on Teensy 4.1
static const uint8_t LEFT_ENC_A_PIN  = 20;
static const uint8_t LEFT_ENC_B_PIN  = 21;
static const uint8_t RIGHT_ENC_A_PIN = 22;
static const uint8_t RIGHT_ENC_B_PIN = 23;

// ---------------------------------------------------------------------------
// Safety / comms watchdog
// ---------------------------------------------------------------------------
static const uint32_t CMD_VEL_TIMEOUT_MS = 500;  // stop drive if no /cmd_vel in this window
static const uint32_t CONTROL_LOOP_HZ    = 50;
static const uint32_t CONTROL_PERIOD_MS  = 1000 / CONTROL_LOOP_HZ;

// ---------------------------------------------------------------------------
// Shared state, updated from ISRs -- must be volatile.
// ---------------------------------------------------------------------------
volatile long g_leftTicks  = 0;
volatile long g_rightTicks = 0;

// Target linear surface speed per wheel (m/s), set from the latest /cmd_vel.
volatile float g_targetLeftSpeed  = 0.0f;
volatile float g_targetRightSpeed = 0.0f;
volatile uint32_t g_lastCmdVelMillis = 0;

// ---------------------------------------------------------------------------
// FuzzyPID controllers -- one per wheel. Baseline gains below are a
// reasonable starting point for a ~1.5 m/s indoor wheelchair; re-tune on
// the bench with the wheels off the ground before field trials.
// ---------------------------------------------------------------------------
FuzzyPID leftPid(/*kp=*/120.0f, /*ki=*/40.0f, /*kd=*/2.0f,
                  /*outMin=*/-255.0f, /*outMax=*/255.0f,
                  /*errorRange=*/1.5f, /*deltaErrorRange=*/3.0f);
FuzzyPID rightPid(/*kp=*/120.0f, /*ki=*/40.0f, /*kd=*/2.0f,
                   /*outMin=*/-255.0f, /*outMax=*/255.0f,
                   /*errorRange=*/1.5f, /*deltaErrorRange=*/3.0f);

// ---------------------------------------------------------------------------
// ROS node handle, publisher, and subscriber
// ---------------------------------------------------------------------------
ros::NodeHandle nh;

wheelchair_msgs::EncoderHeading encoderMsg;
ros::Publisher encoderPub("encoder_heading", &encoderMsg);

void cmdVelCallback(const geometry_msgs::Twist &msg) {
    // Differential-drive inverse kinematics:
    //   v_left  = v - (w * L / 2)
    //   v_right = v + (w * L / 2)
    float v = msg.linear.x;
    float w = msg.angular.z;

    float leftSpeed  = v - (w * WHEEL_BASE_M * 0.5f);
    float rightSpeed = v + (w * WHEEL_BASE_M * 0.5f);

    g_targetLeftSpeed  = constrain(leftSpeed, -MAX_WHEEL_SPEED_MPS, MAX_WHEEL_SPEED_MPS);
    g_targetRightSpeed = constrain(rightSpeed, -MAX_WHEEL_SPEED_MPS, MAX_WHEEL_SPEED_MPS);
    g_lastCmdVelMillis = millis();
}

ros::Subscriber<geometry_msgs::Twist> cmdVelSub("cmd_vel", &cmdVelCallback);

// ---------------------------------------------------------------------------
// Encoder ISRs -- simple quadrature decode using the B channel level at the
// moment of an A-channel edge to determine direction. For higher resolution,
// attach interrupts to both channels; here we keep it lean for readability.
// ---------------------------------------------------------------------------
void leftEncoderIsr() {
    bool bLevel = digitalReadFast(LEFT_ENC_B_PIN);
    if (bLevel) g_leftTicks++; else g_leftTicks--;
}

void rightEncoderIsr() {
    bool bLevel = digitalReadFast(RIGHT_ENC_B_PIN);
    if (bLevel) g_rightTicks--; else g_rightTicks++;  // right side mounted mirrored
}

// ---------------------------------------------------------------------------
// IMU heading stub. Replace with a real driver call (e.g. BNO055 Euler yaw,
// or a Madgwick/Mahony filter fused from an MPU-9250). Kept as a stub here
// so this file compiles standalone without committing to one IMU part.
// ---------------------------------------------------------------------------
float readHeadingRadians() {
    // TODO: replace with actual IMU driver read, e.g.:
    //   imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    //   return euler.x() * DEG_TO_RAD;
    return 0.0f;
}

// ---------------------------------------------------------------------------
// Low-level motor output. BTS7960 half-bridge: drive RPWM for forward
// effort, LPWM for reverse effort, with the unused direction's PWM held at
// zero (per BTS7960 datasheet, do not drive both PWM pins simultaneously).
// ---------------------------------------------------------------------------
void writeMotor(uint8_t rpwmPin, uint8_t lpwmPin, float effort) {
    effort = constrain(effort, -255.0f, 255.0f);
    if (effort >= 0.0f) {
        analogWrite(rpwmPin, static_cast<int>(effort));
        analogWrite(lpwmPin, 0);
    } else {
        analogWrite(rpwmPin, 0);
        analogWrite(lpwmPin, static_cast<int>(-effort));
    }
}

void setMotorsEnabled(bool enabled) {
    digitalWrite(LEFT_R_EN_PIN, enabled ? HIGH : LOW);
    digitalWrite(LEFT_L_EN_PIN, enabled ? HIGH : LOW);
    digitalWrite(RIGHT_R_EN_PIN, enabled ? HIGH : LOW);
    digitalWrite(RIGHT_L_EN_PIN, enabled ? HIGH : LOW);
}

void stopMotorsImmediate() {
    writeMotor(LEFT_RPWM_PIN, LEFT_LPWM_PIN, 0.0f);
    writeMotor(RIGHT_RPWM_PIN, RIGHT_LPWM_PIN, 0.0f);
    setMotorsEnabled(false);
    leftPid.reset();
    rightPid.reset();
}

// ---------------------------------------------------------------------------
// Convert an encoder tick delta over dt into a measured wheel surface speed.
// ---------------------------------------------------------------------------
float ticksToSpeedMps(long tickDelta, float dtSeconds) {
    float revolutions = static_cast<float>(tickDelta) / ENCODER_CPR;
    float distanceM = revolutions * (2.0f * PI * WHEEL_RADIUS_M);
    return (dtSeconds > 0.0f) ? (distanceM / dtSeconds) : 0.0f;
}

void setup() {
    pinMode(LEFT_RPWM_PIN, OUTPUT);
    pinMode(LEFT_LPWM_PIN, OUTPUT);
    pinMode(LEFT_R_EN_PIN, OUTPUT);
    pinMode(LEFT_L_EN_PIN, OUTPUT);

    pinMode(RIGHT_RPWM_PIN, OUTPUT);
    pinMode(RIGHT_LPWM_PIN, OUTPUT);
    pinMode(RIGHT_R_EN_PIN, OUTPUT);
    pinMode(RIGHT_L_EN_PIN, OUTPUT);

    pinMode(LEFT_ENC_A_PIN, INPUT_PULLUP);
    pinMode(LEFT_ENC_B_PIN, INPUT_PULLUP);
    pinMode(RIGHT_ENC_A_PIN, INPUT_PULLUP);
    pinMode(RIGHT_ENC_B_PIN, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A_PIN), leftEncoderIsr, RISING);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A_PIN), rightEncoderIsr, RISING);

    // Drives disabled until we have a live /cmd_vel heartbeat.
    setMotorsEnabled(false);

    nh.initNode();
    nh.subscribe(cmdVelSub);
    nh.advertise(encoderPub);
}

void loop() {
    static uint32_t lastLoopMillis = 0;
    static long lastLeftTicks = 0;
    static long lastRightTicks = 0;

    uint32_t nowMillis = millis();
    if (nowMillis - lastLoopMillis < CONTROL_PERIOD_MS) {
        nh.spinOnce();
        return;
    }
    float dt = (nowMillis - lastLoopMillis) / 1000.0f;
    lastLoopMillis = nowMillis;

    // ---- Safety watchdog: disable drive if /cmd_vel has gone stale ----
    bool cmdVelFresh = (nowMillis - g_lastCmdVelMillis) < CMD_VEL_TIMEOUT_MS;
    if (!cmdVelFresh) {
        stopMotorsImmediate();
    } else {
        setMotorsEnabled(true);
    }

    // ---- Snapshot tick counts atomically ----
    noInterrupts();
    long leftTicksNow = g_leftTicks;
    long rightTicksNow = g_rightTicks;
    interrupts();

    long leftDelta  = leftTicksNow - lastLeftTicks;
    long rightDelta = rightTicksNow - lastRightTicks;
    lastLeftTicks  = leftTicksNow;
    lastRightTicks = rightTicksNow;

    float measuredLeftSpeed  = ticksToSpeedMps(leftDelta, dt);
    float measuredRightSpeed = ticksToSpeedMps(rightDelta, dt);

    // ---- Closed-loop velocity control (skipped while e-stopped) ----
    if (cmdVelFresh) {
        float leftEffort  = leftPid.compute(g_targetLeftSpeed, measuredLeftSpeed);
        float rightEffort = rightPid.compute(g_targetRightSpeed, measuredRightSpeed);
        writeMotor(LEFT_RPWM_PIN, LEFT_LPWM_PIN, leftEffort);
        writeMotor(RIGHT_RPWM_PIN, RIGHT_LPWM_PIN, rightEffort);
    }

    // ---- Publish raw encoder ticks + heading for wheelchair_base's
    //      odom_publisher to convert into /odom + tf ----
    encoderMsg.left_ticks  = leftTicksNow;
    encoderMsg.right_ticks = rightTicksNow;
    encoderMsg.heading     = readHeadingRadians();
    encoderMsg.stamp       = nh.now();
    encoderPub.publish(&encoderMsg);

    nh.spinOnce();
}
