# EncoderHeading.msg
# Raw drivetrain telemetry published by the Teensy firmware over rosserial.
# Consumed by wheelchair_base/odom_publisher to compute wheel odometry.

Header header          # stamp + frame_id (frame_id typically left empty/unused here)
int32 left_ticks       # cumulative left encoder ticks since boot (signed, wraps at long overflow)
int32 right_ticks      # cumulative right encoder ticks since boot
float32 heading        # IMU-derived yaw, radians, ENU convention (0 = +X / forward)
time stamp              # firmware-side timestamp (nh.now()) for latency-aware integration
