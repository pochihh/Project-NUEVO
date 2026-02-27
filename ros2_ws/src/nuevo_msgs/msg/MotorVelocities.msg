# Per-motor velocity command. Sent by the kinematics/trajectory node.
# The bridge forwards each motor's command directly to the Arduino — no
# kinematics are performed in the bridge. motors[0] = motor 1, etc.
#
# mode: 0=no change/skip  2=velocity (ticks/sec)  3=pwm (-255..+255)
# Set mode=0 for motors that should not be commanded this cycle.
std_msgs/Header header
int32[4]  velocity_ticks_per_sec
uint8[4]  mode
