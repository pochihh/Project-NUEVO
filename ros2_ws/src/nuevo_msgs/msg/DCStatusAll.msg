std_msgs/Header header
uint32 frame_index                     # Monotonic counter — detect dropped packets
nuevo_msgs/DCMotorStatus[4] motors     # motors[0] = motor 1, ..., motors[3] = motor 4
