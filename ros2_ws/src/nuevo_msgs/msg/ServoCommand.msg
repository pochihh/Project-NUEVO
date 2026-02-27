# Servo channel command. channel is 1-based (1–16); 255 = all channels.
# If enable=false: disables the channel (pulse cut, servo goes limp).
# If enable=true and pulse_us > 0: enables and sets pulse width.
# If enable=true and pulse_us = 0: enables without changing pulse (use current position).
std_msgs/Header header
uint8  channel     # 1-based (1–16), or 255 for all channels
bool   enable
uint16 pulse_us    # Target pulse width (µs), typically 500–2500; 0 = no change
