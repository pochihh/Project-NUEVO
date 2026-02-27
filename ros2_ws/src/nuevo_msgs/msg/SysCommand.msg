# Arduino firmware state machine command.
# 1=START (IDLE→RUNNING)  2=STOP (RUNNING→IDLE)  3=RESET (ERROR→IDLE)  4=ESTOP (any→ESTOP)
std_msgs/Header header
uint8 command
