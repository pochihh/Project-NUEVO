# Magnetometer calibration command.
# command: 1=START  2=STOP  3=SAVE  4=APPLY  5=CLEAR
# offset_* only used with command=4 (APPLY) to inject known offsets.
std_msgs/Header header
uint8   command
float32 offset_x   # Hard-iron offset X (µT)
float32 offset_y   # Hard-iron offset Y (µT)
float32 offset_z   # Hard-iron offset Z (µT)
