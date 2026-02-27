std_msgs/Header header
uint8   status          # 0=uncalibrated  1=in_progress  2=calibrated
float32 offset_x        # Hard-iron offset X (µT)
float32 offset_y        # Hard-iron offset Y (µT)
float32 offset_z        # Hard-iron offset Z (µT)
float32 field_strength  # Measured field magnitude (µT)
