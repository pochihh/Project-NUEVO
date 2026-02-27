# Status for one DC motor. motor_number is 1-based (1–4).
uint8  motor_number
uint8  mode          # 0=disabled  1=position  2=velocity  3=pwm
uint8  fault_flags   # bit0=overcurrent  bit1=stall
int32  position      # Encoder ticks (measured)
int32  velocity      # Ticks/sec (measured)
int32  target_pos    # Commanded position (position mode)
int32  target_vel    # Commanded velocity (velocity mode)
int16  pwm_output    # Current PWM output (-255 to +255)
int16  current_ma    # Motor current (mA); -1 if not measured
float32 pos_kp
float32 pos_ki
float32 pos_kd
float32 vel_kp
float32 vel_ki
float32 vel_kd
