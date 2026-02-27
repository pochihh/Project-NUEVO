# Status for one stepper motor. stepper_number is 1-based (1–4).
uint8  stepper_number
uint8  enabled         # 0=disabled (coil released)  1=enabled (coil held)
uint8  motion_state    # 0=idle  1=accel  2=cruise  3=decel  4=homing  5=fault
uint8  limit_hit       # bit0=min limit  bit1=max limit
int32  commanded_count # Open-loop commanded step count (not a measured position)
int32  target_count    # Target step count
uint32 current_speed   # Current speed (steps/sec)
uint32 max_speed       # Configured max speed (steps/sec)
uint32 acceleration    # Configured acceleration (steps/sec²)
