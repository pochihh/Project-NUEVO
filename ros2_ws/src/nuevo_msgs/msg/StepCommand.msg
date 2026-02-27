# Stepper motor command. stepper_number is 1-based (1–4).
# command_type selects which TLV is generated:
#   0 = STEP_MOVE       (use move_type, target)
#   1 = STEP_HOME       (use direction, home_velocity, backoff_steps)
#   2 = STEP_ENABLE     (enable driver)
#   3 = STEP_DISABLE    (release coil)
#   4 = STEP_SET_PARAMS (use max_velocity, acceleration)
std_msgs/Header header
uint8  stepper_number
uint8  command_type

# STEP_MOVE fields
uint8  move_type        # 0=absolute  1=relative
int32  target           # Target step count

# STEP_HOME fields
int8   direction        # -1=reverse  +1=forward
uint32 home_velocity    # Homing speed (steps/sec)
int32  backoff_steps    # Steps to retreat after limit triggers

# STEP_SET_PARAMS fields
uint32 max_velocity     # steps/sec
uint32 acceleration     # steps/sec²
