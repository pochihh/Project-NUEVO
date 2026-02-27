# Status for one PCA9685 servo channel. channel_number is 1-based (1–16).
uint8  channel_number
bool   enabled
uint16 pulse_us   # Commanded pulse width (µs); 0 = disabled/not set
