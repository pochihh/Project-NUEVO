std_msgs/Header header
uint16  button_mask       # Bitmask of all digital inputs (buttons + limit switches)
                          # Use SystemStatus.limit_switch_mask to distinguish them
uint8[3] led_brightness   # Brightness 0–255 for each user LED (0 = off)
uint8[]  neo_pixels_rgb   # NeoPixel colors: [R0, G0, B0, R1, G1, B1, ...]
                          # Length = neoPixelCount × 3 (from SystemStatus.neopixel_count)
