# Sensor mounting pose relative to vehicle coordinate system
# Current SPI wire pose is 32 bytes. Error fields not present on the wire
# are retained for ROS API compatibility and default to 0.

# Origin position (meters)
float32 origin_x
float32 origin_y
float32 origin_z
float32 origin_error_x                 # legacy, always 0
float32 origin_error_y                 # legacy, always 0
float32 origin_error_z                 # legacy, always 0

# Orientation (radians)
# yaw/pitch/roll are the configured mounting values (no auto-alignment correction).
# yaw_error/pitch_error carry the auto-alignment misalignment estimate,
# limited to +-3 deg (+-0.0524 rad); they are not a pose uncertainty.
float32 yaw
float32 pitch
float32 roll
float32 yaw_error
float32 pitch_error
float32 roll_error                     # legacy, always 0
