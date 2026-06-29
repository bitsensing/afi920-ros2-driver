# Sensor mounting pose relative to vehicle coordinate system
# Current SPI wire pose is 32 bytes. Error fields not present on the wire
# are retained for ROS API compatibility and default to 0.

# Origin position (meters)
float32 origin_x
float32 origin_y
float32 origin_z
float32 origin_error_x
float32 origin_error_y
float32 origin_error_z

# Orientation (radians)
float32 yaw
float32 pitch
float32 roll
float32 yaw_error
float32 pitch_error
float32 roll_error
