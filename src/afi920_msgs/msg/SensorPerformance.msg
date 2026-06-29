# Sensor Performance Interface (SPI) - Event 0x8004, TCP/UDP 30511
# Maps 1:1 to bts_spi_message_t
std_msgs/Header header

uint8 sensor_id
uint32 message_counter
uint64 timestamp_measurement_ns        # sensor measurement timestamp (nanoseconds, PTP)
uint8 data_qualifier

# Vehicle coordinate system reference
uint8 COORD_REAR_AXLE=0
uint8 COORD_ROAD_LEVEL=1
uint8 vehicle_coordinate_system

# Sensor mounting pose
afi920_msgs/SensorPose sensor_pose

# Field-of-view segments (up to 16)
afi920_msgs/FovSegment[] fov_segments

# Recognisable object types (up to 16)
afi920_msgs/RecognisableObject[] recognisable_objects

# Reference target types (up to 16)
afi920_msgs/ReferenceTarget[] reference_targets
