# Sensor Performance Interface (SPI) - Event 0x8004, TCP/UDP 30511
# Carries the bts_spi_message_t content (interface version and cycle time are not copied)
std_msgs/Header header

uint8 sensor_id
uint32 message_counter
uint64 timestamp_measurement_ns        # sensor measurement timestamp (nanoseconds, PTP)
uint8 data_qualifier                   # 0=normal, 6=invalid (resend of the previous message)

# Vehicle coordinate system reference
uint8 COORD_REAR_AXLE=0                # not sent by the sensor
uint8 COORD_ROAD_LEVEL=1               # always sent
uint8 vehicle_coordinate_system

# Sensor mounting pose
afi920_msgs/SensorPose sensor_pose

# Field-of-view segments (up to 16; the sensor sends 2 in DUAL(0)/UDR(6) mode,
# [0] narrow LR and [1] wide SR, 1 in other modes, 0 before the configuration is applied)
afi920_msgs/FovSegment[] fov_segments

# Recognisable object types (up to 16; the sensor sends 0 entries)
afi920_msgs/RecognisableObject[] recognisable_objects

# Reference target types (up to 16; the sensor sends 0 entries)
afi920_msgs/ReferenceTarget[] reference_targets
