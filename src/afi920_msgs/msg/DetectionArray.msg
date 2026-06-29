# Array of radar detections with header metadata (ISO 23150 RDI)
# Event ID: 0x8002 | TCP/UDP
std_msgs/Header header

# Sensor metadata
uint8 sensor_id
uint32 message_counter
uint64 timestamp_measurement_ns        # sensor measurement timestamp (nanoseconds, PTP)
uint8 data_qualifier                   # 0=normal, 1=not_available, 2=reduced_coverage, 3=reduced_perf

# Ambiguity domains. Current RDI wire payload carries velocity ambiguity only;
# the remaining fields are retained for ROS API compatibility and default to 0.
float32 velocity_ambiguity_begin       # m/s (default: -88.89)
float32 velocity_ambiguity_end         # m/s (default: 55.56)
float32 range_ambiguity_begin          # meters (default: 0)
float32 range_ambiguity_end            # meters (default: 300)
float32 azimuth_ambiguity_begin        # radians (default: -1.0472)
float32 azimuth_ambiguity_end          # radians (default: 1.0472)
float32 elevation_ambiguity_begin      # radians (default: -0.2618)
float32 elevation_ambiguity_end        # radians (default: 0.2618)

# Detections info
uint16 max_detections                  # sensor capability (typically 2048)
uint8 recognised_detections_status     # 0=ok, 1=performance limit reached
afi920_msgs/Detection[] detections
