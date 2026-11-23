# Array of radar detections with header metadata (ISO 23150 RDI)
# Event ID: 0x8002 (interface v1.2.0: headers 0x8002, chunks 0x8100-0x814B) | TCP/UDP
# detections[] holds the detections that passed the driver filters
# (min_range, max_range, min_snr, min_existence_probability), so detection_id
# values can have gaps. A partial v1.2.0 scan holds only the detections that arrived.
std_msgs/Header header

# Sensor metadata
uint8 sensor_id
uint32 message_counter
uint64 timestamp_measurement_ns        # sensor measurement timestamp (nanoseconds, PTP)
uint8 data_qualifier                   # 0=normal, 1=not_available (record not complete: header messages only, no
                                       # detections, header fields repeat the previous header), 6=invalid (resend)
                                       # 2-5 are not sent; on interface v1.0.x the sensor can send 5=test_mode

# Ambiguity domains. Current RDI wire payload carries velocity ambiguity only;
# the remaining fields are retained for ROS API compatibility and default to 0.
float32 velocity_ambiguity_begin       # m/s, computed per scan by the sensor (-17.21 to -12.05)
float32 velocity_ambiguity_end         # m/s, computed per scan by the sensor (12.05 to 17.21)
                                       # Detection.radial_velocity is resolved and can lie outside this domain.
float32 range_ambiguity_begin          # meters (always 0)
float32 range_ambiguity_end            # meters (always 0)
float32 azimuth_ambiguity_begin        # radians (always 0)
float32 azimuth_ambiguity_end          # radians (always 0)
float32 elevation_ambiguity_begin      # radians (always 0)
float32 elevation_ambiguity_end        # radians (always 0)

# Detections info
uint16 max_detections                  # RecognisedDetectionsCapability (2048 on interface v1.2.0)
uint8 recognised_detections_status     # legacy, always 0 (detections beyond the capability are cut without indication)
afi920_msgs/Detection[] detections
