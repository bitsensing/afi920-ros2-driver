# Single radar detection from AFI920 sensor (ISO 23150 RDI)
# Coordinate: sensor frame (X=forward, Y=left, Z=up)
# Current RDI wire detection is 51 bytes. Extra legacy fields below are
# retained for ROS API compatibility and default to 0 when absent on the wire.

# Identification
uint16 detection_id
uint16 object_id_reference             # ObjectIDReference (reserved, 0xFFFF = no tracking)
uint8 existence_probability            # 0-100 %
float32 timestamp_difference           # seconds offset from header timestamp

# Position (spherical, as received from sensor)
float32 radial_distance                # meters (0-300)
float32 azimuth                        # radians (±60°)
float32 elevation                      # radians (±15°)
float32 radial_distance_error          # meters
float32 azimuth_error                  # radians
float32 elevation_error                # radians

# Position (Cartesian, computed from spherical)
float32 x                              # meters
float32 y                              # meters
float32 z                              # meters

# Dynamics
float32 radial_velocity                # m/s (negative=approaching)
float32 radial_velocity_error          # m/s

# Signal quality
float32 rcs                            # dBm² (radar cross section)
float32 rcs_error                      # dBm²
float32 snr                            # dB (signal-to-noise ratio)
float32 snr_error                      # dB

# Probabilities
uint8 multi_target_probability         # 0-100 %
uint16 ambiguity_grouping_id           # ambiguity group ID
uint8 detection_ambiguity_probability  # reserved
uint8 free_space_probability           # reserved

# Classification (up to 8 slots)
uint8 num_classifications
uint8[8] classification_type           # 0=none, 1=noise, 2=obstacle, 3=under, 4=over, 5=nearest, 6=strongest
uint8[8] classification_confidence     # 0-100 per slot

# Debug fields (15 bytes in wire format, user-defined extension to ISO 23150)
float32 debug_power                    # dB, Linear Level NCI Power
uint8 debug_azimuth_method             # 1-4, azimuth estimation method flag
uint8 debug_elevation_method           # 1-2, elevation estimation method flag
uint8 debug_quality_distance           # 0-100%, range quality
uint8 debug_quality_azimuth            # 0-100%, azimuth APS SNR quality
uint8 debug_quality_elevation          # 0-100%, elevation APS SNR quality
uint8 debug_ambiguity_azimuth          # 1-4, Az APS multi-target count
uint8 debug_ambiguity_elevation        # 1-2, El APS multi-target count
uint8 debug_quality_velocity           # 0-100%, velocity quality
uint8 debug_ambiguity_model_velocity   # 0=unknown, 1=resolved (Dual PRF)
uint16 debug_ambiguity_index_velocity  # Dual PRF resolved case fs ID

# Sensor identifier (for multi-sensor)
uint8 sensor_id
