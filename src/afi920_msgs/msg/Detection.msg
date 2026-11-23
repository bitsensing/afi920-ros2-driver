# Single radar detection from AFI920 sensor (ISO 23150 RDI)
# Coordinate: sensor frame (X=forward, Y=left, Z=up)
# Current RDI wire detection is 51 bytes. Fields marked "legacy, always 0" are
# not on the wire; they are retained for ROS API compatibility and are 0.

# Identification
uint16 detection_id
uint16 object_id_reference             # ObjectIDReference (always 0: no tracking)
uint8 existence_probability            # 1-100 % (90-100 real, 70-89 suspected ghost, 50-69 likely ghost, 1-49 ghost;
                                       # 98/97/95/93 = range/angle/range+angle/velocity shape-estimation points)
float32 timestamp_difference           # s, from DetectionArray.timestamp_measurement_ns to the centre of this
                                       # detection's measurement interval (not from header.stamp)

# Position (spherical, as received from sensor)
float32 radial_distance                # meters (design range 0-300, not saturated)
float32 azimuth                        # radians (design range +-60 deg; up to +-61 deg can occur)
float32 elevation                      # radians (design range +-15 deg; up to +-16 deg can occur)
float32 radial_distance_error          # meters, fixed accuracy per Range Mode and sub-cycle (0.025-0.35)
float32 azimuth_error                  # radians, fixed accuracy 0.001745 (0.1 deg)
float32 elevation_error                # radians, fixed accuracy 0.001745 (0.1 deg)

# Position (Cartesian, computed from spherical)
float32 x                              # meters
float32 y                              # meters
float32 z                              # meters

# Dynamics
float32 radial_velocity                # m/s (negative=approaching)
float32 radial_velocity_error          # m/s, fixed accuracy 0.065

# Signal quality
float32 rcs                            # dBsm (radar cross section, -20 to 40)
float32 rcs_error                      # dBsm (legacy, always 0)
float32 snr                            # dB (signal-to-noise ratio, 0 to 254)
float32 snr_error                      # dB (legacy, always 0)

# Probabilities
uint8 multi_target_probability         # legacy, always 0
uint16 ambiguity_grouping_id           # ambiguity group ID (always 0: not supported)
uint8 detection_ambiguity_probability  # legacy, always 0
uint8 free_space_probability           # legacy, always 0

# Classification (legacy, always 0)
uint8 num_classifications
uint8[8] classification_type           # legacy enum: 0=none, 1=noise, 2=obstacle, 3=under, 4=over, 5=nearest, 6=strongest
uint8[8] classification_confidence     # legacy, 0-100 per slot

# Debug fields (legacy, always 0: the current wire detection carries no debug block)
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
