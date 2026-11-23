# Sensor health information from SHII (Event 0x8003, TCP/UDP 30510)
# The SHII format allows variable arrays; the sensor sends N_op 1, N_in 4, N_cal 3.
std_msgs/Header header

uint8 sensor_id
uint32 message_counter
uint64 timestamp_measurement_ns        # sensor measurement timestamp (nanoseconds, PTP)
uint8 data_qualifier                   # 0=normal, 6=invalid (resend of the previous message)

# Operation mode (current mode; the sensor sends one entry)
uint8 num_operation_modes              # number of valid entries (1; array capacity 11)
uint8[11] operation_modes              # SensorOperationMode enum values
# Operation mode enum:
#   0=normal_dual_range, 1=normal_long_range, 2=normal_middle_range,
#   3=normal_short_range, 4=normal_ultra_long_range, 5=normal_ultra_short_range,
#   6=normal_ultra_dual_range, 10=degradation, 20=diagnostic, 21=programming,
#   100=fault, 200=initialising

# Defect status
uint8 DEFECT_FULLY_FUNCTIONAL=0
uint8 DEFECT_NOT_FULLY=1
uint8 DEFECT_OUT_OF_ORDER=2            # not sent on interface v1.2.0
uint8 defect_recognised
uint8 defect_reason                    # 0=none, 1=supply_voltage, 2=hardware, 3=thermal, 5=calibration,
                                       # 7=mechanical, 10=out_of_time_sync, 11=externally_disturbed

# Environmental status
uint8 VOLTAGE_LOW=0
uint8 VOLTAGE_PRE_LOW=1
uint8 VOLTAGE_OK=2
uint8 VOLTAGE_PRE_HIGH=3
uint8 VOLTAGE_HIGH=4
uint8 supply_voltage_status

uint8 TEMP_UNDER=0
uint8 TEMP_PRE_UNDER=1
uint8 TEMP_OK=2
uint8 TEMP_PRE_OVER=3
uint8 TEMP_OVER=4
uint8 temperature_status

# Input signal status (array)
uint8 num_input_signal_statuses        # number of valid entries (max: 4)
uint8[4] input_signal_types            # 0=csii_sensor_operation, 1=environmental_information,
                                       # 2=vehicle_state, 3=sensor_pose
uint8[4] input_signal_statuses         # 0=valid, 1=implausible, 2=missing, 3=out_of_range, 4=timeout

# Time synchronization
uint8 SYNC_OK=0
uint8 SYNC_OUT_OF_LIMITS=1
uint8 SYNC_TIMEOUT=2
uint8 SYNC_NOT_SYNCHRONIZED=3
uint8 time_sync_status
float32 time_sync_offset               # legacy field, 0.0 with current interface

# Sensor calibration status (parallel arrays)
uint8 num_calibration_components       # number of valid entries (max: 3)
uint8[3] calibration_components        # 0=intrinsic, 1=extrinsic_horizontal, 2=extrinsic_vertical
uint8[3] calibration_statuses          # 0=calibrated, 1=not_calibrated, 2=degraded
uint8[3] calibration_states            # 0=completed, 1=not_performed, 2=in_progress, 3=failed
