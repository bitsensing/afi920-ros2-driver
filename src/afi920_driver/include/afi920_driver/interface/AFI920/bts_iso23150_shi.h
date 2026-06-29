// Copyright (c) 2025-2026 bitsensing Inc. All rights reserved.
/**
 * @file bts_iso23150_shi.h
 * @brief Sensor Health Information Interface (SHII) types for AFI920.
 *
 * Current SHII payload is variable length:
 *   CIH(24) | 1+Nop | defect/supply/temp | 1+2*Ninput |
 *   time_sync | 1+3*Ncal
 */

#ifndef BTS_ISO23150_SHI_H
#define BTS_ISO23150_SHI_H

#include "../bts_iso23150.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BTS_SHI_MAX_OPERATION_MODES         (11u)
#define BTS_SHI_MAX_INPUT_SIGNALS           (4u)
#define BTS_SHI_MAX_CALIBRATION_COMPONENTS  (3u)

#define BTS_SHI_MAX_SIZE                                                        \
    (24u + 1u + BTS_SHI_MAX_OPERATION_MODES + 2u + 2u                          \
     + 1u + 2u * BTS_SHI_MAX_INPUT_SIGNALS + 1u                                \
     + 1u + 3u * BTS_SHI_MAX_CALIBRATION_COMPONENTS)

typedef enum {
    BTS_SHI_SOM_NORMAL_DUAL_RANGE        = 0,
    BTS_SHI_SOM_NORMAL_LONG_RANGE        = 1,
    BTS_SHI_SOM_NORMAL_MIDDLE_RANGE      = 2,
    BTS_SHI_SOM_NORMAL_SHORT_RANGE       = 3,
    BTS_SHI_SOM_NORMAL_ULTRA_LONG_RANGE  = 4,
    BTS_SHI_SOM_NORMAL_ULTRA_SHORT_RANGE = 5,
    BTS_SHI_SOM_DEGRADATION_MODE         = 10,
    BTS_SHI_SOM_EVALUATION_MODE          = 50,
    BTS_SHI_SOM_CALIBRATION_MODE         = 100,
    BTS_SHI_SOM_INITIALISING             = 200,
    BTS_SHI_SOM_TEST_MODE                = 201,
} bts_shi_sensor_operation_mode_t;

typedef enum {
    BTS_SHI_SDR_FULLY_FUNCTIONAL        = 0,
    BTS_SHI_SDR_NOT_FULLY_FUNCTIONAL    = 1,
    BTS_SHI_SDR_OUT_OF_ORDER            = 2,
} bts_shi_sensor_defect_recognised_t;

typedef enum {
    BTS_SHI_SDR_NO_DEFECT               = 0,
    BTS_SHI_SDR_INTERNAL_MEMORY_ERROR   = 1,
    BTS_SHI_SDR_HW_DEFECT               = 2,
    BTS_SHI_SDR_THERMAL_DEFECT          = 3,
    BTS_SHI_SDR_COMMUNICATION_ERROR     = 4,
    BTS_SHI_SDR_CALIBRATION_ERROR       = 5,
    BTS_SHI_SDR_CONFIGURATION_ERROR     = 6,
    BTS_SHI_SDR_MECHANICAL_DEFECT       = 7,
    BTS_SHI_SDR_SOFTWARE_DEFECT         = 8,
    BTS_SHI_SDR_COMPUTING_POWER         = 9,
    BTS_SHI_SDR_OUT_OF_TIME_SYNC        = 10,
    BTS_SHI_SDR_EXTERNALLY_DISTURBED    = 11,
} bts_shi_sensor_defect_reason_t;

typedef enum {
    BTS_SHI_SVS_LOW                     = 0,
    BTS_SHI_SVS_PRE_LOW                 = 1,
    BTS_SHI_SVS_WITHIN_LIMITS           = 2,
    BTS_SHI_SVS_PRE_HIGH                = 3,
    BTS_SHI_SVS_HIGH                    = 4,
} bts_shi_supply_voltage_status_t;

typedef enum {
    BTS_SHI_STS_UNDER_TEMPERATURE       = 0,
    BTS_SHI_STS_PRE_UNDER_TEMPERATURE   = 1,
    BTS_SHI_STS_TEMPERATURE_IN_LIMITS   = 2,
    BTS_SHI_STS_PRE_OVER_TEMPERATURE    = 3,
    BTS_SHI_STS_OVER_TEMPERATURE        = 4,
} bts_shi_sensor_temperature_status_t;

typedef enum {
    BTS_SHI_SIST_CSII_SENSOR_OPERATION  = 0,
    BTS_SHI_SIST_CSII_ENV_INFORMATION   = 1,
    BTS_SHI_SIST_CSII_VEHICLE_STATE     = 2,
    BTS_SHI_SIST_CSII_SENSOR_POSE       = 3,
} bts_shi_sensor_input_signal_type_t;

typedef enum {
    BTS_SHI_SISS_VALID                  = 0,
    BTS_SHI_SISS_IMPLAUSIBLE            = 1,
    BTS_SHI_SISS_MISSING                = 2,
    BTS_SHI_SISS_OUT_OF_RANGE           = 3,
    BTS_SHI_SISS_TIMEOUT                = 4,
} bts_shi_sensor_input_signal_status_t;

typedef enum {
    BTS_SHI_STS_WITHIN_LIMITS           = 0,
    BTS_SHI_STS_OUT_OF_LIMITS           = 1,
    BTS_SHI_STS_TIMEOUT                 = 2,
    BTS_SHI_STS_NOT_SYNCHRONIZED        = 3,
} bts_shi_sensor_time_sync_t;

typedef enum {
    BTS_SHI_SCC_INTRINSIC                   = 0,
    BTS_SHI_SCC_EXTRINSIC_ONLINE_HORIZONTAL = 1,
    BTS_SHI_SCC_EXTRINSIC_ONLINE_VERTICAL   = 2,
} bts_shi_sensor_calibration_component_t;

typedef enum {
    BTS_SHI_SCS_CALIBRATED              = 0,
    BTS_SHI_SCS_NOT_CALIBRATED          = 1,
    BTS_SHI_SCS_DEGRADED                = 2,
} bts_shi_sensor_calibration_status_t;

typedef enum {
    BTS_SHI_CPS_INITIAL_CAL_PERFORMED       = 0,
    BTS_SHI_CPS_INITIAL_CAL_NOT_PERFORMED   = 1,
    BTS_SHI_CPS_INITIAL_CAL_FAILED          = 2,
    BTS_SHI_CPS_RECAL_NEEDED_INTRINSIC      = 3,
    BTS_SHI_CPS_RECAL_NEEDED_EXTRINSIC      = 4,
    BTS_SHI_CPS_RECAL_NEEDED_FULL           = 5,
} bts_shi_sensor_calibration_state_t;

typedef struct {
    bts_interface_version_t interface_version;
    uint8_t  interface_id;
    uint8_t  num_serving_sensors;
    uint8_t  sensor_id;
    uint64_t timestamp;
    uint32_t message_counter;
    uint32_t interface_cycle_time;
    uint8_t  interface_cycle_time_variation;
    bts_data_qualifier_t data_qualifier;

    uint8_t  num_valid_operation_modes;
    uint8_t  sensor_operation_modes[BTS_SHI_MAX_OPERATION_MODES];

    bts_shi_sensor_defect_recognised_t sensor_defect_recognised;
    bts_shi_sensor_defect_reason_t sensor_defect_reason;
    bts_shi_supply_voltage_status_t supply_voltage_status;
    bts_shi_sensor_temperature_status_t sensor_temperature_status;

    uint8_t  num_valid_input_signal_statuses;
    uint8_t  sensor_input_signal_types[BTS_SHI_MAX_INPUT_SIGNALS];
    uint8_t  sensor_input_signal_statuses[BTS_SHI_MAX_INPUT_SIGNALS];

    bts_shi_sensor_time_sync_t sensor_time_sync;

    uint8_t  num_valid_calibration_components;
    uint8_t  sensor_calibration_components[BTS_SHI_MAX_CALIBRATION_COMPONENTS];
    uint8_t  sensor_calibration_statuses[BTS_SHI_MAX_CALIBRATION_COMPONENTS];
    uint8_t  sensor_calibration_states[BTS_SHI_MAX_CALIBRATION_COMPONENTS];

    float    sensor_time_sync_offset_value;
} bts_shi_message_t;

#ifdef __cplusplus
}
#endif

#endif  // BTS_ISO23150_SHI_H
