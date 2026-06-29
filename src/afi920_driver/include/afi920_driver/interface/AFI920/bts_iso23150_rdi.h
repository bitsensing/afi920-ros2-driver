// Copyright (c) 2025-2026 bitsensing Inc. All rights reserved.
/**
 * @file bts_iso23150_rdi.h
 * @brief Radar Detection Interface (RDI) serialization for ISO 23150 - AFI920
 */

#ifndef BTS_ISO23150_RDI_H
#define BTS_ISO23150_RDI_H

#include "../bts_iso23150.h"
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * Constants
 *
 * Current AFI920 interface wire layout:
 *   Header: 36B, Detection: 51B
 *
 * This local ROS driver struct intentionally remains a compatibility superset
 * of older ROS messages; fields no longer present on the wire are zero-filled
 * by the parser.
 *============================================================================*/

#define BTS_RDI_HEADER_SIZE             (36u)
#define BTS_RDI_DETECTION_SIZE          (51u)
#define BTS_RDI_MAX_DETECTIONS          (4096u)
#define BTS_RDI_MAX_CLASSIFICATIONS     (8u)
#define BTS_RDI_MAX_PAYLOAD_SIZE        (BTS_RDI_HEADER_SIZE + BTS_RDI_DETECTION_SIZE * BTS_RDI_MAX_DETECTIONS)

/*============================================================================
 * Detection Classification Types
 *============================================================================*/

typedef enum {
    BTS_RDI_DCT_NO_CLASSIFICATION   = 0,
    BTS_RDI_DCT_NOISE               = 1,
    BTS_RDI_DCT_OBSTACLE            = 2,
    BTS_RDI_DCT_UNDERDRIVEABLE      = 3,
    BTS_RDI_DCT_OVERDRIVEABLE       = 4,
    BTS_RDI_DCT_NEAREST             = 5,
    BTS_RDI_DCT_STRONGEST           = 6,
    BTS_RDI_DCT_RESERVED            = 7,
} bts_rdi_classification_type_t;

/*============================================================================
 * RDI Detection Structure
 *============================================================================*/

typedef struct {
    /* Status */
    uint8_t  existence_probability;         /* 0-100 % */
    uint16_t detection_id;
    uint16_t object_id_reference;
    float    timestamp_difference;          /* seconds */

    /* Signal Quality */
    float    radar_cross_section;           /* dBms */
    float    radar_cross_section_error;
    float    signal_to_noise_ratio;         /* dB */
    float    signal_to_noise_ratio_error;

    /* Probabilities */
    uint8_t  multi_target_probability;      /* 0-100 % */
    uint16_t ambiguity_grouping_id;
    uint8_t  detection_ambiguity_probability;
    uint8_t  free_space_probability;

    /* Classification */
    uint8_t  num_classifications;
    uint8_t  classification_type[BTS_RDI_MAX_CLASSIFICATIONS];       /* 8 slots */
    uint8_t  classification_confidence[BTS_RDI_MAX_CLASSIFICATIONS]; /* 8 slots */

    /* Position */
    float    position_radial_distance;      /* meters */
    float    position_azimuth;              /* radians */
    float    position_elevation;            /* radians */
    float    position_radial_distance_error;
    float    position_azimuth_error;
    float    position_elevation_error;

    /* Velocity */
    float    relative_velocity_radial;      /* m/s */
    float    relative_velocity_radial_error;

    /* Debug */
    float    debug_power;                   /* dB */
    uint8_t  debug_azimuth_method;
    uint8_t  debug_elevation_method;
    uint8_t  debug_quality_distance;        /* 0-100 % */
    uint8_t  debug_quality_azimuth;
    uint8_t  debug_quality_elevation;
    uint8_t  debug_ambiguity_azimuth;
    uint8_t  debug_ambiguity_elevation;
    uint8_t  debug_quality_velocity;
    uint8_t  debug_ambiguity_model_velocity;
    uint16_t debug_ambiguity_index_velocity;
} bts_rdi_detection_t;

/*============================================================================
 * RDI Ambiguity Domain Structure
 *============================================================================*/

typedef struct {
    float radial_velocity_begin;    /* m/s */
    float radial_velocity_end;
    float range_begin;              /* meters */
    float range_end;
    float azimuth_begin;            /* radians */
    float azimuth_end;
    float elevation_begin;          /* radians */
    float elevation_end;
} bts_rdi_ambiguity_domain_t;

/*============================================================================
 * RDI Message Structure
 *============================================================================*/

typedef struct {
    /* Header */
    bts_interface_version_t interface_version;
    uint8_t  interface_id;
    uint8_t  num_serving_sensors;
    uint8_t  sensor_id;
    uint64_t timestamp;                     /* nanoseconds */
    uint32_t message_counter;
    uint32_t interface_cycle_time;          /* nanoseconds */
    uint8_t  interface_cycle_time_variation;/* 0-100 % */
    bts_data_qualifier_t data_qualifier;

    /* Ambiguity domains */
    bts_rdi_ambiguity_domain_t ambiguity_domain;

    /* Detections info */
    uint16_t recognised_detections_capability;
    uint8_t  recognised_detections_status;
    uint16_t num_detections;

    /* Detections array (external allocation recommended for large data) */
    bts_rdi_detection_t* detections;
} bts_rdi_message_t;

/*============================================================================
 * Inline Helpers
 *============================================================================*/

/**
 * @brief Initialize detection with default values
 */
static inline void bts_rdi_detection_init(bts_rdi_detection_t* det)
{
    memset(det, 0, sizeof(*det));
}

#ifdef __cplusplus
}
#endif

#endif /* BTS_ISO23150_RDI_H */
