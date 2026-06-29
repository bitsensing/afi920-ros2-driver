# Field-of-view segment with blockage status
# Maps to bts_spi_fov_segment_t (17 bytes)

float32 azimuth_begin                  # radians
float32 azimuth_end                    # radians
float32 elevation_begin                # radians
float32 elevation_end                  # radians

uint8 BLOCKAGE_NONE=0
uint8 BLOCKAGE_PARTIAL=1
uint8 BLOCKAGE_FULL=2
uint8 BLOCKAGE_UNKNOWN=3
uint8 blockage_status
