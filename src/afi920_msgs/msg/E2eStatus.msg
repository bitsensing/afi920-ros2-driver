# E2E verification result of one received RDI / SHI / SPI message (~/E2E).
# header.stamp equals the header.stamp of the PointCloud2 / RDI / SHI / SPI
# message published from the same sensor message, so the two can be paired
# by exact time. When data_published is false no data message was published
# and header.stamp is the verification time.
# On sensor interface v1.2.0 one message is published per RDI scan (header and
# chunk messages share one E2E Counter): result is the worst result of those
# messages and length_ok / data_id_ok are false when any of them failed.
std_msgs/Header header

uint8 STREAM_RDI=0
uint8 STREAM_SHI=1
uint8 STREAM_SPI=2

uint8 RESULT_OK=0
uint8 RESULT_CRC_MISMATCH=1
uint8 RESULT_CRC_ZERO=2                # CRC field is 0; not treated as a pass
uint8 RESULT_NOT_CHECKED=3             # validate_e2e is false

uint8 CRC_FORMAT_UNKNOWN=0
uint8 CRC_FORMAT_CRC32=1               # sensor interface v1.1.0+
uint8 CRC_FORMAT_CRC64_XZ=2            # sensor interface v1.0.x

uint8 stream                           # STREAM_*
uint8 result                           # RESULT_*
uint8 crc_format                       # CRC_FORMAT_* (UNKNOWN unless result is RESULT_OK)
bool length_ok                         # E2E Length equals the payload size
bool data_id_ok                        # E2E Data ID matches the stream
uint32 e2e_counter                     # E2E Counter
uint32 missing_before                  # E2E Counter values skipped since the previous message
bool sensor_restarted                  # E2E Counter decreased (sensor restart)
bool data_published                    # false: dropped by e2e_strict or payload parse failure
uint32 message_counter                 # ISO 23150 MessageCounter (0 when not parsed)
uint8 data_qualifier                   # ISO 23150 DataQualifier (0 when not parsed)
