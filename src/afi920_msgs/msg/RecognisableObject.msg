# Recognisable object type with detection range
# Maps to bts_spi_recognisable_object_type_t (9 bytes)

uint8 OBJECT_CAR=0
uint8 OBJECT_HEAVY_TRUCK=1
uint8 OBJECT_MOTORBIKE=2
uint8 OBJECT_BICYCLE=3
uint8 OBJECT_PEDESTRIAN=4
uint8 OBJECT_POLE=5
uint8 OBJECT_GUARD_RAIL=6
uint8 OBJECT_BUILDING=7
uint8 OBJECT_TRAFFIC_SIGN=8
uint8 OBJECT_TRAFFIC_LIGHT=9
uint8 object_type

float32 detection_range_begin          # meters
float32 detection_range_end            # meters
