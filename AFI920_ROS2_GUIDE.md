# AFI920 ROS 2 Driver — AI Integration Guide

> **Who this is for:** AI agents, LLM-driven automation, and developers who need the fastest path to integrating the bitsensing AFI920 4D imaging radar into a ROS 2 system.  
> **Driver version:** 2.2.0 · **Supported ROS 2:** Foxy (Ubuntu 20.04), Humble (Ubuntu 22.04), Jazzy (Ubuntu 24.04, aarch64) · **Arch:** x86_64, aarch64  
> **License:** BSD-3-Clause

---

## Table of Contents

1. [What the Driver Does](#1-what-the-driver-does)
2. [Repository Layout](#2-repository-layout)
3. [Network Prerequisites](#3-network-prerequisites)
4. [Build & Install](#4-build--install)
5. [Quickstart: Single Sensor](#5-quickstart-single-sensor)
6. [Published Topics](#6-published-topics)
7. [All Node Parameters](#7-all-node-parameters)
8. [Multi-Sensor Setup](#8-multi-sensor-setup)
9. [Custom Message Definitions](#9-custom-message-definitions)
10. [PointCloud2 Field Reference](#10-pointcloud2-field-reference)
11. [Dynamic Parameter Tuning](#11-dynamic-parameter-tuning)
12. [Diagnostic Tools](#12-diagnostic-tools)
13. [Port & Protocol Reference](#13-port--protocol-reference)
14. [Troubleshooting Checklist](#14-troubleshooting-checklist)

---

## 1. What the Driver Does

The AFI920 ROS 2 driver is a self-contained ROS 2 node with its own SOME/IP, E2E and ISO 23150 receivers; it does not depend on the AFI920 SDK. It connects to the radar over Ethernet and publishes four data topics at ~10 Hz:

| What you get | Topic | Type |
|---|---|---|
| Standard point cloud (RViz-ready) | `PointCloud2` | `sensor_msgs/PointCloud2` |
| Full ISO 23150 detection data | `RDI` | `afi920_msgs/DetectionArray` |
| Sensor health & fault status | `SHI` | `afi920_msgs/HealthInfo` |
| Sensor performance & mounting pose | `SPI` | `afi920_msgs/SensorPerformance` |

Topic names are relative to the node namespace (default `afi920`, so `PointCloud2` is `/afi920/PointCloud2`). In addition, it publishes a `detection_count` RViz overlay marker, the E2E verification result of each RDI/SHI/SPI message (one per scan on interface v1.2.0 RDI) on `E2E`, and broadcasts the sensor mounting pose (from SPI) as a TF transform `parent_frame_id → frame_id`. See [§6](#6-published-topics).

All data topics use `SensorDataQoS`. The driver supports TCP (default) and UDP transport, E2E CRC verification for sensor interface v1.0.x and v1.1.0+ (detected from received frames, no parameter), SOME/IP-TP segment reassembly, interface v1.2.0 RDI scan assembly from header and chunk messages, TF broadcast of the sensor pose, and runtime filter adjustment via `ros2 param set`.

---

## 2. Repository Layout

```
afi920-ros2-driver/
├── run.sh                          ← convenience build + launch script
├── src/
│   ├── afi920_msgs/                ← custom message package
│   │   └── msg/
│   │       ├── Detection.msg       ← single detection (full ISO 23150)
│   │       ├── DetectionArray.msg  ← RDI frame with header
│   │       ├── HealthInfo.msg      ← SHII sensor health
│   │       ├── SensorPerformance.msg ← SPI frame
│   │       ├── SensorPose.msg      ← mounting position & orientation
│   │       ├── FovSegment.msg      ← FoV blockage per angular segment
│   │       ├── RecognisableObject.msg ← per-object-type detection range
│   │       ├── ReferenceTarget.msg ← reference target characteristics
│   │       └── E2eStatus.msg       ← per-message E2E verification result
│   └── afi920_driver/
│       ├── src/afi920_node.cpp     ← main composable node
│       ├── config/                 ← parameter YAML, afi920.rviz, multi_sensor.rviz, dds_profile.xml
│       └── launch/
│           ├── afi920.launch.py        ← single sensor
│           └── multi_sensor.launch.py  ← four sensors (front, rear, left, right)
└── tools/
    ├── live_data_check.py   ← end-to-end sensor verification
    └── rdi_diag.py          ← SOME/IP + E2E + RDI byte-level debugger
```

---

## 3. Network Prerequisites

| Item | Default |
|------|---------|
| Sensor IP | `192.168.10.150` |
| Host IP | `192.168.10.x` |
| Subnet mask | `255.255.255.0` |

**Required before running:**
1. Connect host to radar over Ethernet.
2. Assign the host a static IP in the `192.168.10.x` subnet.
3. Verify: `ping 192.168.10.150`

---

## 4. Build & Install

### Standard colcon build

```bash
source /opt/ros/$ROS_DISTRO/setup.bash   # foxy, humble or jazzy

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
# place afi920-ros2-driver/ here (git clone or copy)

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select afi920_msgs afi920_driver \
             --symlink-install
source install/setup.bash
```

> Build `afi920_msgs` first — `afi920_driver` depends on it. `colcon` handles the order automatically.

### One-command shortcut (run.sh)

```bash
cd afi920-ros2-driver
./run.sh                    # build + launch single sensor + RViz
./run.sh --no-build         # skip build, launch + RViz only
./run.sh --multi            # build + launch 4-sensor + RViz
./run.sh --no-build --multi # skip build, 4-sensor + RViz
```

### Package dependencies

| Package | Key deps |
|---------|---------|
| `afi920_msgs` | `std_msgs`, `sensor_msgs`, `rosidl_default_generators` |
| `afi920_driver` | `rclcpp`, `rclcpp_components`, `sensor_msgs`, `std_msgs`, `visualization_msgs`, `geometry_msgs`, `tf2`, `tf2_ros`, `afi920_msgs` |

---

## 5. Quickstart: Single Sensor

```bash
# Minimal launch — all defaults
ros2 launch afi920_driver afi920.launch.py

# Override common options
ros2 launch afi920_driver afi920.launch.py \
    sensor_ip:=192.168.10.150 \
    namespace:=afi920 \
    frame_id:=afi920 \
    transport_mode:=tcp
```

**Verify data is flowing:**
```bash
ros2 topic hz /afi920/PointCloud2      # expect ~10 Hz
ros2 topic echo /afi920/RDI --once
ros2 topic echo /afi920/SHI --once
ros2 topic echo /afi920/SPI --once
ros2 topic echo /afi920/E2E --once      # result 0 = OK, crc_format 1 = CRC-32, 2 = CRC-64/XZ
```

---

## 6. Published Topics

Default namespace: `afi920` (set via `namespace` launch arg).  
Topic names are declared as relative names (e.g. `PointCloud2`), so the namespace prefix is applied automatically to produce the absolute topics below.

| Topic (absolute) | Message type | QoS | Rate |
|---|---|---|---|
| `/afi920/PointCloud2` | `sensor_msgs/PointCloud2` | SensorDataQoS | 10 Hz |
| `/afi920/RDI` | `afi920_msgs/DetectionArray` | SensorDataQoS | 10 Hz |
| `/afi920/SHI` | `afi920_msgs/HealthInfo` | SensorDataQoS | 10 Hz |
| `/afi920/SPI` | `afi920_msgs/SensorPerformance` | SensorDataQoS | 10 Hz |
| `/afi920/detection_count` | `visualization_msgs/Marker` | SensorDataQoS | 10 Hz |
| `/afi920/E2E` | `afi920_msgs/E2eStatus` | SensorDataQoS | 30 Hz (RDI + SHI + SPI) |

`detection_count` is a `TEXT_VIEW_FACING` marker (`Pts: <n>`) for RViz overlay; it is always published alongside `PointCloud2`.

**DDS profile:** both launch files set `FASTRTPS_DEFAULT_PROFILES_FILE` to `config/dds_profile.xml` for the node. The profile disables the Fast DDS built-in transports (including shared memory) and uses one UDPv4 transport with 1 MB send/receive buffers and a 65000 B maximum message size. Subscribers on the same host therefore receive over UDP loopback.

**DataQualifier:** RDI frames with DataQualifier 1 (record not complete: header messages only, no detections, header fields repeat the previous header) or 6 (resend of the previous scan) carry no new measurement. They are excluded from `PointCloud2` and `detection_count` and are still published on `RDI` with `data_qualifier` set.

**E2E:** one `E2eStatus` message is published for every received RDI/SHI/SPI message that carries an E2E header, including frames dropped by `e2e_strict`. On interface v1.2.0, RDI is reported once per scan; a late message of an already closed scan is ignored and not reported. SHI and SPI are not received, and so not reported, when `publish_health` or `publish_performance` is `false`. Its `header.stamp` equals the stamp of the matching `PointCloud2`/`RDI`/`SHI`/`SPI` message, so the two can be paired by exact time. See [§9](#afi920_msgse2estatus--per-message-e2e-result).

**TF broadcast:** when `publish_tf` is `true` (default) and `publish_performance` is `true`, the node broadcasts the sensor mounting pose received in each SPI frame to `/tf` (`tf2_msgs/TFMessage`) as the transform `parent_frame_id → frame_id` (default `base_link → afi920`). This lets RViz/consumers place each sensor's cloud at its mounting position when the fixed frame is set to `parent_frame_id`. The pose is the configured mounting pose without auto-alignment correction, and is all zero until the sensor has applied its configuration. If `publish_performance` is `false`, no SPI pose is available and TF is not broadcast (a warning is logged).

**Coordinate convention:** sensor-frame FLU (X forward, Y left, Z up) — REP-103 compliant.

---

## 7. All Node Parameters

Parameters marked "not dynamic" are read once at startup. `ros2 param set` on them is rejected with the reason `cannot be changed at runtime`; restart the node to change them.

### Connection (not dynamic — set at launch)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `sensor_ip` | string | `192.168.10.150` | Radar IP address |
| `host_ip` | string | `auto` | Host IP for the sensor stream destination (`auto` = detected from route to sensor). Interface v1.0.x: written with `SetNetworkConfig` at startup. v1.1.0+: compared with the sensor's UDP destination when `transport_mode` is `udp`. Receive sockets are not bound to it |
| `data_port` | int | `30509` | RDI receive port |
| `shi_port` | int | `30510` | SHII receive port |
| `spi_port` | int | `30511` | SPI receive port |
| `config_port` | int | `30500` | Config API TCP port |
| `transport_mode` | string | `tcp` | `tcp` or `udp` |

### Publishing (not dynamic)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `publish_detections` | bool | `true` | Publish `RDI` topic |
| `publish_health` | bool | `true` | Publish `SHI` topic |
| `publish_performance` | bool | `true` | Publish `SPI` topic |

### TF (not dynamic)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `publish_tf` | bool | `true` | Broadcast sensor mounting pose (from SPI) as TF. Requires `publish_performance:=true`. |
| `parent_frame_id` | string | `base_link` | Parent TF frame; pose is broadcast as `parent_frame_id → frame_id` |

### Filtering & frame (dynamic — change at runtime)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `frame_id` | string | `afi920` | TF frame ID on all published messages |
| `min_range` | double | `0.0` m | Drop detections closer than this (applies to `PointCloud2` and `RDI`) |
| `max_range` | double | `300.0` m | Drop detections farther than this (applies to `PointCloud2` and `RDI`) |
| `min_snr` | double | `0.0` dB | Drop detections below this SNR (applies to `PointCloud2` and `RDI`) |
| `min_existence_probability` | double | `0.0` % | Drop detections below this confidence (applies to `PointCloud2` and `RDI`) |

### E2E integrity (dynamic)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `validate_e2e` | bool | `true` | Verify the E2E CRC on every RDI/SHI/SPI frame (CRC-32 on interface v1.1.0+, CRC-64/XZ on v1.0.x). Results go to `E2E`; CRC mismatches are logged and the frame is still published |
| `e2e_strict` | bool | `false` | If `true`, drop frames whose E2E Length or Data ID does not match. CRC mismatches never drop a frame |

### Full launch-argument list (afi920.launch.py)

```bash
ros2 launch afi920_driver afi920.launch.py \
    sensor_ip:=192.168.10.150 \
    namespace:=afi920 \
    frame_id:=afi920 \
    parent_frame_id:=base_link \
    publish_tf:=true \
    data_port:=30509 \
    shi_port:=30510 \
    spi_port:=30511 \
    config_port:=30500 \
    host_ip:=auto \
    transport_mode:=tcp \
    min_range:=0.0 \
    max_range:=300.0 \
    min_snr:=0.0 \
    min_existence_probability:=0.0 \
    publish_detections:=true \
    publish_health:=true \
    publish_performance:=true \
    validate_e2e:=true \
    e2e_strict:=false \
    config_file:=""
```

---

## 8. Multi-Sensor Setup

`multi_sensor.launch.py` always starts four nodes (`front`, `rear`, `left`, `right`), each in its own namespace. A node whose sensor is absent keeps retrying its TCP connection (see [§14](#14-troubleshooting-checklist)); for fewer sensors, launch `afi920.launch.py` once per sensor.

### Launch

```bash
ros2 launch afi920_driver multi_sensor.launch.py \
    front_ip:=192.168.10.150 \
    rear_ip:=192.168.10.151 \
    left_ip:=192.168.10.152 \
    right_ip:=192.168.10.153
```

### Resulting namespaces & topics

| Sensor position | Namespace | Frame ID | Example topic |
|----------------|-----------|----------|---------------|
| Front | `radar_front` | `radar_front` | `/radar_front/PointCloud2` |
| Rear | `radar_rear` | `radar_rear` | `/radar_rear/PointCloud2` |
| Left | `radar_left` | `radar_left` | `/radar_left/PointCloud2` |
| Right | `radar_right` | `radar_right` | `/radar_right/PointCloud2` |

### Per-sensor frame override

```bash
ros2 launch afi920_driver multi_sensor.launch.py \
    front_ip:=192.168.10.150  front_frame:=radar_front \
    rear_ip:=192.168.10.151   rear_frame:=radar_rear
```

### Launch arguments

Per-sensor (one pair for each of `front` / `rear` / `left` / `right`):

| Argument | Default | Description |
|----------|---------|-------------|
| `<pos>_ip` | `192.168.10.150…153` | Sensor IP for that position |
| `<pos>_frame` | `radar_<pos>` | TF frame ID for that sensor |

Shared across all four sensors:

| Argument | Default | Description |
|----------|---------|-------------|
| `transport_mode` | `tcp` | `tcp` or `udp` |
| `validate_e2e` | `true` | Validate the E2E CRC on data streams (results on `E2E`, no drop) |
| `e2e_strict` | `false` | Drop frames on E2E Length/Data ID mismatch |
| `parent_frame` | `base_link` | Common parent TF frame; each pose is broadcast as `parent_frame → <pos>_frame` |
| `publish_tf` | `true` | Broadcast each sensor's mounting pose (from SPI) as TF |
| `config_file` | `""` | Path to a YAML config file (e.g. `config/multi_sensor.yaml`). When set, the nodes take their parameters only from this file and the other arguments (`<pos>_ip`, `transport_mode`, …) are not applied |

`publish_detections`, `publish_health`, and `publish_performance` are hard-wired to `true` for every sensor in this launch file (not exposed as arguments). To toggle them, use a `config_file`.

### Ports

`multi_sensor.launch.py` does **not** expose or set per-sensor ports. The port rule depends on the transport:

| Transport | Port rule |
|-----------|-----------|
| `tcp` | All sensors can keep the same default ports (30509/30510/30511). Each node connects to its own sensor IP, and the host side uses ephemeral ports. |
| `udp` | Each sensor **must** send to its own set of host ports. A node receives on its UDP ports from any address and does not filter by sensor IP, so two sensors that send to the same host port are mixed. |

For UDP, set each radar's stream destination ports (diagnostic mode, `SetNetworkConfig`, restart), then pass a `config_file` that sets `transport_mode: "udp"` and the matching `data_port`/`shi_port`/`spi_port` for each sensor. The `transport_mode` launch argument is not applied when `config_file` is given. Example (the port values are an illustration):

```yaml
radar_front:
  afi920_driver:
    ros__parameters:
      sensor_ip: "192.168.10.150"
      transport_mode: "udp"
      data_port: 30509
      shi_port: 30510
      spi_port: 30511
radar_rear:
  afi920_driver:
    ros__parameters:
      sensor_ip: "192.168.10.151"
      transport_mode: "udp"
      data_port: 30609
      shi_port: 30610
      spi_port: 30611
```

---

## 9. Custom Message Definitions

### `afi920_msgs/DetectionArray` — RDI frame

```
std_msgs/Header header
uint8  sensor_id
uint32 message_counter
uint64 timestamp_measurement_ns      # sensor PTP timestamp (ns)
uint8  data_qualifier                # 0=normal 1=not_available (record not complete: no detections,
                                     # header fields repeat the previous header) 6=invalid (resend)
                                     # 2-5 are not sent; interface v1.0.x can send 5=test_mode

float32 velocity_ambiguity_begin     # m/s, computed per scan (-17.21 to -12.05)
float32 velocity_ambiguity_end       # m/s, computed per scan (12.05 to 17.21);
                                     # Detection.radial_velocity is resolved and can lie outside it
float32 range_ambiguity_begin        # m    (always 0, not on the wire)
float32 range_ambiguity_end          # m    (always 0)
float32 azimuth_ambiguity_begin      # rad  (always 0)
float32 azimuth_ambiguity_end        # rad  (always 0)
float32 elevation_ambiguity_begin    # rad  (always 0)
float32 elevation_ambiguity_end      # rad  (always 0)

uint16 max_detections                # RecognisedDetectionsCapability (2048 on interface v1.2.0)
uint8  recognised_detections_status  # legacy, always 0
afi920_msgs/Detection[] detections   # detections that passed the driver filters
```

### `afi920_msgs/Detection` — single detection

```
# Identification
uint16  detection_id
uint16  object_id_reference    # always 0 (no tracking)
uint8   existence_probability  # 1–100 %: 90–100 real, 70–89 suspected ghost, 50–69 likely ghost,
                               # 1–49 ghost; 98/97/95/93 = range/angle/range+angle/velocity shape-estimation points
float32 timestamp_difference   # s, from timestamp_measurement_ns to the centre of the
                               # detection's measurement interval (not from header.stamp)

# Position (spherical, as received from sensor)
float32 radial_distance        # m (design range 0–300, not saturated)
float32 azimuth                # rad (design range ±60°; detections up to ±61° can occur)
float32 elevation              # rad (design range ±15°; detections up to ±16° can occur)
float32 radial_distance_error  # m, fixed accuracy per Range Mode and sub-cycle (0.025–0.35)
float32 azimuth_error          # rad, fixed accuracy 0.001745 (0.1°)
float32 elevation_error        # rad, fixed accuracy 0.001745 (0.1°)

# Position (Cartesian, computed from spherical; X=forward, Y=left, Z=up)
float32 x                      # m
float32 y                      # m
float32 z                      # m

# Dynamics
float32 radial_velocity        # m/s (negative = approaching)
float32 radial_velocity_error  # m/s, fixed accuracy 0.065

# Signal quality
float32 rcs                    # dBsm (radar cross section, -20 to 40)
float32 rcs_error              # legacy, always 0
float32 snr                    # dB (0 to 254)
float32 snr_error              # legacy, always 0

# Probabilities
uint8   multi_target_probability        # legacy, always 0
uint16  ambiguity_grouping_id           # always 0 (not supported)
uint8   detection_ambiguity_probability # legacy, always 0
uint8   free_space_probability          # legacy, always 0

# Classification (legacy, always 0)
uint8   num_classifications
uint8[8] classification_type   # 0=none 1=noise 2=obstacle 3=under 4=over 5=nearest 6=strongest
uint8[8] classification_confidence  # 0–100 per slot

# Debug fields (legacy, always 0; not on the wire)
float32 debug_power
uint8   debug_azimuth_method
uint8   debug_elevation_method
uint8   debug_quality_distance
uint8   debug_quality_azimuth
uint8   debug_quality_elevation
uint8   debug_ambiguity_azimuth
uint8   debug_ambiguity_elevation
uint8   debug_quality_velocity
uint8   debug_ambiguity_model_velocity
uint16  debug_ambiguity_index_velocity

# Sensor identifier (for multi-sensor)
uint8   sensor_id
```

### `afi920_msgs/HealthInfo` — SHII sensor health

```
std_msgs/Header header
uint8  sensor_id
uint32 message_counter
uint64 timestamp_measurement_ns
uint8  data_qualifier          # 0=normal 6=invalid (resend)

# Operation mode (current mode; the sensor sends 1 entry, array capacity 11)
uint8   num_operation_modes
uint8[11] operation_modes
# Values: 0=normal_dual_range  1=normal_long_range  2=normal_middle_range
#         3=normal_short_range 4=normal_ultra_long_range  5=normal_ultra_short_range
#         6=normal_ultra_dual_range  10=degradation  20=diagnostic  21=programming
#         100=fault  200=initialising

# Defect status
uint8 defect_recognised        # 0=fully_functional  1=not_fully (2=out_of_order is not sent on v1.2.0)
uint8 defect_reason            # 0=none  1=supply_voltage  2=hardware  3=thermal  5=calibration
                               # 7=mechanical  10=out_of_time_sync  11=externally_disturbed

# Supply & temperature
uint8 supply_voltage_status    # 0=low  1=pre_low  2=ok  3=pre_high  4=high
uint8 temperature_status       # 0=under  1=pre_under  2=ok  3=pre_over  4=over

# Input signal status (the sensor sends 4 entries)
uint8   num_input_signal_statuses
uint8[4] input_signal_types     # 0=csii_sensor_operation 1=environmental_information
                                # 2=vehicle_state 3=sensor_pose
uint8[4] input_signal_statuses  # 0=valid 1=implausible 2=missing 3=out_of_range 4=timeout

# Time sync
uint8   time_sync_status       # 0=ok  1=out_of_limits  2=timeout  3=not_synchronized
float32 time_sync_offset       # legacy, always 0.0

# Calibration (parallel arrays, the sensor sends 3 components)
uint8   num_calibration_components
uint8[3] calibration_components # 0=intrinsic  1=extrinsic_horizontal  2=extrinsic_vertical
uint8[3] calibration_statuses   # 0=calibrated  1=not_calibrated  2=degraded
uint8[3] calibration_states     # 0=completed 1=not_performed 2=in_progress 3=failed
```

### `afi920_msgs/SensorPerformance` — SPI frame

```
std_msgs/Header header
uint8  sensor_id
uint32 message_counter
uint64 timestamp_measurement_ns
uint8  data_qualifier                # 0=normal 6=invalid (resend)
uint8  vehicle_coordinate_system     # 1=road_level (always; 0=rear_axle is not sent)

afi920_msgs/SensorPose sensor_pose
afi920_msgs/FovSegment[]        fov_segments           # 2 in DUAL(0)/UDR(6) ([0] narrow LR, [1] wide SR),
                                                        # 1 in other modes, 0 before the configuration is applied
afi920_msgs/RecognisableObject[] recognisable_objects   # the sensor sends 0 entries
afi920_msgs/ReferenceTarget[]    reference_targets      # the sensor sends 0 entries
```

### `afi920_msgs/SensorPose`

```
float32 origin_x / origin_y / origin_z          # mounting position (m)
float32 origin_error_x / origin_error_y / origin_error_z  # legacy, always 0
float32 yaw / pitch / roll                       # configured mounting orientation (rad),
                                                 # no auto-alignment correction
float32 yaw_error / pitch_error                  # auto-alignment misalignment estimate (rad),
                                                 # limited to ±3° (±0.0524 rad)
float32 roll_error                               # legacy, always 0
```

### `afi920_msgs/FovSegment`

```
float32 azimuth_begin / azimuth_end     # rad
float32 elevation_begin / elevation_end # rad
uint8   blockage_status                 # 0=none  1=partial  2=full  (3=unknown is not sent on v1.2.0)
```

### `afi920_msgs/RecognisableObject`

```
uint8   object_type    # 0=car  1=heavy_truck  2=motorbike  3=bicycle
                       # 4=pedestrian  5=pole  6=guard_rail  7=building
                       # 8=traffic_sign  9=traffic_light
float32 detection_range_begin  # m
float32 detection_range_end    # m
```

### `afi920_msgs/ReferenceTarget`

```
float32 radar_cross_section    # dBsm
float32 detection_range_begin  # m
float32 detection_range_end    # m
float32 signal_to_noise_ratio  # dB
```

### `afi920_msgs/E2eStatus` — per-message E2E result

```
std_msgs/Header header        # stamp = stamp of the matching data message
                              # (verification time when data_published is false)
uint8  stream                 # 0=RDI  1=SHI  2=SPI
uint8  result                 # 0=ok  1=crc_mismatch  2=crc_zero (not a pass)  3=not_checked
uint8  crc_format             # 0=unknown  1=CRC-32 (v1.1.0+)  2=CRC-64/XZ (v1.0.x);
                              # on a v1.2.0 scan, from the first passing message
bool   length_ok              # E2E Length equals the payload size
bool   data_id_ok             # E2E Data ID matches the stream
uint32 e2e_counter
uint32 missing_before         # E2E Counter values skipped since the previous message
bool   sensor_restarted       # E2E Counter decreased (v1.0.x: also on TCP reconnect)
bool   data_published         # false: no data message published (e2e_strict drop, parse failure,
                              # scan without header, or nothing enabled for the frame)
uint32 message_counter        # ISO 23150 MessageCounter (0 when not parsed)
uint8  data_qualifier         # ISO 23150 DataQualifier (0 when not parsed)
```

Message loss is counted from `missing_before`. The count also includes cycles the sensor did not send (the cycle after one whose header B could not be sent, and cycles held back in Fault Mode or an access-mode halt), so it is an upper bound of lost messages. On interface v1.2.0 it counts skipped scans; chunks lost inside a scan show up as a `partial` warning instead. MessageCounter is not used for loss detection because interface v1.1.0+ repeats it on resend. On interface v1.0.x the E2E Counter restarts on every TCP connection, so a reconnect sets `sensor_restarted`.

On interface v1.2.0 one `E2eStatus` is published per RDI scan, because all header and chunk messages of a scan share one E2E Counter. `result` is the worst result of those messages (CRC mismatch, then CRC zero, then OK), and `length_ok` / `data_id_ok` are false when any message failed the check. A message dropped by `e2e_strict` is reported on its own with `data_published` false. A scan without any header message is reported with `data_published` false and `message_counter` 0.

---

## 10. PointCloud2 Field Reference

18 fields, all `FLOAT32`, **72-byte point stride**.

| # | Field name | Unit | Description |
|---|-----------|------|-------------|
| 1 | `x` | m | Cartesian X (forward) |
| 2 | `y` | m | Cartesian Y (left) |
| 3 | `z` | m | Cartesian Z (up) |
| 4 | `radial_distance` | m | Range |
| 5 | `azimuth_angle` | rad | Horizontal angle (positive = left) |
| 6 | `elevation_angle` | rad | Vertical angle (positive = up) |
| 7 | `radial_velocity` | m/s | Doppler; negative = approaching |
| 8 | `radar_cross_section` | dBsm | RCS |
| 9 | `signal_noise_ratio` | dB | SNR |
| 10 | `radial_distance_error` | m | Fixed accuracy per Range Mode and sub-cycle |
| 11 | `azimuth_angle_error` | rad | Fixed accuracy 0.001745 |
| 12 | `elevation_angle_error` | rad | Fixed accuracy 0.001745 |
| 13 | `radial_velocity_error` | m/s | Fixed accuracy 0.065 |
| 14 | `existence_probability` | % | 1–100 detection confidence (90–100 real; 98, 97, 95, 93 mark shape-estimation points) |
| 15 | `multi_target_probability` | % | Legacy, always 0 |
| 16 | `detection_id` | — | Per-frame unique ID |
| 17 | `sensor_id` | — | Identifies the source sensor |
| 18 | `intensity` | dBsm | = `radar_cross_section`; for RViz colormap |

---

## 11. Dynamic Parameter Tuning

These parameters can be changed while the node is running — no restart needed. Setting any other parameter at runtime is rejected (`cannot be changed at runtime`).

```bash
# Tighten range window
ros2 param set /afi920/afi920_driver min_range 1.0
ros2 param set /afi920/afi920_driver max_range 100.0

# Filter noisy detections
ros2 param set /afi920/afi920_driver min_snr 5.0
ros2 param set /afi920/afi920_driver min_existence_probability 30.0

# Change TF frame
ros2 param set /afi920/afi920_driver frame_id radar_front

# Toggle E2E strict mode
ros2 param set /afi920/afi920_driver e2e_strict true
```

**List all current values:**
```bash
ros2 param list /afi920/afi920_driver
ros2 param dump /afi920/afi920_driver
```

**Save to YAML and reload** (on Foxy, `ros2 param dump` writes a file by default; add `--print` to write to standard output):
```bash
ros2 param dump /afi920/afi920_driver > afi920_params.yaml
# Then in launch:
ros2 launch afi920_driver afi920.launch.py config_file:=afi920_params.yaml
```

---

## 12. Diagnostic Tools

### `live_data_check.py` — end-to-end sensor verification

Runs a full connectivity + data-flow check against one or more live sensors (interface v1.1.0 and v1.2.0; the SDK suite does not cover v1.0.x).

```bash
cd tools/

# Single sensor (default IP)
python3 live_data_check.py

# Custom IP, 10-second test
python3 live_data_check.py 192.168.10.150 --duration 10.0

# Multiple sensors
python3 live_data_check.py 192.168.10.150 192.168.10.151

# Skip CSII vehicle-input test
python3 live_data_check.py --skip-csii

# Point to SDK manually (order: --sdk-root, AFI920_SDK_ROOT, then afi920_sdk next to the repository;
# --sdk-root is relative to the current directory)
python3 live_data_check.py --sdk-root ../../afi920_sdk

# Local IP for the discovery broadcast, Python used for the SDK test
python3 live_data_check.py --bind 192.168.10.100 --python python3
```

Tests performed: Discovery (UDP 30520) · Config API · RDI/SHII/SPI streams · SOME/IP + E2E integrity · CSII vehicle input (UDP 30501)

### `rdi_diag.py` — byte-level RDI protocol debugger

Captures raw frames and validates SOME/IP framing, E2E CRC (CRC-32 or CRC-64/XZ, same rule as the driver), and RDI header/detection fields.

```bash
# UDP (default; receives on --port from any sender, --sensor-ip is not used)
python3 rdi_diag.py --port 30509 --max-frames 5

# TCP
python3 rdi_diag.py --sensor-ip 192.168.10.150 --tcp --max-frames 5
```

Output includes: SOME/IP header, E2E CRC result and matched format, RDI header fields, first 10 detections with NaN diagnostics.

---

## 13. Port & Protocol Reference

| Port | Proto | Direction | Purpose |
|------|-------|-----------|---------|
| 30500 | TCP | ↔ | Config API (sensor IP, stream destination) |
| 30509 | TCP/UDP | ← | RDI — point cloud (Event ID `0x8002`; interface v1.2.0 adds chunk Events `0x8100`–`0x814B`) |
| 30510 | TCP/UDP | ← | SHII — health info (Event ID `0x8003`) |
| 30511 | TCP/UDP | ← | SPI — performance info (Event ID `0x8004`) |
| 30501 | UDP | → | CSII vehicle input (`live_data_check.py` only; UDP on v1.1.0+) |
| 30520 | UDP | ↔ | Discovery (`live_data_check.py` only) |

**Receive buffer sizes** (TCP: application buffer and kernel `SO_RCVBUF`; UDP: application buffer for one datagram, and every UDP socket also requests a 16 MB kernel buffer):
| Stream | TCP | UDP |
|--------|-----|-----|
| RDI | 512 KB | 400 KB |
| SHII | 4 KB | 4 KB |
| SPI | 4 KB | 4 KB |

**Protocol stack:**
```
ROS 2 topic (sensor_msgs / afi920_msgs)
    ↑
ISO 23150 payload (binary, little-endian)
    ↑
E2E header (20 B · big-endian · CRC / Length / Counter / Data ID)
    ↑
SOME/IP header (16 B · big-endian; + 4 B TP header on segmented single-message RDI)
    ↑
TCP / UDP
```

**Sensor interface compatibility:** the format is detected from received frames; no parameter selects it.

| Item | Interface v1.0.x | Interface v1.1.0 | Interface v1.2.0 |
|------|------------------|------------------|------------------|
| Sensor firmware | v3.0.0 – v3.1.0 | v3.1.1 | v3.1.2 |
| RDI detections per scan | Capability 4096 or 8192 (driver accepts up to 8192) | 2048 | 2048 (EntitiesPerChunk 27, up to 76 chunks) |
| E2E CRC field (bytes 0–7) | CRC-64/XZ over header bytes 8–19 and the payload | bytes 0–3 = 0, bytes 4–7 = IEEE 802.3 CRC-32 over the payload | Same as v1.1.0 |
| RDI framing | One Event `0x8002` message per scan, SOME/IP-TP over UDP | Same as v1.0.x | SOME/IP Interface Version `0x02`: header A (`0x8002`), chunk k (`0x8100`+k, k = 0–75), header B (`0x8002`); no SOME/IP-TP |
| RDI E2E Data ID | `0x60008002` | `0x60008002` | `0x60008002` for both headers, `0x60008100`+k for chunk k |
| Config API frame | SOME/IP header + JSON | SOME/IP header + E2E header + JSON | Same as v1.1.0 |
| Stream destination at startup | Applied with `SetNetworkConfig` once at node start (a sensor powered on later needs a node restart) | Read with `GetNetworkConfig`; differences are logged and the sensor is not changed | Same as v1.1.0 |
| Field meanings that differ | `existence_probability` markers 100/60/40; DataQualifier 0/5/6; SHII operation mode `150` can occur; 1 input signal status; E2E Counter per stream from 1 on each TCP connection | `existence_probability` = probability × 100 (augmented 95/90); DataQualifier 0/1/6; 4 input signal statuses | See §9 |
| Message loss detection | E2E Counter | E2E Counter | E2E Counter (shared by all messages of a scan) |
| Frames excluded from `PointCloud2` | DataQualifier 1 and 6 (5 = test mode is published) | DataQualifier 1 and 6 | DataQualifier 1 and 6 (from the header message) |

On interface v1.1.0+, changing the stream protocol, port, or destination requires diagnostic mode, `SetNetworkConfig`, and a sensor restart. The driver does not perform these steps.

**Interface v1.2.0 RDI scan assembly** (Interface Overview §3.2.3):

| Rule | Driver behavior |
|------|-----------------|
| Header body | Detection Header (36 B) + List Layout (EntitiesPerChunk u16, ChunkCount u16) |
| Chunk body | Chunk Header (MessageCounter u32, FirstEntityIndex u16, EntityCount u16) + EntityCount × 51 B detections |
| Scan key | E2E Counter. A message with another E2E Counter closes the open scan |
| Header A / header B | The first `0x8002` of a scan is header A, the second header B. A header copy that passed the E2E check is preferred |
| Scan close | One header and ChunkCount chunks received, header B received, another E2E Counter received, or 100 ms after the first message (the RDI receive timeout of 20 ms keeps this on TCP and UDP; while a TCP connection is being re-established the close waits for the reconnect) |
| Late message | A message of one of the last two closed scans is ignored |
| Invalid scan | A header with EntitiesPerChunk 0, ChunkCount above 76 or a body shorter than 40 B is not accepted; the scan is handled as a scan without header. Bytes after the first 40 B of a header body are ignored |
| Chunk check | A chunk is rejected and logged when its size does not match EntityCount, EntityCount exceeds EntitiesPerChunk, its MessageCounter differs from the header, its index or slot (FirstEntityIndex / EntitiesPerChunk) is not below ChunkCount, or FirstEntityIndex is not a multiple of EntitiesPerChunk. When two chunks claim the same slot, the one that passed the E2E check is kept |
| Partial scan | Published on `PointCloud2` and `RDI` with the detections that arrived, in FirstEntityIndex order; a `partial` warning is logged. The sensor also leaves out chunks it cannot send within its 94 ms send window, on TCP and UDP |
| Scan without header | Not published on `PointCloud2` or `RDI`; reported on `E2E` with `data_published` false |

**Multi-sensor:** over TCP, each sensor is distinguished by its **IP** (and ROS namespace),
and all sensors can use the same default ports (30509/30510/30511). Over UDP, each sensor
must use its own host ports; see [§8 Ports](#ports).
`multi_sensor.launch.py` does not assign per-sensor port offsets.

---

## 14. Troubleshooting Checklist

| Symptom | Check |
|---------|-------|
| `ping` fails | Host IP in `192.168.10.x`? Ethernet connected? |
| Node starts but no topics | `sensor_ip` correct? Port 30509 reachable? Over TCP the node keeps retrying the connection with backoff from 1 s up to 30 s and logs each failed attempt |
| Topics exist but no messages | `ros2 topic hz` shows 0 Hz → check firewall rules for ports 30509–30511 |
| `RDI` detections array always empty | Object in FoV? Check `min_range`/`max_range` and `min_snr` |
| E2E CRC mismatch warnings in node log | Frames are still published. Check `ros2 topic echo /afi920/E2E` (`result`, `crc_format`); persistent mismatches over UDP → switch TCP (`transport_mode:=tcp`) |
| `Sensor stream config differs from driver parameters` warning | Interface v1.1.0+ sensor uses another stream protocol, port, or (UDP) destination IP than `host_ip` → match `transport_mode` and port parameters to the sensor, or reconfigure and restart the sensor |
| No cloud in RViz with Fixed Frame `base_link` | No TF to the sensor frame → keep `publish_tf` and `publish_performance` at `true`, or publish the transform yourself |
| Sensor frame jumps between two poses | Driver and URDF/another node publish the same transform → set `publish_tf:=false` |
| RDI missing, SHI/SPI fine | A hardware defect or calibration error condition (`defect_reason` 2 or 5) holds → the sensor holds RDI back while it holds; `operation_modes[0]` then shows 100 (Fault Mode), or 20 (Diagnostic) / 200 (Initialising) when those take priority. Mode 20 by itself only means that the Config API access mode is Diagnostic and does not stop RDI. Check `defect_reason`. A calibration CRC failure lasts the whole boot and clears only after the calibration record is written again (0x00A6) and the sensor restarts |
| `connect to ... failed -- retrying` with the default ports | Stream ports are set per sensor and can differ from the defaults 30509–30511 → read `detection_port`, `health_port`, `performance_port` with SDK `GetNetworkConfig` or Discovery and set `data_port`, `shi_port`, `spi_port` |
| `connected` followed at once by `connection ... lost -- reconnecting`, repeating | Another client holds the stream port. The sensor accepts one TCP client per stream channel and resets further connections → stop the other client (another driver instance, tool or host) |
| `RDI scan ... partial` warning | Interface v1.2.0 chunk messages missing → the scan is published with the detections that arrived. Over UDP the cause can be network loss (try `transport_mode:=tcp`); on both transports the sensor leaves out chunks it cannot send within its send window |
| Multi-sensor: topics missing | Verify each `<pos>_ip` is reachable and namespaces differ. Nodes without a sensor keep retrying; launch only the sensors you have with `afi920.launch.py` |
| `colcon build` fails | Build `afi920_msgs` first; check `$ROS_DISTRO` is sourced |
| `host_ip:=auto` picks wrong NIC | Set `host_ip:=192.168.10.100` explicitly |
| High latency / packet loss | TCP preferred. Jumbo frames are not needed: every UDP message fits in 1,472 B (v1.0.x/v1.1.0 RDI uses SOME/IP-TP segments) |

---

## Quick Reference Card

```
BUILD   → colcon build --packages-select afi920_msgs afi920_driver
LAUNCH  → ros2 launch afi920_driver afi920.launch.py sensor_ip:=<IP>
TOPICS  → /afi920/PointCloud2  /afi920/RDI  /afi920/SHI  /afi920/SPI  /afi920/E2E
TUNE    → ros2 param set /afi920/afi920_driver min_range 1.0   (live)
MULTI   → ros2 launch afi920_driver multi_sensor.launch.py front_ip:=... rear_ip:=...
VERIFY  → python3 tools/live_data_check.py [IP] [--duration N]
DEBUG   → python3 tools/rdi_diag.py [--port 30509] | --tcp --sensor-ip <IP>
```

**Key files to read next:**
- [src/afi920_driver/launch/afi920.launch.py](src/afi920_driver/launch/afi920.launch.py)
- [src/afi920_driver/launch/multi_sensor.launch.py](src/afi920_driver/launch/multi_sensor.launch.py)
- [src/afi920_msgs/msg/Detection.msg](src/afi920_msgs/msg/Detection.msg)
- [tools/live_data_check.py](tools/live_data_check.py)
