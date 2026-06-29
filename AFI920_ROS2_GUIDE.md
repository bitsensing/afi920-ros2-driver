# AFI920 ROS 2 Driver — AI Integration Guide

> **Who this is for:** AI agents, LLM-driven automation, and developers who need the fastest path to integrating the bitsensing AFI920 4D imaging radar into a ROS 2 system.  
> **Driver version:** 2.1.0 · **Supported ROS 2:** Foxy (Ubuntu 20.04), Humble (Ubuntu 22.04) · **Arch:** x86_64, aarch64  
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

The AFI920 ROS 2 driver wraps the AFI920 SDK into a composable ROS 2 node. It connects to the radar over Ethernet and publishes four data topics at ~10 Hz:

| What you get | Topic | Type |
|---|---|---|
| Standard point cloud (RViz-ready) | `~/PointCloud2` | `sensor_msgs/PointCloud2` |
| Full ISO 23150 detection data | `~/RDI` | `afi920_msgs/DetectionArray` |
| Sensor health & fault status | `~/SHI` | `afi920_msgs/HealthInfo` |
| Sensor performance & mounting pose | `~/SPI` | `afi920_msgs/SensorPerformance` |

In addition, it publishes a `~/detection_count` RViz overlay marker and broadcasts the sensor mounting pose (from SPI) as a TF transform `parent_frame_id → frame_id`. See [§6](#6-published-topics).

All data topics use `SensorDataQoS`. The driver supports TCP (default) and UDP transport, AUTOSAR E2E CRC validation, SOME/IP-TP segment reassembly, TF broadcast of the sensor pose, and runtime filter adjustment via `ros2 param set`.

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
│   │       └── ReferenceTarget.msg ← reference target characteristics
│   └── afi920_driver/
│       ├── src/afi920_node.cpp     ← main composable node
│       └── launch/
│           ├── afi920.launch.py        ← single sensor
│           └── multi_sensor.launch.py  ← up to 4 sensors
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
source /opt/ros/$ROS_DISTRO/setup.bash   # foxy or humble

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

`detection_count` is a `TEXT_VIEW_FACING` marker (`Pts: <n>`) for RViz overlay; it is always published alongside `PointCloud2`.

**TF broadcast:** when `publish_tf` is `true` (default) and `publish_performance` is `true`, the node broadcasts the sensor mounting pose received in each SPI frame to `/tf` (`tf2_msgs/TFMessage`) as the transform `parent_frame_id → frame_id` (default `base_link → afi920`). This lets RViz/consumers place each sensor's cloud at its calibrated position when the fixed frame is set to `parent_frame_id`. If `publish_performance` is `false`, no SPI pose is available and TF is not broadcast (a warning is logged).

**Coordinate convention:** sensor-frame FLU (X forward, Y left, Z up) — REP-103 compliant.

---

## 7. All Node Parameters

### Connection (not dynamic — set at launch)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `sensor_ip` | string | `192.168.10.150` | Radar IP address |
| `host_ip` | string | `auto` | Local bind IP (`auto` = detected from route to sensor) |
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
| `min_range` | double | `0.0` m | Drop detections closer than this |
| `max_range` | double | `300.0` m | Drop detections farther than this |
| `min_snr` | double | `0.0` dB | Drop detections below this SNR |
| `min_existence_probability` | double | `0.0` % | Drop detections below this confidence |

### E2E integrity (dynamic)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `validate_e2e` | bool | `true` | Verify AUTOSAR E2E CRC-64/XZ on every frame |
| `e2e_strict` | bool | `false` | If `true`, drop frames that fail E2E check |

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

Up to 4 sensors simultaneously, each in its own namespace.

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
| `validate_e2e` | `true` | Validate AUTOSAR E2E CRC on data streams |
| `e2e_strict` | `false` | Drop frames on E2E mismatch |
| `parent_frame` | `base_link` | Common parent TF frame; each pose is broadcast as `parent_frame → <pos>_frame` |
| `publish_tf` | `true` | Broadcast each sensor's mounting pose (from SPI) as TF |
| `config_file` | `""` | Path to a YAML config file (overrides individual params; e.g. `config/multi_sensor.yaml`) |

`publish_detections`, `publish_health`, and `publish_performance` are hard-wired to `true` for every sensor in this launch file (not exposed as arguments). To toggle them, use a `config_file`.

### Ports

`multi_sensor.launch.py` does **not** expose or set per-sensor ports. Each sensor is
reached at its own IP on the **same default ports** (30509/30510/30511), and each node
runs in its own namespace, so there is no conflict (over TCP the host side uses ephemeral
ports). To change a port, reconfigure it on the radar and pass a `config_file` with the
matching `data_port`/`shi_port`/`spi_port`.

---

## 9. Custom Message Definitions

### `afi920_msgs/DetectionArray` — RDI frame

```
std_msgs/Header header
uint8  sensor_id
uint32 message_counter
uint64 timestamp_measurement_ns      # sensor PTP timestamp (ns)
uint8  data_qualifier                # 0=normal 1=not_available 2=reduced_coverage 3=reduced_perf

float32 velocity_ambiguity_begin     # m/s  (default: -88.89)
float32 velocity_ambiguity_end       # m/s  (default:  55.56)
float32 range_ambiguity_begin        # m    (default:   0.0)
float32 range_ambiguity_end          # m    (default: 300.0)
float32 azimuth_ambiguity_begin      # rad  (default: -1.0472,  ≈ -60°)
float32 azimuth_ambiguity_end        # rad  (default:  1.0472,  ≈ +60°)
float32 elevation_ambiguity_begin    # rad  (default: -0.2618,  ≈ -15°)
float32 elevation_ambiguity_end      # rad  (default:  0.2618,  ≈ +15°)

uint16 max_detections                # sensor capability (typically 2048)
uint8  recognised_detections_status  # 0=ok  1=performance_limit_reached
afi920_msgs/Detection[] detections
```

### `afi920_msgs/Detection` — single detection

```
# Identification
uint16  detection_id
uint16  object_id_reference    # reserved, 0xFFFF = no tracking
uint8   existence_probability  # 0–100 %
float32 timestamp_difference   # s offset from header timestamp

# Position (spherical, as received from sensor)
float32 radial_distance        # m (0–300)
float32 azimuth                # rad (±60°)
float32 elevation              # rad (±15°)
float32 radial_distance_error  # m
float32 azimuth_error          # rad
float32 elevation_error        # rad

# Position (Cartesian, computed from spherical; X=forward, Y=left, Z=up)
float32 x                      # m
float32 y                      # m
float32 z                      # m

# Dynamics
float32 radial_velocity        # m/s (negative = approaching)
float32 radial_velocity_error  # m/s

# Signal quality
float32 rcs                    # dBm² (radar cross section)
float32 rcs_error              # dBm²
float32 snr                    # dB
float32 snr_error              # dB

# Probabilities
uint8   multi_target_probability        # 0–100 %
uint16  ambiguity_grouping_id
uint8   detection_ambiguity_probability # reserved
uint8   free_space_probability          # reserved

# Classification (up to 8 slots)
uint8   num_classifications
uint8[8] classification_type   # 0=none 1=noise 2=obstacle 3=under 4=over 5=nearest 6=strongest
uint8[8] classification_confidence  # 0–100 per slot

# Debug fields (user-defined extension to ISO 23150)
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

uint8  data_qualifier

# Operation mode (up to 11 entries)
uint8   num_operation_modes
uint8[11] operation_modes
# Values: 0=normal_dual_range  1=normal_long_range  2=normal_middle_range
#         3=normal_short_range 4=normal_ultra_long_range  5=normal_ultra_short_range
#         10=degradation  50=evaluation  100=calibration
#         200=initialising  201=test

# Defect status
uint8 defect_recognised        # 0=fully_functional  1=not_fully  2=out_of_order
uint8 defect_reason            # 0=none  1=memory  2=hardware  3=thermal  4=communication
                               # 5=calibration  6=configuration  7=mechanical
                               # 8=software  9=computing_power  10=time_sync  11=external

# Supply & temperature
uint8 supply_voltage_status    # 0=low  1=pre_low  2=ok  3=pre_high  4=high
uint8 temperature_status       # 0=under  1=pre_under  2=ok  3=pre_over  4=over

# Input signal status (up to 4 entries)
uint8   num_input_signal_statuses
uint8[4] input_signal_types
uint8[4] input_signal_statuses  # 0=valid 1=implausible 2=missing 3=out_of_range 4=timeout

# Time sync
uint8   time_sync_status       # 0=ok  1=out_of_limits  2=timeout  3=not_synchronized
float32 time_sync_offset       # legacy, always 0.0

# Calibration (parallel arrays, up to 3 components)
uint8   num_calibration_components
uint8[3] calibration_components # 0=intrinsic  1=extrinsic_horizontal  2=extrinsic_vertical
uint8[3] calibration_statuses   # 0=calibrated  1=not_calibrated  2=degraded
uint8[3] calibration_states     # 0=initial_done 1=initial_not_done 2=initial_failed
                                # 3=recal_intrinsic 4=recal_extrinsic 5=recal_full
```

### `afi920_msgs/SensorPerformance` — SPI frame

```
std_msgs/Header header
uint8  sensor_id
uint32 message_counter
uint64 timestamp_measurement_ns
uint8  data_qualifier
uint8  vehicle_coordinate_system     # 0=rear_axle  1=road_level

afi920_msgs/SensorPose sensor_pose
afi920_msgs/FovSegment[]        fov_segments
afi920_msgs/RecognisableObject[] recognisable_objects
afi920_msgs/ReferenceTarget[]    reference_targets
```

### `afi920_msgs/SensorPose`

```
float32 origin_x / origin_y / origin_z          # mounting position (m)
float32 origin_error_x / origin_error_y / origin_error_z
float32 yaw / pitch / roll                       # mounting orientation (rad)
float32 yaw_error / pitch_error / roll_error
```

### `afi920_msgs/FovSegment`

```
float32 azimuth_begin / azimuth_end     # rad
float32 elevation_begin / elevation_end # rad
uint8   blockage_status                 # 0=none  1=partial  2=full  3=unknown
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
float32 radar_cross_section    # dBm²
float32 detection_range_begin  # m
float32 detection_range_end    # m
float32 signal_to_noise_ratio  # dB
```

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
| 10 | `radial_distance_error` | m | |
| 11 | `azimuth_angle_error` | rad | |
| 12 | `elevation_angle_error` | rad | |
| 13 | `radial_velocity_error` | m/s | |
| 14 | `existence_probability` | % | 0–100 detection confidence |
| 15 | `multi_target_probability` | % | 0–100 |
| 16 | `detection_id` | — | Per-frame unique ID |
| 17 | `sensor_id` | — | Identifies the source sensor |
| 18 | `intensity` | dBsm | = `radar_cross_section`; for RViz colormap |

---

## 11. Dynamic Parameter Tuning

These parameters can be changed while the node is running — no restart needed.

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

**Save to YAML and reload:**
```bash
ros2 param dump /afi920/afi920_driver > afi920_params.yaml
# Then in launch:
ros2 launch afi920_driver afi920.launch.py config_file:=afi920_params.yaml
```

---

## 12. Diagnostic Tools

### `live_data_check.py` — end-to-end sensor verification

Runs a full connectivity + data-flow check against one or more live sensors.

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

# Point to SDK manually
python3 live_data_check.py --sdk-root /path/to/afi920-sdk
```

Tests performed: Discovery · Config API · RDI/SHII/SPI streams · SOME/IP + E2E integrity · CSII vehicle input

### `rdi_diag.py` — byte-level RDI protocol debugger

Captures raw frames and validates SOME/IP framing, E2E CRC, and RDI header/detection fields.

```bash
# UDP (default)
python3 rdi_diag.py --sensor-ip 192.168.10.150 --port 30509 --max-frames 5

# TCP
python3 rdi_diag.py --sensor-ip 192.168.10.150 --tcp --max-frames 5
```

Output includes: SOME/IP header, E2E CRC result, RDI header fields, first 10 detections with NaN diagnostics.

---

## 13. Port & Protocol Reference

| Port | Proto | Direction | Purpose |
|------|-------|-----------|---------|
| 30500 | TCP | ↔ | Config API (sensor IP, stream destination) |
| 30509 | TCP/UDP | ← | RDI — point cloud (Event ID `0x8002`) |
| 30510 | TCP/UDP | ← | SHII — health info (Event ID `0x8003`) |
| 30511 | TCP/UDP | ← | SPI — performance info (Event ID `0x8004`) |

**Internal buffer sizes:**
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
AUTOSAR E2E Profile 7 header (20 B · CRC-64/XZ)
    ↑
SOME/IP-TP header (16 B + 4 B TP · big-endian)
    ↑
TCP / UDP
```

**Multi-sensor:** each sensor is distinguished by its **IP** (and ROS namespace),
not by port. All sensors use the same default ports (30509/30510/30511);
`multi_sensor.launch.py` does not assign per-sensor port offsets.

---

## 14. Troubleshooting Checklist

| Symptom | Check |
|---------|-------|
| `ping` fails | Host IP in `192.168.10.x`? Ethernet connected? |
| Node starts but no topics | `sensor_ip` correct? Port 30509 reachable? |
| Topics exist but no messages | `ros2 topic hz` shows 0 Hz → check firewall rules for ports 30509–30511 |
| `RDI` detections array always empty | Object in FoV? Check `min_range`/`max_range` and `min_snr` |
| E2E errors in node log | Normal during startup; persistent errors → switch TCP (`transport_mode:=tcp`) |
| RDI missing, SHI/SPI fine | RDI buffer overrun → force TCP (512 KB buffer vs 400 KB UDP) |
| Multi-sensor: topics missing | Verify each `<pos>_ip` is reachable and namespaces differ |
| `colcon build` fails | Build `afi920_msgs` first; check `$ROS_DISTRO` is sourced |
| `host_ip:=auto` picks wrong NIC | Set `host_ip:=192.168.10.100` explicitly |
| High latency / packet loss | TCP preferred; check network switch for jumbo frames if UDP used |

---

## Quick Reference Card

```
BUILD   → colcon build --packages-select afi920_msgs afi920_driver
LAUNCH  → ros2 launch afi920_driver afi920.launch.py sensor_ip:=<IP>
TOPICS  → /afi920/PointCloud2  /afi920/RDI  /afi920/SHI  /afi920/SPI
TUNE    → ros2 param set /afi920/afi920_driver min_range 1.0   (live)
MULTI   → ros2 launch afi920_driver multi_sensor.launch.py front_ip:=... rear_ip:=...
VERIFY  → python3 tools/live_data_check.py [IP] [--duration N]
DEBUG   → python3 tools/rdi_diag.py --sensor-ip <IP> [--tcp]
```

**Key files to read next:**
- [src/afi920_driver/launch/afi920.launch.py](src/afi920_driver/launch/afi920.launch.py)
- [src/afi920_driver/launch/multi_sensor.launch.py](src/afi920_driver/launch/multi_sensor.launch.py)
- [src/afi920_msgs/msg/Detection.msg](src/afi920_msgs/msg/Detection.msg)
- [tools/live_data_check.py](tools/live_data_check.py)
