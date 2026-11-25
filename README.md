# AFI920 ROS 2 Driver

![ROS2](https://img.shields.io/badge/ROS2-Foxy%20%7C%20Humble%20%7C%20Jazzy-blue)
![Platform](https://img.shields.io/badge/platform-Ubuntu%2020.04%20%7C%2022.04-blue)
![License](https://img.shields.io/badge/license-BSD--3--Clause-green)

bitsensing **AFI920 4D Imaging Radar** ROS 2 driver package.
It publishes AFI920 sensor data as ROS 2 topics for integration with customer ROS 2 applications.

> **bitsensing** — [bitsensing.com](https://bitsensing.com)
>
> For the Korean version, see [README_ko.md](README_ko.md).

---

## Scope

- This repository provides the **public ROS 2 driver for AFI920 customer integration**.
- **Product manuals, interface specifications, and detailed installation/operation guides** are provided separately.
- This README is intended as a quick reference for **build, launch, key topics/parameters, and the basic visualization flow**.
- [AFI920_ROS2_GUIDE.md](AFI920_ROS2_GUIDE.md) is the full integration reference with every parameter, message definition, and protocol detail.

---

## Table of Contents

- [Scope](#scope)
- [Supported Platforms](#supported-platforms)
- [Quick Start](#quick-start)
- [Live Radar Verification](#live-radar-verification)
- [Features](#features)
- [Prerequisites](#prerequisites)
- [Topics](#topics)
- [PointCloud2 Fields](#pointcloud2-fields)
- [Coordinate Convention](#coordinate-convention)
- [Parameters](#parameters)
- [Multi-Sensor](#multi-sensor)
- [Architecture](#architecture)
- [RViz2 Visualization](#rviz2-visualization)
- [Isaac ROS Integration](#isaac-ros-integration)
- [Troubleshooting](#troubleshooting)
- [Security Notice](#security-notice)
- [Related Projects](#related-projects)
- [Changelog](#changelog)
- [License](#license)
- [Support](#support)

---

## Supported Platforms

| OS | ROS 2 Distribution | Architecture |
|----|-------------------|-------------|
| Ubuntu 20.04 | Foxy Fitzroy | x86_64, aarch64 |
| Ubuntu 22.04 | Humble Hawksbill | x86_64, aarch64 |
| Ubuntu 24.04 | Jazzy Jalisco | aarch64 |

---

## Quick Start

### 1. Clone and Build

```bash
source /opt/ros/$ROS_DISTRO/setup.bash

cd ~/ros2_ws/src
git clone https://github.com/bitsensing/afi920-ros2-driver.git

cd ~/ros2_ws

# Install workspace dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the driver packages
colcon build --packages-select afi920_msgs afi920_driver
source install/setup.bash
```

### 2. Launch

```bash
ros2 launch afi920_driver afi920.launch.py sensor_ip:=192.168.10.150
```

The above command launches a single sensor with the default namespace `afi920` and the default sensor IP `192.168.10.150`.

> **Tip:** Use the `config_file` argument if you want to manage parameters from a file. When `config_file` is set, the node takes its parameters only from that file, and other launch arguments such as `sensor_ip` are not applied. Use `/**:` as the top-level key (as in `config/config.yaml`) so the file applies in any namespace.
>
> ```bash
> ros2 launch afi920_driver afi920.launch.py config_file:=/path/to/custom_params.yaml
> ```

### 3. Check Data

```bash
# List topics
ros2 topic list

# Check the PointCloud2 publish rate
ros2 topic hz /afi920/PointCloud2

# Check key data streams
ros2 topic echo /afi920/RDI --once
ros2 topic echo /afi920/SHI --once
ros2 topic echo /afi920/SPI --once
```

---

## Live Radar Verification

Use the live data check before or after launching ROS when you need to verify
the radar contract directly against a real sensor:

```bash
python3 tools/live_data_check.py 192.168.10.150 --duration 8
```

The tool locates `afi920_sdk` from `--sdk-root`, `AFI920_SDK_ROOT`, or a sibling
`../afi920_sdk` directory, in that order, auto-detects the local bind IP for the target radar,
then runs the SDK live conformance suite. It checks Discovery, Config API,
RDI/SHII/SPI data streams, SOME/IP/E2E integrity, and CSII vehicle-input
transmission. The SDK suite targets interface v1.1.0 and v1.2.0 sensors.

```bash
python3 tools/live_data_check.py 192.168.10.150 --sdk-root /path/to/afi920_sdk
```

---

## Features

- **PointCloud2 Publish**: Publishes AFI920 detections as the standard ROS message `sensor_msgs/PointCloud2`
- **Full RDI Publish**: Provides all ISO 23150 RDI fields through the `DetectionArray` message (detections pass the same filters as `PointCloud2`)
- **Health and Performance Topics**: Publishes sensor health and performance information on the `SHI` and `SPI` topics
- **Multi-Sensor Ready**: Supports independent multi-sensor operation with namespace-based isolation
- **Runtime Filtering**: Supports runtime adjustment of range, SNR, and existence probability filters
- **RViz2 Friendly**: Enables immediate visualization in RViz2 using the default PointCloud2 topic

---

## Prerequisites

### ROS 2 Dependencies

```bash
sudo apt install ros-$ROS_DISTRO-rclcpp \
                  ros-$ROS_DISTRO-rclcpp-components \
                  ros-$ROS_DISTRO-sensor-msgs \
                  ros-$ROS_DISTRO-std-msgs \
                  ros-$ROS_DISTRO-visualization-msgs \
                  ros-$ROS_DISTRO-geometry-msgs \
                  ros-$ROS_DISTRO-tf2 \
                  ros-$ROS_DISTRO-tf2-ros
```

### Network Setup

The AFI920 sensor communicates over Ethernet, and the host PC must be on the same subnet.

| Item | Value |
|------|-------|
| Default sensor IP | `192.168.10.150` |
| Host IP | `192.168.10.x` (for example, `192.168.10.100`) |
| Subnet mask | `255.255.255.0` |

Example:

```bash
sudo ip addr add 192.168.10.100/24 dev eth0
```

### Default Data Stream Ports

| Stream | Port | Event ID | Description |
|--------|------|----------|-------------|
| RDI | 30509 | 0x8002 (v1.2.0 chunks 0x8100-0x814B) | Detection / point cloud data |
| SHI  | 30510 | 0x8003 | Sensor status and health information |
| SPI | 30511 | 0x8004 | Sensor performance and pose information |

> **Note:** If your environment uses a firewall or network security policy, allow the ports in use.

---

## Topics

The default namespace is `afi920`, and the table below lists the relative topic names within that namespace.

Example: with the default namespace, `PointCloud2` is published as `/afi920/PointCloud2`.

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `PointCloud2` | `sensor_msgs/PointCloud2` | 10 Hz | Publishes AFI920 detections as `sensor_msgs/PointCloud2` |
| `PointCloud2XYZI` | `sensor_msgs/PointCloud2` | 10 Hz | `x`, `y`, `z`, `intensity` only (16 B/point) for Isaac ROS. Published only when `publish_pointcloud_xyzi` is `true`. See [Isaac ROS Integration](#isaac-ros-integration) |
| `RDI` | `afi920_msgs/DetectionArray` | 10 Hz | Full detection data |
| `SHI` | `afi920_msgs/HealthInfo` | 10 Hz | Sensor status and health information |
| `SPI` | `afi920_msgs/SensorPerformance` | 10 Hz | Sensor performance and pose information |
| `detection_count` | `visualization_msgs/Marker` | 10 Hz | RViz text overlay with the point count, published with `PointCloud2` |
| `E2E` | `afi920_msgs/E2eStatus` | 30 Hz | E2E verification result of each RDI/SHI/SPI message (one per RDI scan on interface v1.2.0). `header.stamp` equals the stamp of the matching data message |

All topics are published with `SensorDataQoS` (best effort). A subscriber with the default reliable QoS receives nothing; subscribe with `rclcpp::SensorDataQoS()` (C++) or `qos_profile_sensor_data` (Python).

When `publish_tf` is `true` (default), the driver also broadcasts the sensor mounting pose from each SPI message on `/tf` as the transform `parent_frame_id → frame_id` (default `base_link → afi920`). The pose is not broadcast when `publish_performance` is `false`.

## PointCloud2 Fields

PointCloud2 includes 18 fields.

| # | Field | Unit | Description |
|---|-------|------|-------------|
| 1-3 | x, y, z | m | Cartesian position |
| 4-6 | radial_distance, azimuth_angle, elevation_angle | m, rad | Spherical position |
| 7 | radial_velocity | m/s | Doppler radial velocity |
| 8 | radar_cross_section | dBsm | RCS |
| 9 | signal_noise_ratio | dB | SNR |
| 10-13 | radial_distance_error, azimuth_angle_error, elevation_angle_error, radial_velocity_error | m, rad, m/s | Fixed accuracy values of the sensor |
| 14 | existence_probability | 1-100 | Detection confidence |
| 15 | multi_target_probability | - | Legacy, always 0 |
| 16 | detection_id | - | Detection identifier |
| 17 | sensor_id | - | Sensor identifier |
| 18 | intensity | dBsm | RViz-compatible intensity value |

The point stride is 72 bytes, compatible with standard ROS 2 processing pipelines at the default 10 Hz rate.

## Coordinate Convention

The Cartesian coordinates `(x, y, z)` in PointCloud2 use the sensor-frame **FLU** convention: `(X forward, Y left, Z up)`.
This follows the ROS standard coordinate convention defined in [REP-103](https://www.ros.org/reps/rep-0103.html).

- Azimuth 0 points forward.
- Positive azimuth is `+Y` (left).
- Positive elevation is `+Z` (up).
- If your stack uses a different vehicle coordinate convention, apply a TF rotation.

---

## Parameters

### Core Parameters

| Parameter | Type | Default | Dynamic | Description |
|-----------|------|---------|---------|-------------|
| `sensor_ip` | string | `192.168.10.150` | No | Sensor IP address |
| `frame_id` | string | `afi920` | Yes | TF frame ID applied to published messages |
| `parent_frame_id` | string | `base_link` | No | Parent TF frame of the broadcast sensor pose |
| `publish_tf` | bool | `true` | No | Broadcast the SPI mounting pose as TF `parent_frame_id → frame_id`. Requires `publish_performance` |
| `min_range` | double | `0.0` | Yes | Minimum range filter (m), applied to `PointCloud2` and `RDI` |
| `max_range` | double | `300.0` | Yes | Maximum range filter (m), applied to `PointCloud2` and `RDI` |
| `min_snr` | double | `0.0` | Yes | Minimum SNR filter (dB), applied to `PointCloud2` and `RDI` |
| `min_existence_probability` | double | `0.0` | Yes | Minimum existence probability filter (0-100), applied to `PointCloud2` and `RDI` |
| `publish_detections` | bool | `true` | No | Enable publishing on the `RDI` topic |
| `publish_health` | bool | `true` | No | Enable publishing on the `SHI` topic |
| `publish_performance` | bool | `true` | No | Enable publishing on the `SPI` topic |
| `publish_pointcloud_xyzi` | bool | `false` | No | Enable publishing on the `PointCloud2XYZI` topic |
| `xyzi_pad_multiple` | int | `16` | No | Pad `PointCloud2XYZI` to a multiple of this point count with `z = +inf` points (`0` or `1`: no padding) |

### Network and Configuration Parameters

| Parameter | Type | Default | Dynamic | Description |
|-----------|------|---------|---------|-------------|
| `data_port` | int | `30509` | No | RDI data port |
| `shi_port` | int | `30510` | No | SHI data port |
| `spi_port` | int | `30511` | No | SPI data port |
| `config_port` | int | `30500` | No | Sensor configuration port |
| `host_ip` | string | `auto` | No | Host IP for the stream destination: written with `SetNetworkConfig` on interface v1.0.x, compared with the sensor's UDP destination on v1.1.0+ (`auto` is supported) |
| `transport_mode` | string | `tcp` | No | Data stream transport (`tcp` or `udp`) |
| `validate_e2e` | bool | `true` | Yes | Validate the E2E CRC on RDI/SHI/SPI. Results are published on `E2E`, and frames are not dropped on CRC mismatch |
| `e2e_strict` | bool | `false` | Yes | Drop data stream frames on E2E Length or Data ID mismatch |

### Sensor Interface Compatibility

The driver supports AFI920 interface v1.0.x, v1.1.0 and v1.2.0 without extra parameters. The format is detected from the frames the sensor sends.

| Item | Interface v1.0.x | Interface v1.1.0 | Interface v1.2.0 |
|------|------------------|------------------|------------------|
| Sensor firmware | v3.0.0 – v3.1.0 | v3.1.1 | v3.1.2 |
| E2E CRC on RDI/SHI/SPI | CRC-64/XZ | IEEE 802.3 CRC-32 over the payload | IEEE 802.3 CRC-32 over the payload |
| RDI framing | One message per scan (SOME/IP-TP over UDP) | One message per scan (SOME/IP-TP over UDP) | Header A, chunk messages (Event `0x8100`+k) and header B per scan, each with its own E2E header and no SOME/IP-TP |
| `E2E` topic for RDI | One per message | One per message | One per scan (worst result of its messages) |
| Config API frame | SOME/IP header + JSON | SOME/IP header + E2E header + JSON | SOME/IP header + E2E header + JSON |
| Stream destination at startup | Applied with `SetNetworkConfig` once at node start (a sensor powered on later needs a node restart) | Read with `GetNetworkConfig`. Differences are logged as warnings and the sensor is not changed | Same as v1.1.0 |
| Message loss detection | E2E Counter | E2E Counter | E2E Counter (one value per scan) |
| Frames excluded from `PointCloud2` | DataQualifier 1 and 6 (5 = test mode is published) | DataQualifier 1 and 6 | DataQualifier 1 and 6 (from the header message) |

> On interface v1.1.0+, changing the stream protocol, port, or destination requires diagnostic mode, `SetNetworkConfig`, and a sensor restart. The driver does not perform these steps. Frames excluded from `PointCloud2` are still published on `RDI` with their `data_qualifier`.
>
> On interface v1.2.0, a scan with missing chunk messages is published with the detections that arrived and logged as a partial scan. A scan without any header message is not published; its result is still published on `E2E` with `data_published` false.

Runtime update example:

```bash
ros2 param set /afi920/afi920_driver min_range 1.0
ros2 param set /afi920/afi920_driver min_existence_probability 50.0
```

---

## Multi-Sensor

When using multiple AFI920 sensors, each sensor can be isolated with its own namespace and `frame_id`.

```bash
ros2 launch afi920_driver multi_sensor.launch.py
```

You can also launch each sensor individually.

```bash
ros2 launch afi920_driver afi920.launch.py namespace:=radar_front sensor_ip:=192.168.10.150 frame_id:=radar_front
ros2 launch afi920_driver afi920.launch.py namespace:=radar_rear  sensor_ip:=192.168.10.151 frame_id:=radar_rear
```

Expected topics:

```text
/radar_front/PointCloud2    /radar_rear/PointCloud2
/radar_front/RDI            /radar_rear/RDI
/radar_front/SHI            /radar_rear/SHI
/radar_front/SPI            /radar_rear/SPI
```

Recommendations:

- Use a unique `namespace` for each sensor.
- Use a unique `frame_id` for each sensor.
- With `transport_mode:=udp`, use a separate set of host ports (`data_port`/`shi_port`/`spi_port`) for each sensor. A node does not filter UDP datagrams by sensor IP, so sensors sending to the same host port are mixed. Over TCP, all sensors can keep the default ports. See the integration guide, §8 Ports.

### TF

Each node broadcasts its sensor mounting pose from SPI as `parent_frame_id → frame_id`. `multi_sensor.launch.py` sets the common parent with the `parent_frame` argument (default `base_link`), so the four clouds are placed at their mounting positions when the RViz2 Fixed Frame is `base_link`.

```bash
ros2 launch afi920_driver multi_sensor.launch.py parent_frame:=base_link
```

If the vehicle URDF or another node already publishes the sensor frames, set `publish_tf:=false` and publish the transform yourself. Two publishers of the same transform produce conflicting TF.

```bash
ros2 launch afi920_driver afi920.launch.py namespace:=radar_front frame_id:=radar_front publish_tf:=false
ros2 run tf2_ros static_transform_publisher 3.5 0.0 0.7 0 0 0 base_link radar_front
```

---

## Architecture

```text
AFI920 Sensor
   ├─ Data Streams
   └─ Config API
          │
          ▼
     afi920_driver
   ├─ PointCloud2
   ├─ RDI
   ├─ SHI
   ├─ SPI
   ├─ E2E
   └─ /tf (sensor pose)
          │
          ▼
   RViz2 / ROS 2 Applications
```

`PointCloud2` is provided as a standard ROS message, while `RDI`, `SHI`, and `SPI` are provided through dedicated messages in the `afi920_msgs` package.

---

## RViz2 Visualization

The fastest way to check the output is to visualize the `PointCloud2` topic in RViz2.

1. Run `rviz2`
2. Select **Add** → **By topic** → `/afi920/PointCloud2`, and set the display's **Topic → Reliability Policy** to **Best Effort** (the driver publishes with `SensorDataQoS`)
3. Set **Fixed Frame** to `afi920` or your configured `frame_id`. With `publish_tf:=true`, `base_link` also works and places the cloud at the mounting pose

The package installs ready-made RViz2 configurations. `./run.sh` opens `afi920.rviz`, and `./run.sh --multi` opens `multi_sensor.rviz` (Fixed Frame `base_link`, the four `/radar_<pos>/PointCloud2` topics).

```bash
rviz2 -d $(ros2 pkg prefix afi920_driver)/share/afi920_driver/config/multi_sensor.rviz
```

Recommended starting settings:

| Item | Recommended Value |
|------|-------------------|
| Style | Points |
| Size (Pixels) | 4 |
| Color Transformer | AxisColor or intensity-based coloring |

Color examples:

- `intensity`: Reflectivity-based coloring
- `radial_velocity`: Approach / recede velocity-based coloring

---

## Isaac ROS Integration

Isaac ROS NITROS point cloud nodes do not accept the 18-field `PointCloud2` layout. With `publish_pointcloud_xyzi:=true` the driver also publishes `PointCloud2XYZI`, which Isaac ROS accepts:

| Item | `PointCloud2XYZI` |
|------|-------------------|
| Fields | `x`, `y`, `z`, `intensity` (FLOAT32, 16 B/point) |
| Points, header, filters | Same as `PointCloud2` |
| Empty frames | Not published |
| Padding | Point count padded to a multiple of `xyzi_pad_multiple` (default 16) with `x = y = 0`, `z = +inf` points; `is_dense` is then `false` |

`afi920_isaac.launch.py` starts the driver with this topic and the Isaac ROS `PointCloudToFlatScanNode` and `FlatScantoLaserScanNode`, publishing `/afi920/flatscan` and `/afi920/scan`. Isaac ROS (`isaac_ros_pointcloud_utils`) is needed at run time only.

```bash
ros2 launch afi920_driver afi920_isaac.launch.py sensor_ip:=192.168.10.150
```

Verified with Isaac ROS 4.1 on DGX Spark (Ubuntu 24.04, aarch64, Jazzy). The Isaac ROS input conditions and the launch arguments are listed in the [AI Integration Guide](AFI920_ROS2_GUIDE.md#isaac-ros-pipeline-afi920_isaaclaunchpy).

---

## Troubleshooting

| Symptom | Cause | Action |
|---------|-------|--------|
| No topics are published | Sensor connection or network issue | Check the sensor IP, host IP, cable, and subnet first |
| PointCloud2 contains zero points | Filters are too restrictive | Check `min_range`, `min_snr`, and `min_existence_probability` |
| Build or launch fails | ROS 2 environment or dependency issue | Run `source /opt/ros/$ROS_DISTRO/setup.bash` and then retry `rosdep install` |
| Packet loss or intermittent data | Network buffer or NIC issue | Check NIC settings and system receive buffer settings |
| `Sensor stream config differs from driver parameters` warning | The sensor (interface v1.1.0+) uses a different stream protocol, port, or (UDP) destination IP than `host_ip` | Match `transport_mode` and the port parameters to the sensor, or reconfigure the sensor and restart it |
| No `PointCloud2`/`RDI` while `SHI` and `SPI` arrive | A hardware defect or calibration error condition holds (`defect_reason` 2 or 5): the sensor holds RDI back while it holds. `SHI` `operation_modes[0]` then shows 100 (Fault Mode), or 20 (Diagnostic) / 200 (Initialising) when those take priority. Mode 20 by itself only means that the Config API access mode is Diagnostic and does not stop RDI | Check `defect_reason`. A calibration CRC failure lasts the whole boot and clears only after the calibration record is written again and the sensor restarts |
| `RDI scan ... partial` warning | Interface v1.2.0 chunk messages are missing: lost over UDP, or left out by the sensor when its send window ends (TCP and UDP) | The scan is published with the detections that arrived. Persistent warnings over UDP → try `transport_mode:=tcp` |
| `connect to ... failed -- retrying` with the default ports | The stream ports are set per sensor and can differ from the driver defaults 30509–30511 | Read `detection_port`, `health_port` and `performance_port` from the sensor (SDK `GetNetworkConfig` or Discovery) and set `data_port`, `shi_port` and `spi_port` to them |
| `connected` followed at once by `connection ... lost -- reconnecting`, repeating | Another client holds the stream port. The sensor accepts one TCP client per stream channel and resets further connections | Stop the other client (another driver instance, tool or host) |
| Multi-sensor topics overlap | Duplicate namespace or frame_id | Use unique `namespace` and `frame_id` values per sensor |
| RViz2 shows no cloud with Fixed Frame `base_link` | No TF from `base_link` to the sensor frame | Keep `publish_tf` and `publish_performance` at `true`, or publish the transform yourself |
| Sensor frame jumps between two poses | The driver and another node publish the same transform | Set `publish_tf:=false` on the driver |

---

## Security Notice

The AFI920 ROS 2 driver uses plaintext network communication with the sensor.

- Use a dedicated VLAN or a physically isolated network segment for the sensor network whenever possible.
- Restrict access to the network segment connected to the sensor.
- Do not expose the sensor directly to untrusted external networks.

---

## Related Projects

| Project | Description |
|---------|-------------|
| [afi920-sdk](https://github.com/bitsensing/afi920-sdk) | C/C++ and Python SDK for AFI920 |

---

## Changelog

For release history and changes, see [CHANGELOG.md](CHANGELOG.md).

---

## License

BSD-3-Clause

See [LICENSE](LICENSE) for the full license text.

---

## Support

- **GitHub Issues**: Reproducible bugs in the public repository, documentation issues, and improvement suggestions
- **Technical Support**: AFI920 product usage, customer-environment integration, and separately provided documentation (`tech-support@bitsensing.com`)
