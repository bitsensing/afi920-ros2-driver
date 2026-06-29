# AFI920 ROS 2 Driver

![ROS2](https://img.shields.io/badge/ROS2-Foxy%20%7C%20Humble-blue)
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

> **Tip:** Use the `config_file` argument if you want to manage parameters from a file.
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

The tool locates `afi920_sdk` from `AFI920_SDK_ROOT`, `--sdk-root`, or a sibling
`../afi920_sdk` directory, auto-detects the local bind IP for the target radar,
then runs the SDK live conformance suite. It checks Discovery, Config API,
RDI/SHII/SPI data streams, SOME/IP/E2E integrity, and CSII vehicle-input
transmission.

```bash
python3 tools/live_data_check.py 192.168.10.150 --sdk-root /path/to/afi920_sdk
```

---

## Features

- **PointCloud2 Publish**: Publishes AFI920 detections as the standard ROS message `sensor_msgs/PointCloud2`
- **Full RDI Publish**: Provides all ISO 23150 RDI fields through the `DetectionArray` message
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
                  ros-$ROS_DISTRO-visualization-msgs
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
| RDI | 30509 | 0x8002 | Detection / point cloud data |
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
| `RDI` | `afi920_msgs/DetectionArray` | 10 Hz | Full detection data |
| `SHI` | `afi920_msgs/HealthInfo` | 10 Hz | Sensor status and health information |
| `SPI` | `afi920_msgs/SensorPerformance` | 10 Hz | Sensor performance and pose information |

## PointCloud2 Fields

PointCloud2 includes 18 fields.

| # | Field | Unit | Description |
|---|-------|------|-------------|
| 1-3 | x, y, z | m | Cartesian position |
| 4-6 | radial_distance, azimuth_angle, elevation_angle | m, rad | Spherical position |
| 7 | radial_velocity | m/s | Doppler radial velocity |
| 8 | radar_cross_section | dBsm | RCS |
| 9 | signal_noise_ratio | dB | SNR |
| 10-13 | radial_distance_error, azimuth_angle_error, elevation_angle_error, radial_velocity_error | m, rad, m/s | Measurement error information |
| 14 | existence_probability | 0-100 | Detection confidence |
| 15 | multi_target_probability | 0-100 | Multi-target probability |
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
| `min_range` | double | `0.0` | Yes | Minimum range filter (m) |
| `max_range` | double | `300.0` | Yes | Maximum range filter (m) |
| `min_snr` | double | `0.0` | Yes | Minimum SNR filter (dB) |
| `min_existence_probability` | double | `0.0` | Yes | Minimum existence probability filter (0-100) |
| `publish_detections` | bool | `true` | No | Enable publishing on the `RDI` topic |
| `publish_health` | bool | `true` | No | Enable publishing on the `SHI` topic |
| `publish_performance` | bool | `true` | No | Enable publishing on the `SPI` topic |

### Network and Configuration Parameters

| Parameter | Type | Default | Dynamic | Description |
|-----------|------|---------|---------|-------------|
| `data_port` | int | `30509` | No | RDI data port |
| `shi_port` | int | `30510` | No | SHI data port |
| `spi_port` | int | `30511` | No | SPI data port |
| `config_port` | int | `30500` | No | Sensor configuration port |
| `host_ip` | string | `auto` | No | Host IP address (`auto` is supported) |
| `transport_mode` | string | `tcp` | No | Data stream transport (`tcp` or `udp`) |
| `validate_e2e` | bool | `true` | Yes | Validate AUTOSAR E2E CRC on RDI/SHI/SPI |
| `e2e_strict` | bool | `false` | Yes | Drop data stream frames on E2E CRC or metadata mismatch |

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
- Configure TF explicitly when connecting sensors to the vehicle coordinate system.

### TF Example

```bash
ros2 run tf2_ros static_transform_publisher 3.5 0.0 0.7 0 0 0 base_link radar_front
```

> **Tip:** Matching `frame_id` and the TF child frame name simplifies visualization and downstream processing.

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
   └─ SPI
          │
          ▼
   RViz2 / ROS 2 Applications
```

`PointCloud2` is provided as a standard ROS message, while `RDI`, `SHI`, and `SPI` are provided through dedicated messages in the `afi920_msgs` package.

---

## RViz2 Visualization

The fastest way to check the output is to visualize the `PointCloud2` topic in RViz2.

1. Run `rviz2`
2. Select **Add** → **By topic** → `/afi920/PointCloud2`
3. Set **Fixed Frame** to `afi920` or your configured `frame_id`

Recommended starting settings:

| Item | Recommended Value |
|------|-------------------|
| Style | Points |
| Size (m) | 0.05 |
| Color Transformer | AxisColor or intensity-based coloring |

Color examples:

- `intensity`: Reflectivity-based coloring
- `radial_velocity`: Approach / recede velocity-based coloring

---

## Troubleshooting

| Symptom | Cause | Action |
|---------|-------|--------|
| No topics are published | Sensor connection or network issue | Check the sensor IP, host IP, cable, and subnet first |
| PointCloud2 contains zero points | Filters are too restrictive | Check `min_range`, `min_snr`, and `min_existence_probability` |
| Build or launch fails | ROS 2 environment or dependency issue | Run `source /opt/ros/$ROS_DISTRO/setup.bash` and then retry `rosdep install` |
| Packet loss or intermittent data | Network buffer or NIC issue | Check NIC settings and system receive buffer settings |
| Multi-sensor topics overlap | Duplicate namespace or frame_id | Use unique `namespace` and `frame_id` values per sensor |

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
