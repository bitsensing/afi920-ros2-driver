# AFI920 ROS 2 Driver

![ROS2](https://img.shields.io/badge/ROS2-Foxy%20%7C%20Humble-blue)
![Platform](https://img.shields.io/badge/platform-Ubuntu%2020.04%20%7C%2022.04-blue)
![License](https://img.shields.io/badge/license-BSD--3--Clause-green)

bitsensing **AFI920 4D Imaging Radar**용 ROS 2 드라이버 패키지입니다.
AFI920 센서 데이터를 ROS 2 토픽으로 발행하여 고객 환경의 ROS 2 애플리케이션과 연동할 수 있습니다.

> **bitsensing** — [bitsensing.com](https://bitsensing.com)

---

## Scope

- 이 저장소는 **AFI920 고객의 ROS 2 통합용 공개 드라이버**를 제공합니다.
- **제품 매뉴얼, 인터페이스 명세, 설치/운영 상세 가이드**는 별도 제공 문서에서 확인할 수 있습니다.
- 이 README는 **빌드, 실행, 주요 토픽/파라미터, 기본 시각화 흐름**을 빠르게 확인할 수 있도록 구성되어 있습니다.

---

## Table of Contents

- [Scope](#scope)
- [Supported Platforms](#supported-platforms)
- [Quick Start](#quick-start)
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

### 1. 클론 및 빌드

```bash
source /opt/ros/$ROS_DISTRO/setup.bash

cd ~/ros2_ws/src
git clone https://github.com/bitsensing/afi920-ros2-driver.git

cd ~/ros2_ws

# workspace 의존성 설치
rosdep install --from-paths src --ignore-src -r -y

# 드라이버 패키지 빌드
colcon build --packages-select afi920_msgs afi920_driver
source install/setup.bash
```

### 2. 실행

```bash
ros2 launch afi920_driver afi920.launch.py sensor_ip:=192.168.10.150
```

위 명령은 기본 namespace `afi920`와 센서 기본 IP `192.168.10.150`를 기준으로 단일 센서를 실행합니다.

> **Tip:** 전체 파라미터를 파일로 관리하려면 `config_file` 인자를 사용하세요.
>
> ```bash
> ros2 launch afi920_driver afi920.launch.py config_file:=/path/to/custom_params.yaml
> ```

### 3. 데이터 확인

```bash
# 토픽 목록 확인
ros2 topic list

# PointCloud2 수신 주기 확인
ros2 topic hz /afi920/PointCloud2

# 주요 데이터 스트림 확인
ros2 topic echo /afi920/RDI --once
ros2 topic echo /afi920/SHI --once
ros2 topic echo /afi920/SPI --once
```

---

## Features

- **PointCloud2 Publish**: AFI920 검출 결과를 표준 ROS 메시지 `sensor_msgs/PointCloud2`로 발행
- **Full RDI Publish**: ISO 23150 RDI 전체 필드를 `DetectionArray` 메시지로 제공
- **Health and Performance Topics**: 센서 상태 정보와 성능 정보를 `SHI`, `SPI` 토픽으로 제공
- **Multi-Sensor Ready**: namespace 기반으로 다중 센서를 독립적으로 운용 가능
- **Runtime Filtering**: 거리, SNR, 존재 확률 기반 필터를 런타임에 조정 가능
- **RViz2 Friendly**: 기본 PointCloud2 토픽만으로 RViz2에서 바로 시각화 가능

---

## Prerequisites

### ROS 2 의존성

```bash
sudo apt install ros-$ROS_DISTRO-rclcpp \
                  ros-$ROS_DISTRO-rclcpp-components \
                  ros-$ROS_DISTRO-sensor-msgs \
                  ros-$ROS_DISTRO-std-msgs \
                  ros-$ROS_DISTRO-visualization-msgs
```

### 네트워크 설정

AFI920 센서는 Ethernet으로 통신하며, 호스트 PC는 센서와 동일한 서브넷에 있어야 합니다.

| 항목 | 값 |
|------|-----|
| 센서 기본 IP | `192.168.10.150` |
| 호스트 IP | `192.168.10.x` (예: `192.168.10.100`) |
| 서브넷 마스크 | `255.255.255.0` |

예시:

```bash
sudo ip addr add 192.168.10.100/24 dev eth0
```

### 기본 데이터 스트림 포트

| Stream | Port | Event ID | 설명 |
|--------|------|----------|------|
| RDI | 30509 | 0x8002 | Detection / Point Cloud 데이터 |
| SHI  | 30510 | 0x8003 | 센서 상태 및 건강 정보 |
| SPI | 30511 | 0x8004 | 센서 성능 및 자세 정보 |

> **참고:** 방화벽 또는 보안 정책이 있는 환경에서는 사용 중인 포트를 허용해야 합니다.

---

## Topics

기본 namespace는 `afi920`이며, 아래 표에는 namespace 기준 상대 토픽 이름을 정리했습니다.

예: 기본 namespace 사용 시 `PointCloud2`는 `/afi920/PointCloud2`로 발행됩니다.

| Topic | Type | Rate | 설명 |
|-------|------|------|------|
| `PointCloud2` | `sensor_msgs/PointCloud2` | 10 Hz | AFI920 검출 결과를 `sensor_msgs/PointCloud2`로 발행 |
| `RDI` | `afi920_msgs/DetectionArray` | 10 Hz | 전체 Detection 데이터 |
| `SHI` | `afi920_msgs/HealthInfo` | 10 Hz | 센서 상태 및 건강 정보 |
| `SPI` | `afi920_msgs/SensorPerformance` | 10 Hz | 센서 성능 및 자세 정보 |

## PointCloud2 Fields

PointCloud2는 총 18개 필드를 포함합니다.

| # | Field | Unit | 설명 |
|---|-------|------|------|
| 1-3 | x, y, z | m | Cartesian 위치 |
| 4-6 | radial_distance, azimuth_angle, elevation_angle | m, rad | Spherical 위치 |
| 7 | radial_velocity | m/s | Doppler 시선 속도 |
| 8 | radar_cross_section | dBsm | RCS |
| 9 | signal_noise_ratio | dB | SNR |
| 10-13 | radial_distance_error, azimuth_angle_error, elevation_angle_error, radial_velocity_error | m, rad, m/s | 측정 오차 정보 |
| 14 | existence_probability | 0-100 | 검출 신뢰도 |
| 15 | multi_target_probability | 0-100 | Multi-target 가능성 |
| 16 | detection_id | - | Detection 식별자 |
| 17 | sensor_id | - | Sensor 식별자 |
| 18 | intensity | dBsm | RViz 호환용 intensity 값 |

Point stride는 72 bytes이며, 기본 10 Hz 환경에서 일반적인 ROS 2 처리 흐름에 바로 사용할 수 있습니다.

## Coordinate Convention

PointCloud2의 Cartesian 좌표 `(x, y, z)`는 sensor frame 기준 **FLU** `(X forward, Y left, Z up)`입니다.
이는 ROS 표준 좌표계 [REP-103](https://www.ros.org/reps/rep-0103.html)을 따릅니다.

- azimuth 0은 전방 기준입니다.
- azimuth 증가 방향은 `+Y`(좌측)입니다.
- elevation 증가 방향은 `+Z`(상방)입니다.
- 다른 차량 좌표계를 사용하는 경우 TF에서 축 회전을 적용하세요.

---

## Parameters

### 주요 파라미터

| Parameter | Type | Default | Dynamic | 설명 |
|-----------|------|---------|---------|------|
| `sensor_ip` | string | `192.168.10.150` | No | 센서 IP 주소 |
| `frame_id` | string | `afi920` | Yes | 메시지에 적용할 TF frame ID |
| `min_range` | double | `0.0` | Yes | 최소 거리 필터 (m) |
| `max_range` | double | `300.0` | Yes | 최대 거리 필터 (m) |
| `min_snr` | double | `0.0` | Yes | 최소 SNR 필터 (dB) |
| `min_existence_probability` | double | `0.0` | Yes | 최소 존재 확률 필터 (0-100) |
| `publish_detections` | bool | `true` | No | `RDI` 토픽 발행 여부 |
| `publish_health` | bool | `true` | No | `SHI` 토픽 발행 여부 |
| `publish_performance` | bool | `true` | No | `SPI` 토픽 발행 여부 |

### 네트워크 및 설정 파라미터

| Parameter | Type | Default | Dynamic | 설명 |
|-----------|------|---------|---------|------|
| `data_port` | int | `30509` | No | RDI 데이터 포트 |
| `shi_port` | int | `30510` | No | SHI 데이터 포트 |
| `spi_port` | int | `30511` | No | SPI 데이터 포트 |
| `config_port` | int | `30500` | No | 센서 설정 포트 |
| `host_ip` | string | `auto` | No | 호스트 IP 주소 (`auto` 사용 가능) |
| `transport_mode` | string | `tcp` | No | 데이터 스트림 전송 방식 (`tcp` 또는 `udp`) |
| `validate_e2e` | bool | `true` | Yes | RDI/SHI/SPI AUTOSAR E2E CRC 검증 |
| `e2e_strict` | bool | `false` | Yes | E2E CRC 또는 metadata 불일치 프레임 drop |

런타임 변경 예시:

```bash
ros2 param set /afi920/afi920_driver min_range 1.0
ros2 param set /afi920/afi920_driver min_existence_probability 50.0
```

---

## Multi-Sensor

AFI920 여러 대를 사용할 경우, 각 센서를 별도 namespace와 `frame_id`로 분리해 운용할 수 있습니다.

```bash
ros2 launch afi920_driver multi_sensor.launch.py
```

또는 개별 launch를 각각 실행할 수도 있습니다.

```bash
ros2 launch afi920_driver afi920.launch.py namespace:=radar_front sensor_ip:=192.168.10.150 frame_id:=radar_front
ros2 launch afi920_driver afi920.launch.py namespace:=radar_rear  sensor_ip:=192.168.10.151 frame_id:=radar_rear
```

예상 토픽:

```text
/radar_front/PointCloud2    /radar_rear/PointCloud2
/radar_front/RDI            /radar_rear/RDI
/radar_front/SHI            /radar_rear/SHI
/radar_front/SPI            /radar_rear/SPI
```

권장 사항:

- 각 센서에 고유한 `namespace`를 사용하세요.
- 각 센서에 고유한 `frame_id`를 사용하세요.
- 차량 좌표계와 연결할 때는 TF를 명확히 구성하세요.

### TF 연결 예시

```bash
ros2 run tf2_ros static_transform_publisher 3.5 0.0 0.7 0 0 0 base_link radar_front
```

> **Tip:** `frame_id`와 TF child frame 이름을 동일하게 맞추면 시각화 및 후처리 구성이 단순해집니다.

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

`PointCloud2`는 표준 ROS 메시지로 제공되며, `RDI`, `SHI`, `SPI`는 `afi920_msgs` 패키지의 전용 메시지로 제공됩니다.

---

## RViz2 Visualization

가장 빠른 확인 방법은 RViz2에서 `PointCloud2` 토픽을 바로 보는 것입니다.

1. `rviz2` 실행
2. **Add** → **By topic** → `/afi920/PointCloud2` 선택
3. **Fixed Frame**을 `afi920` 또는 사용 중인 `frame_id`로 설정

권장 시작 설정:

| 항목 | 권장값 |
|------|--------|
| Style | Points |
| Size (m) | 0.05 |
| Color Transformer | AxisColor 또는 intensity 기반 색상 |

색상 확인 예시:

- `intensity`: 반사 강도 기반 시각화
- `radial_velocity`: 접근/이탈 속도 기반 시각화

---

## Troubleshooting

| 증상 | 원인 | 해결 |
|------|------|------|
| 토픽이 출력되지 않음 | 센서 연결 또는 네트워크 문제 | 센서 IP, 호스트 IP, 케이블, 서브넷을 먼저 확인하세요 |
| PointCloud2 데이터가 0개 | 필터 설정이 너무 엄격함 | `min_range`, `min_snr`, `min_existence_probability` 값을 확인하세요 |
| 빌드 또는 launch 실패 | ROS 2 환경 또는 의존성 문제 | `source /opt/ros/$ROS_DISTRO/setup.bash` 후 `rosdep install`을 다시 실행하세요 |
| 패킷 손실 또는 데이터 끊김 | 네트워크 버퍼 또는 NIC 환경 문제 | NIC 설정과 시스템 수신 버퍼를 점검하세요 |
| 멀티센서 토픽이 섞임 | namespace 또는 frame_id 중복 | 센서별로 고유한 `namespace`, `frame_id`를 사용하세요 |

---

## Security Notice

AFI920 ROS 2 드라이버는 센서와 평문 기반 네트워크 통신을 사용합니다.

- 센서 네트워크는 전용 VLAN 또는 물리적으로 분리된 환경에서 운용하는 것을 권장합니다.
- 센서가 연결된 네트워크 세그먼트에 대한 접근 권한을 제한하세요.
- 신뢰할 수 없는 외부 네트워크에 센서를 직접 노출하지 마세요.

---

## Related Projects

| 프로젝트 | 설명 |
|---------|------|
| [afi920-sdk](https://github.com/bitsensing/afi920-sdk) | AFI920용 C/C++ 및 Python SDK |

---

## Changelog

릴리스 및 변경 이력은 [CHANGELOG.md](CHANGELOG.md)를 참조하세요.

---

## License

BSD-3-Clause

전체 라이선스 조건은 [LICENSE](LICENSE) 파일을 참조하세요.

---

## Support

- **GitHub Issues**: 공개 레포 사용 중 재현 가능한 버그, 문서 오탈자, 개선 제안
- **기술 지원**: AFI920 제품 사용, 고객 환경 통합, 별도 제공 문서 관련 문의 (`tech-support@bitsensing.com`)
