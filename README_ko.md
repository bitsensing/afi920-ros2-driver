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
- 전체 파라미터, 메시지 정의, 프로토콜 세부는 통합 참조 문서 [AFI920_ROS2_GUIDE_ko.md](AFI920_ROS2_GUIDE_ko.md)에서 확인한다.

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
                  ros-$ROS_DISTRO-visualization-msgs \
                  ros-$ROS_DISTRO-geometry-msgs \
                  ros-$ROS_DISTRO-tf2 \
                  ros-$ROS_DISTRO-tf2-ros
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
| `E2E` | `afi920_msgs/E2eStatus` | 30 Hz | RDI/SHI/SPI 메시지별 E2E 판정 결과. `header.stamp`는 대응 데이터 메시지의 stamp와 같다 |

`publish_tf`가 `true`(기본값)이면 드라이버는 SPI 메시지마다 수신한 센서 장착 자세를 `/tf`에 `parent_frame_id → frame_id` 변환(기본 `base_link → afi920`)으로 송출한다. `publish_performance`가 `false`이면 자세를 송출하지 않는다.

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
| `parent_frame_id` | string | `base_link` | No | 센서 자세 TF의 부모 frame |
| `publish_tf` | bool | `true` | No | SPI 장착 자세를 `parent_frame_id → frame_id` TF로 송출. `publish_performance`가 `true`여야 한다 |
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
| `validate_e2e` | bool | `true` | Yes | RDI/SHI/SPI E2E CRC 검증. 결과는 `E2E` 토픽으로 발행하며, CRC 불일치로 프레임을 폐기하지 않는다 |
| `e2e_strict` | bool | `false` | Yes | E2E Length 또는 Data ID 불일치 프레임 drop |

### 센서 인터페이스 호환

드라이버는 AFI920 인터페이스 v1.0.x, v1.1.0, v1.2.0을 별도 파라미터 없이 지원한다. 형식은 센서가 송신한 프레임으로 판별한다.

| 항목 | 인터페이스 v1.0.x | 인터페이스 v1.1.0 | 인터페이스 v1.2.0 |
|------|------------------|------------------|------------------|
| RDI/SHI/SPI E2E CRC | CRC-64/XZ | 본문 대상 IEEE 802.3 CRC-32 | 본문 대상 IEEE 802.3 CRC-32 |
| RDI 프레이밍 | 스캔당 메시지 1건 (UDP는 SOME/IP-TP 분할) | 스캔당 메시지 1건 (UDP는 SOME/IP-TP 분할) | 스캔당 헤더 A, 묶음 메시지(Event `0x8100`+k), 헤더 B. 메시지마다 E2E 헤더를 두며 SOME/IP-TP를 쓰지 않는다 |
| RDI의 `E2E` 토픽 | 메시지당 1건 | 메시지당 1건 | 스캔당 1건 (구성 메시지 중 최악 판정) |
| Config API 프레임 | SOME/IP 헤더 + JSON | SOME/IP 헤더 + E2E 헤더 + JSON | SOME/IP 헤더 + E2E 헤더 + JSON |
| 기동 시 스트림 목적지 | `SetNetworkConfig`로 적용 | `GetNetworkConfig`로 조회. 차이는 경고로 출력하며 센서 설정을 변경하지 않는다 | v1.1.0과 같다 |
| 메시지 손실 판정 | E2E Counter | E2E Counter | E2E Counter (스캔당 1개 값) |
| `PointCloud2` 제외 프레임 | DataQualifier 1, 6 | DataQualifier 1, 6 | DataQualifier 1, 6 |

> 인터페이스 v1.1.0 이후에서 스트림 프로토콜, 포트, 목적지를 변경하려면 진단 모드 진입, `SetNetworkConfig`, 센서 재시작이 필요하다. 드라이버는 이 절차를 수행하지 않는다. `PointCloud2`에서 제외된 프레임도 `RDI` 토픽에는 `data_qualifier`와 함께 발행한다.
>
> 인터페이스 v1.2.0에서 묶음 메시지가 누락된 스캔은 수신한 검출만으로 발행하며 부분 스캔 경고를 출력한다. 헤더 메시지를 하나도 받지 못한 스캔은 발행하지 않으며, 그 판정은 `data_published`가 false인 `E2E` 메시지로 발행한다.

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

### TF

각 노드는 SPI로 수신한 센서 장착 자세를 `parent_frame_id → frame_id` 변환으로 송출한다. `multi_sensor.launch.py`는 `parent_frame` 인자(기본 `base_link`)로 공통 부모 frame을 지정한다. RViz2 Fixed Frame을 `base_link`로 두면 4개 점군이 각 장착 위치에 표시된다.

```bash
ros2 launch afi920_driver multi_sensor.launch.py parent_frame:=base_link
```

차량 URDF나 다른 노드가 센서 frame을 이미 송출하면 `publish_tf:=false`로 두고 변환을 직접 송출한다. 같은 변환을 두 송출원이 내보내면 TF가 충돌한다.

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
   └─ /tf (센서 자세)
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
3. **Fixed Frame**을 `afi920` 또는 사용 중인 `frame_id`로 설정. `publish_tf:=true`이면 `base_link`도 사용할 수 있으며, 점군이 장착 자세에 맞춰 표시된다

패키지는 RViz2 설정 파일을 함께 설치한다. `./run.sh`는 `afi920.rviz`를, `./run.sh --multi`는 `multi_sensor.rviz`(Fixed Frame `base_link`, `/radar_<pos>/PointCloud2` 토픽 4개)를 연다.

```bash
rviz2 -d $(ros2 pkg prefix afi920_driver)/share/afi920_driver/config/multi_sensor.rviz
```

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
| `Sensor stream config differs from driver parameters` 경고 | 센서(인터페이스 v1.1.0 이후)의 스트림 프로토콜 또는 포트가 다름 | `transport_mode`와 포트 파라미터를 센서 설정에 맞추거나, 센서를 재설정한 뒤 재시작한다 |
| `SHI`와 `SPI`는 수신되나 `PointCloud2`/`RDI`가 없음 | `SHI`의 `operation_modes[0]`이 100(Fault Mode)이며, 센서는 이 부팅에서 RDI를 송신하지 않는다 | 센서 Calibration record를 점검하고 센서를 재시작한다 |
| `RDI scan ... partial` 경고 | 인터페이스 v1.2.0 묶음 메시지 손실 (UDP) | 스캔은 수신한 검출만으로 발행된다. UDP에서 경고가 지속되면 `transport_mode:=tcp`를 사용한다 |
| 멀티센서 토픽이 섞임 | namespace 또는 frame_id 중복 | 센서별로 고유한 `namespace`, `frame_id`를 사용하세요 |
| Fixed Frame `base_link`에서 점군이 표시되지 않음 | `base_link`에서 센서 frame으로 가는 TF가 없음 | `publish_tf`와 `publish_performance`를 `true`로 유지하거나 변환을 직접 송출한다 |
| 센서 frame이 두 자세 사이를 오감 | 드라이버와 다른 노드가 같은 변환을 송출함 | 드라이버에 `publish_tf:=false`를 지정한다 |

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
