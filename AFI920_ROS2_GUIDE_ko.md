# AFI920 ROS 2 드라이버 — AI 통합 가이드

> **대상:** AI 에이전트, LLM 기반 자동화, bitsensing AFI920 4D 이미징 레이더를 ROS 2 시스템에 가장 빠르게 통합하려는 개발자  
> **드라이버 버전:** 2.1.0 · **지원 ROS 2:** Foxy (Ubuntu 20.04), Humble (Ubuntu 22.04) · **아키텍처:** x86_64, aarch64  
> **라이선스:** BSD-3-Clause

---

## 목차

1. [드라이버 개요](#1-드라이버-개요)
2. [레포지토리 구조](#2-레포지토리-구조)
3. [네트워크 사전 조건](#3-네트워크-사전-조건)
4. [빌드 & 설치](#4-빌드--설치)
5. [빠른 시작: 단일 센서](#5-빠른-시작-단일-센서)
6. [발행 토픽](#6-발행-토픽)
7. [전체 노드 파라미터](#7-전체-노드-파라미터)
8. [멀티 센서 설정](#8-멀티-센서-설정)
9. [커스텀 메시지 정의](#9-커스텀-메시지-정의)
10. [PointCloud2 필드 레퍼런스](#10-pointcloud2-필드-레퍼런스)
11. [동적 파라미터 튜닝](#11-동적-파라미터-튜닝)
12. [진단 도구](#12-진단-도구)
13. [포트 & 프로토콜 레퍼런스](#13-포트--프로토콜-레퍼런스)
14. [문제 해결 체크리스트](#14-문제-해결-체크리스트)

---

## 1. 드라이버 개요

AFI920 ROS 2 드라이버는 AFI920 SDK를 컴포저블 ROS 2 노드로 래핑합니다. 이더넷으로 레이더에 연결하여 ~10 Hz로 네 가지 데이터 토픽을 발행합니다:

| 제공 데이터 | 토픽 | 타입 |
|---|---|---|
| 표준 포인트 클라우드 (RViz 즉시 사용 가능) | `~/PointCloud2` | `sensor_msgs/PointCloud2` |
| 완전한 ISO 23150 감지 데이터 | `~/RDI` | `afi920_msgs/DetectionArray` |
| 센서 상태 & 장애 정보 | `~/SHI` | `afi920_msgs/HealthInfo` |
| 센서 성능 & 장착 포즈 | `~/SPI` | `afi920_msgs/SensorPerformance` |

이 외에도 RViz 오버레이용 `~/detection_count` 마커를 발행하고, SPI에서 받은 센서 장착 포즈를 `parent_frame_id → frame_id` TF 변환으로 브로드캐스트합니다. [§6](#6-발행-토픽) 참고.

모든 데이터 토픽은 `SensorDataQoS`를 사용합니다. TCP(기본값) 및 UDP 전송, AUTOSAR E2E CRC 검증, SOME/IP-TP 세그먼트 재조립, 센서 포즈 TF 브로드캐스트, `ros2 param set`을 통한 런타임 필터 조정을 지원합니다.

---

## 2. 레포지토리 구조

```
afi920-ros2-driver/
├── run.sh                          ← 빌드 + 런치 편의 스크립트
├── src/
│   ├── afi920_msgs/                ← 커스텀 메시지 패키지
│   │   └── msg/
│   │       ├── Detection.msg       ← 단일 감지 (완전한 ISO 23150)
│   │       ├── DetectionArray.msg  ← 헤더 포함 RDI 프레임
│   │       ├── HealthInfo.msg      ← SHII 센서 상태
│   │       ├── SensorPerformance.msg ← SPI 프레임
│   │       ├── SensorPose.msg      ← 장착 위치 & 방향
│   │       ├── FovSegment.msg      ← 각도 구역별 FoV 차단
│   │       ├── RecognisableObject.msg ← 물체 유형별 감지 범위
│   │       └── ReferenceTarget.msg ← 참조 표적 특성
│   └── afi920_driver/
│       ├── src/afi920_node.cpp     ← 메인 컴포저블 노드
│       └── launch/
│           ├── afi920.launch.py        ← 단일 센서
│           └── multi_sensor.launch.py  ← 최대 4개 센서
└── tools/
    ├── live_data_check.py   ← 엔드투엔드 센서 검증
    └── rdi_diag.py          ← SOME/IP + E2E + RDI 바이트 레벨 디버거
```

---

## 3. 네트워크 사전 조건

| 항목 | 기본값 |
|------|-------|
| 센서 IP | `192.168.10.150` |
| 호스트 IP | `192.168.10.x` |
| 서브넷 마스크 | `255.255.255.0` |

**실행 전 필수 확인 사항:**
1. 호스트와 레이더를 이더넷으로 연결합니다.
2. 호스트에 `192.168.10.x` 대역의 정적 IP를 할당합니다.
3. 연결 확인: `ping 192.168.10.150`

---

## 4. 빌드 & 설치

### 표준 colcon 빌드

```bash
source /opt/ros/$ROS_DISTRO/setup.bash   # foxy 또는 humble

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
# afi920-ros2-driver/ 를 여기에 위치 (git clone 또는 복사)

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select afi920_msgs afi920_driver \
             --symlink-install
source install/setup.bash
```

> `afi920_msgs`를 먼저 빌드해야 합니다 — `afi920_driver`가 의존합니다. `colcon`이 순서를 자동으로 처리합니다.

### 원커맨드 단축 (run.sh)

```bash
cd afi920-ros2-driver
./run.sh                    # 빌드 + 단일 센서 런치 + RViz
./run.sh --no-build         # 빌드 생략, 런치 + RViz만
./run.sh --multi            # 빌드 + 4센서 런치 + RViz
./run.sh --no-build --multi # 빌드 생략, 4센서 + RViz
```

### 패키지 의존성

| 패키지 | 주요 의존성 |
|--------|-----------|
| `afi920_msgs` | `std_msgs`, `sensor_msgs`, `rosidl_default_generators` |
| `afi920_driver` | `rclcpp`, `rclcpp_components`, `sensor_msgs`, `std_msgs`, `visualization_msgs`, `geometry_msgs`, `tf2`, `tf2_ros`, `afi920_msgs` |

---

## 5. 빠른 시작: 단일 센서

```bash
# 최소 런치 — 모든 기본값 사용
ros2 launch afi920_driver afi920.launch.py

# 주요 옵션 지정
ros2 launch afi920_driver afi920.launch.py \
    sensor_ip:=192.168.10.150 \
    namespace:=afi920 \
    frame_id:=afi920 \
    transport_mode:=tcp
```

**데이터 수신 확인:**
```bash
ros2 topic hz /afi920/PointCloud2      # ~10 Hz 예상
ros2 topic echo /afi920/RDI --once
ros2 topic echo /afi920/SHI --once
ros2 topic echo /afi920/SPI --once
```

---

## 6. 발행 토픽

기본 네임스페이스: `afi920` (`namespace` 런치 인자로 설정).  
토픽 이름은 상대 이름(예: `PointCloud2`)으로 선언되므로, 네임스페이스 접두사가 자동으로 적용되어 아래 절대 토픽이 만들어집니다.

| 토픽 (절대 경로) | 메시지 타입 | QoS | 주기 |
|---|---|---|---|
| `/afi920/PointCloud2` | `sensor_msgs/PointCloud2` | SensorDataQoS | 10 Hz |
| `/afi920/RDI` | `afi920_msgs/DetectionArray` | SensorDataQoS | 10 Hz |
| `/afi920/SHI` | `afi920_msgs/HealthInfo` | SensorDataQoS | 10 Hz |
| `/afi920/SPI` | `afi920_msgs/SensorPerformance` | SensorDataQoS | 10 Hz |
| `/afi920/detection_count` | `visualization_msgs/Marker` | SensorDataQoS | 10 Hz |

`detection_count`는 RViz 오버레이용 `TEXT_VIEW_FACING` 마커(`Pts: <n>`)이며, `PointCloud2`와 함께 항상 발행됩니다.

**TF 브로드캐스트:** `publish_tf`가 `true`(기본값)이고 `publish_performance`가 `true`이면, 노드는 각 SPI 프레임에서 받은 센서 장착 포즈를 `/tf`(`tf2_msgs/TFMessage`)에 `parent_frame_id → frame_id` 변환(기본 `base_link → afi920`)으로 브로드캐스트합니다. 고정 프레임(fixed frame)을 `parent_frame_id`로 설정하면 RViz/소비자가 각 센서의 클라우드를 보정된 위치에 배치할 수 있습니다. `publish_performance`가 `false`이면 SPI 포즈가 없으므로 TF는 브로드캐스트되지 않습니다(경고 로그 출력).

**좌표 규약:** 센서 프레임 FLU (X=전방, Y=좌측, Z=상방) — REP-103 준수

---

## 7. 전체 노드 파라미터

### 연결 설정 (동적 변경 불가 — 런치 시 설정)

| 파라미터 | 타입 | 기본값 | 설명 |
|---------|------|-------|------|
| `sensor_ip` | string | `192.168.10.150` | 레이더 IP 주소 |
| `host_ip` | string | `auto` | 로컬 바인드 IP (`auto` = 센서 경로에서 자동 탐지) |
| `data_port` | int | `30509` | RDI 수신 포트 |
| `shi_port` | int | `30510` | SHII 수신 포트 |
| `spi_port` | int | `30511` | SPI 수신 포트 |
| `config_port` | int | `30500` | Config API TCP 포트 |
| `transport_mode` | string | `tcp` | `tcp` 또는 `udp` |

### 발행 설정 (동적 변경 불가)

| 파라미터 | 타입 | 기본값 | 설명 |
|---------|------|-------|------|
| `publish_detections` | bool | `true` | `RDI` 토픽 발행 |
| `publish_health` | bool | `true` | `SHI` 토픽 발행 |
| `publish_performance` | bool | `true` | `SPI` 토픽 발행 |

### TF 설정 (동적 변경 불가)

| 파라미터 | 타입 | 기본값 | 설명 |
|---------|------|-------|------|
| `publish_tf` | bool | `true` | 센서 장착 포즈(SPI)를 TF로 브로드캐스트. `publish_performance:=true` 필요 |
| `parent_frame_id` | string | `base_link` | 부모 TF 프레임; 포즈가 `parent_frame_id → frame_id`로 브로드캐스트됨 |

### 필터링 & 프레임 설정 (동적 변경 가능 — 런타임 변경)

| 파라미터 | 타입 | 기본값 | 설명 |
|---------|------|-------|------|
| `frame_id` | string | `afi920` | 발행 메시지의 TF 프레임 ID |
| `min_range` | double | `0.0` m | 이 거리 미만 감지 제거 |
| `max_range` | double | `300.0` m | 이 거리 초과 감지 제거 |
| `min_snr` | double | `0.0` dB | 이 SNR 미만 감지 제거 |
| `min_existence_probability` | double | `0.0` % | 이 신뢰도 미만 감지 제거 |

### E2E 무결성 (동적 변경 가능)

| 파라미터 | 타입 | 기본값 | 설명 |
|---------|------|-------|------|
| `validate_e2e` | bool | `true` | 모든 프레임에 AUTOSAR E2E CRC-64/XZ 검증 |
| `e2e_strict` | bool | `false` | `true`이면 E2E 검증 실패 프레임 폐기 |

### 전체 런치 인자 목록 (afi920.launch.py)

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

## 8. 멀티 센서 설정

최대 4개 센서를 동시에 운용하며, 각각 독립된 네임스페이스로 분리됩니다.

### 런치

```bash
ros2 launch afi920_driver multi_sensor.launch.py \
    front_ip:=192.168.10.150 \
    rear_ip:=192.168.10.151 \
    left_ip:=192.168.10.152 \
    right_ip:=192.168.10.153
```

### 생성되는 네임스페이스 & 토픽

| 센서 위치 | 네임스페이스 | 프레임 ID | 토픽 예시 |
|---------|-----------|---------|---------|
| 전방 | `radar_front` | `radar_front` | `/radar_front/PointCloud2` |
| 후방 | `radar_rear` | `radar_rear` | `/radar_rear/PointCloud2` |
| 좌측 | `radar_left` | `radar_left` | `/radar_left/PointCloud2` |
| 우측 | `radar_right` | `radar_right` | `/radar_right/PointCloud2` |

### 센서별 프레임 이름 변경

```bash
ros2 launch afi920_driver multi_sensor.launch.py \
    front_ip:=192.168.10.150  front_frame:=radar_front \
    rear_ip:=192.168.10.151   rear_frame:=radar_rear
```

### 런치 인자

센서별 (`front` / `rear` / `left` / `right` 각각에 대해 한 쌍씩):

| 인자 | 기본값 | 설명 |
|------|-------|------|
| `<pos>_ip` | `192.168.10.150…153` | 해당 위치 센서 IP |
| `<pos>_frame` | `radar_<pos>` | 해당 센서의 TF 프레임 ID |

네 센서 공통:

| 인자 | 기본값 | 설명 |
|------|-------|------|
| `transport_mode` | `tcp` | `tcp` 또는 `udp` |
| `validate_e2e` | `true` | 데이터 스트림 AUTOSAR E2E CRC 검증 |
| `e2e_strict` | `false` | E2E 불일치 프레임 폐기 |
| `parent_frame` | `base_link` | 공통 부모 TF 프레임; 각 포즈가 `parent_frame → <pos>_frame`로 브로드캐스트됨 |
| `publish_tf` | `true` | 각 센서 장착 포즈(SPI)를 TF로 브로드캐스트 |
| `config_file` | `""` | YAML 설정 파일 경로(개별 파라미터 대체; 예: `config/multi_sensor.yaml`) |

`publish_detections`, `publish_health`, `publish_performance`는 이 런치 파일에서 모든 센서에 대해 `true`로 고정되어 있습니다(인자로 노출되지 않음). 이를 변경하려면 `config_file`을 사용하세요.

### 포트

`multi_sensor.launch.py`는 센서별 포트를 노출하거나 설정하지 **않습니다**. 각 센서는 자신의
IP로 **동일한 기본 포트**(30509/30510/30511)에 연결되고, 각 노드는 독립된 네임스페이스에서
실행되므로 충돌이 없습니다(TCP에서는 호스트 측이 임시 포트를 사용). 포트를 변경하려면 레이더
자체에서 포트를 바꾸고, 일치하는 `data_port`/`shi_port`/`spi_port`를 담은 `config_file`을
전달하세요.

---

## 9. 커스텀 메시지 정의

### `afi920_msgs/DetectionArray` — RDI 프레임

```
std_msgs/Header header
uint8  sensor_id
uint32 message_counter
uint64 timestamp_measurement_ns      # 센서 PTP 타임스탬프 (나노초)
uint8  data_qualifier                # 0=normal 1=not_available 2=reduced_coverage 3=reduced_perf

float32 velocity_ambiguity_begin     # m/s  (기본: -88.89)
float32 velocity_ambiguity_end       # m/s  (기본:  55.56)
float32 range_ambiguity_begin        # m    (기본:   0.0)
float32 range_ambiguity_end          # m    (기본: 300.0)
float32 azimuth_ambiguity_begin      # rad  (기본: -1.0472, ≈ -60°)
float32 azimuth_ambiguity_end        # rad  (기본:  1.0472, ≈ +60°)
float32 elevation_ambiguity_begin    # rad  (기본: -0.2618, ≈ -15°)
float32 elevation_ambiguity_end      # rad  (기본:  0.2618, ≈ +15°)

uint16 max_detections                # 센서 최대 감지 수 (일반적으로 2048)
uint8  recognised_detections_status  # 0=ok  1=performance_limit_reached
afi920_msgs/Detection[] detections
```

### `afi920_msgs/Detection` — 단일 감지

```
# 식별
uint16  detection_id
uint16  object_id_reference    # 예약, 0xFFFF = 추적 없음
uint8   existence_probability  # 0–100 %
float32 timestamp_difference   # 헤더 타임스탬프 기준 오프셋 (s)

# 위치 (구면 좌표, 센서 수신값)
float32 radial_distance        # m (0–300)
float32 azimuth                # rad (±60°)
float32 elevation              # rad (±15°)
float32 radial_distance_error  # m
float32 azimuth_error          # rad
float32 elevation_error        # rad

# 위치 (구면→데카르트 변환값; X=전방, Y=좌측, Z=상방)
float32 x                      # m
float32 y                      # m
float32 z                      # m

# 속도
float32 radial_velocity        # m/s (음수 = 접근)
float32 radial_velocity_error  # m/s

# 신호 품질
float32 rcs                    # dBm² (레이더 단면적)
float32 rcs_error              # dBm²
float32 snr                    # dB
float32 snr_error              # dB

# 확률
uint8   multi_target_probability        # 0–100 %
uint16  ambiguity_grouping_id
uint8   detection_ambiguity_probability # 예약
uint8   free_space_probability          # 예약

# 분류 (최대 8 슬롯)
uint8   num_classifications
uint8[8] classification_type   # 0=none 1=noise 2=obstacle 3=under 4=over 5=nearest 6=strongest
uint8[8] classification_confidence  # 슬롯별 0–100

# 디버그 필드 (ISO 23150 사용자 정의 확장)
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

# 센서 식별자 (멀티 센서용)
uint8   sensor_id
```

### `afi920_msgs/HealthInfo` — SHII 센서 상태

```
std_msgs/Header header
uint8  sensor_id
uint32 message_counter
uint64 timestamp_measurement_ns
uint8  data_qualifier

# 동작 모드 (최대 11개)
uint8   num_operation_modes
uint8[11] operation_modes
# 값: 0=normal_dual_range  1=normal_long_range  2=normal_middle_range
#     3=normal_short_range 4=normal_ultra_long_range  5=normal_ultra_short_range
#     10=degradation  50=evaluation  100=calibration
#     200=initialising  201=test

# 장애 상태
uint8 defect_recognised        # 0=fully_functional  1=not_fully  2=out_of_order
uint8 defect_reason            # 0=none  1=memory  2=hardware  3=thermal  4=communication
                               # 5=calibration  6=configuration  7=mechanical
                               # 8=software  9=computing_power  10=time_sync  11=external

# 전원 & 온도
uint8 supply_voltage_status    # 0=low  1=pre_low  2=ok  3=pre_high  4=high
uint8 temperature_status       # 0=under  1=pre_under  2=ok  3=pre_over  4=over

# 입력 신호 상태 (최대 4개)
uint8   num_input_signal_statuses
uint8[4] input_signal_types
uint8[4] input_signal_statuses  # 0=valid 1=implausible 2=missing 3=out_of_range 4=timeout

# 시간 동기화
uint8   time_sync_status       # 0=ok  1=out_of_limits  2=timeout  3=not_synchronized
float32 time_sync_offset       # 레거시 필드, 항상 0.0

# 캘리브레이션 (병렬 배열, 최대 3개 구성요소)
uint8   num_calibration_components
uint8[3] calibration_components # 0=intrinsic  1=extrinsic_horizontal  2=extrinsic_vertical
uint8[3] calibration_statuses   # 0=calibrated  1=not_calibrated  2=degraded
uint8[3] calibration_states     # 0=initial_done 1=initial_not_done 2=initial_failed
                                # 3=recal_intrinsic 4=recal_extrinsic 5=recal_full
```

### `afi920_msgs/SensorPerformance` — SPI 프레임

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
float32 origin_x / origin_y / origin_z          # 장착 위치 (m)
float32 origin_error_x / origin_error_y / origin_error_z
float32 yaw / pitch / roll                       # 장착 방향 (rad)
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

## 10. PointCloud2 필드 레퍼런스

18개 필드, 모두 `FLOAT32`, **72바이트 포인트 스트라이드**.

| # | 필드명 | 단위 | 설명 |
|---|-------|------|------|
| 1 | `x` | m | 데카르트 X (전방) |
| 2 | `y` | m | 데카르트 Y (좌측) |
| 3 | `z` | m | 데카르트 Z (상방) |
| 4 | `radial_distance` | m | 방사 거리 |
| 5 | `azimuth_angle` | rad | 수평 각도 (양수 = 좌측) |
| 6 | `elevation_angle` | rad | 수직 각도 (양수 = 상방) |
| 7 | `radial_velocity` | m/s | 도플러; 음수 = 접근 |
| 8 | `radar_cross_section` | dBsm | RCS |
| 9 | `signal_noise_ratio` | dB | SNR |
| 10 | `radial_distance_error` | m | |
| 11 | `azimuth_angle_error` | rad | |
| 12 | `elevation_angle_error` | rad | |
| 13 | `radial_velocity_error` | m/s | |
| 14 | `existence_probability` | % | 0–100 감지 신뢰도 |
| 15 | `multi_target_probability` | % | 0–100 |
| 16 | `detection_id` | — | 프레임 내 고유 ID |
| 17 | `sensor_id` | — | 소스 센서 식별자 |
| 18 | `intensity` | dBsm | = `radar_cross_section`; RViz 컬러맵용 |

---

## 11. 동적 파라미터 튜닝

아래 파라미터는 노드 실행 중에도 변경 가능합니다 — 재시작 불필요.

```bash
# 거리 필터 조정
ros2 param set /afi920/afi920_driver min_range 1.0
ros2 param set /afi920/afi920_driver max_range 100.0

# 노이즈 감지 제거
ros2 param set /afi920/afi920_driver min_snr 5.0
ros2 param set /afi920/afi920_driver min_existence_probability 30.0

# TF 프레임 변경
ros2 param set /afi920/afi920_driver frame_id radar_front

# E2E 엄격 모드 활성화
ros2 param set /afi920/afi920_driver e2e_strict true
```

**현재 값 전체 조회:**
```bash
ros2 param list /afi920/afi920_driver
ros2 param dump /afi920/afi920_driver
```

**YAML로 저장 후 재사용:**
```bash
ros2 param dump /afi920/afi920_driver > afi920_params.yaml
# 런치 시 적용:
ros2 launch afi920_driver afi920.launch.py config_file:=afi920_params.yaml
```

---

## 12. 진단 도구

### `live_data_check.py` — 엔드투엔드 센서 검증

하나 이상의 센서에 대해 연결 및 데이터 흐름 전체를 테스트합니다.

```bash
cd tools/

# 단일 센서 (기본 IP)
python3 live_data_check.py

# 커스텀 IP, 10초 테스트
python3 live_data_check.py 192.168.10.150 --duration 10.0

# 멀티 센서
python3 live_data_check.py 192.168.10.150 192.168.10.151

# CSII 차량 입력 테스트 생략
python3 live_data_check.py --skip-csii

# SDK 경로 직접 지정
python3 live_data_check.py --sdk-root /path/to/afi920-sdk
```

테스트 항목: Discovery · Config API · RDI/SHII/SPI 스트림 · SOME/IP + E2E 무결성 · CSII 차량 입력

### `rdi_diag.py` — 바이트 레벨 RDI 프로토콜 디버거

원시 프레임을 캡처하여 SOME/IP 프레이밍, E2E CRC, RDI 헤더/감지 필드를 검증합니다.

```bash
# UDP (기본값)
python3 rdi_diag.py --sensor-ip 192.168.10.150 --port 30509 --max-frames 5

# TCP
python3 rdi_diag.py --sensor-ip 192.168.10.150 --tcp --max-frames 5
```

출력 내용: SOME/IP 헤더 · E2E CRC 결과 · RDI 헤더 필드 · 처음 10개 감지 데이터 및 NaN 진단

---

## 13. 포트 & 프로토콜 레퍼런스

| 포트 | 프로토콜 | 방향 | 용도 |
|-----|---------|------|------|
| 30500 | TCP | ↔ | Config API (센서 IP, 스트림 목적지) |
| 30509 | TCP/UDP | ← | RDI — 포인트 클라우드 (Event ID `0x8002`) |
| 30510 | TCP/UDP | ← | SHII — 상태 정보 (Event ID `0x8003`) |
| 30511 | TCP/UDP | ← | SPI — 성능 정보 (Event ID `0x8004`) |

**내부 버퍼 크기:**
| 스트림 | TCP | UDP |
|--------|-----|-----|
| RDI | 512 KB | 400 KB |
| SHII | 4 KB | 4 KB |
| SPI | 4 KB | 4 KB |

**프로토콜 스택:**
```
ROS 2 토픽 (sensor_msgs / afi920_msgs)
    ↑
ISO 23150 페이로드 (바이너리, 리틀 엔디안)
    ↑
AUTOSAR E2E Profile 7 헤더 (20B · CRC-64/XZ)
    ↑
SOME/IP-TP 헤더 (16B + 4B TP · 빅 엔디안)
    ↑
TCP / UDP
```

**멀티 센서:** 센서는 포트가 아니라 **IP**(및 ROS 네임스페이스)로 구분됩니다.
모든 센서는 동일한 기본 포트(30509/30510/30511)를 사용하며,
`multi_sensor.launch.py`는 센서별 포트 오프셋을 설정하지 않습니다.

---

## 14. 문제 해결 체크리스트

| 증상 | 확인 사항 |
|-----|---------|
| `ping` 실패 | 호스트 IP가 `192.168.10.x` 대역인가? 이더넷 연결됨? |
| 노드 시작하나 토픽 없음 | `sensor_ip` 올바름? 포트 30509 접근 가능? |
| 토픽은 있으나 메시지 없음 | `ros2 topic hz`가 0 Hz → 포트 30509–30511 방화벽 규칙 확인 |
| `RDI` 감지 배열이 항상 비어 있음 | FoV 내 물체 있음? `min_range`/`max_range`, `min_snr` 확인 |
| 노드 로그에 E2E 오류 | 시작 시 정상; 지속적이면 TCP로 전환 (`transport_mode:=tcp`) |
| RDI 없음, SHI/SPI 정상 | RDI 버퍼 오버런 → TCP 강제 사용 (512KB vs UDP 400KB) |
| 멀티 센서: 토픽 누락 | 각 `<pos>_ip` 접근 가능 여부와 네임스페이스가 서로 다른지 확인 |
| `colcon build` 실패 | `afi920_msgs` 먼저 빌드; `$ROS_DISTRO` 소싱 확인 |
| `host_ip:=auto`가 잘못된 NIC 선택 | `host_ip:=192.168.10.100`으로 명시 지정 |
| 높은 지연 / 패킷 손실 | TCP 권장; UDP 사용 시 스위치의 점보 프레임 설정 확인 |

---

## 빠른 참조 카드

```
빌드     → colcon build --packages-select afi920_msgs afi920_driver
런치     → ros2 launch afi920_driver afi920.launch.py sensor_ip:=<IP>
토픽     → /afi920/PointCloud2  /afi920/RDI  /afi920/SHI  /afi920/SPI
튜닝     → ros2 param set /afi920/afi920_driver min_range 1.0  (라이브)
멀티센서 → ros2 launch afi920_driver multi_sensor.launch.py front_ip:=... rear_ip:=...
검증     → python3 tools/live_data_check.py [IP] [--duration N]
디버그   → python3 tools/rdi_diag.py --sensor-ip <IP> [--tcp]
```

**다음으로 읽어볼 파일:**
- [src/afi920_driver/launch/afi920.launch.py](src/afi920_driver/launch/afi920.launch.py)
- [src/afi920_driver/launch/multi_sensor.launch.py](src/afi920_driver/launch/multi_sensor.launch.py)
- [src/afi920_msgs/msg/Detection.msg](src/afi920_msgs/msg/Detection.msg)
- [tools/live_data_check.py](tools/live_data_check.py)
