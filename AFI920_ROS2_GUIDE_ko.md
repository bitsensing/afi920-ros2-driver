# AFI920 ROS 2 드라이버 — AI 통합 가이드

> **대상:** AI 에이전트, LLM 기반 자동화, bitsensing AFI920 4D 이미징 레이더를 ROS 2 시스템에 가장 빠르게 통합하려는 개발자  
> **드라이버 버전:** 2.2.0 · **지원 ROS 2:** Foxy (Ubuntu 20.04), Humble (Ubuntu 22.04) · **아키텍처:** x86_64, aarch64  
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

AFI920 ROS 2 드라이버는 SOME/IP, E2E, ISO 23150 수신을 자체 구현한 ROS 2 노드이며 AFI920 SDK에 의존하지 않습니다. 이더넷으로 레이더에 연결하여 ~10 Hz로 네 가지 데이터 토픽을 발행합니다:

| 제공 데이터 | 토픽 | 타입 |
|---|---|---|
| 표준 포인트 클라우드 (RViz 즉시 사용 가능) | `PointCloud2` | `sensor_msgs/PointCloud2` |
| 완전한 ISO 23150 감지 데이터 | `RDI` | `afi920_msgs/DetectionArray` |
| 센서 상태 & 장애 정보 | `SHI` | `afi920_msgs/HealthInfo` |
| 센서 성능 & 장착 포즈 | `SPI` | `afi920_msgs/SensorPerformance` |

토픽 이름은 노드 namespace 기준 상대 이름입니다(기본 `afi920`이므로 `PointCloud2`는 `/afi920/PointCloud2`입니다). 이 외에도 RViz 오버레이용 `detection_count` 마커와 RDI/SHI/SPI 메시지별 E2E 판정 결과 `E2E`(인터페이스 v1.2.0 RDI는 스캔당 1건)를 발행하고, SPI에서 받은 센서 장착 포즈를 `parent_frame_id → frame_id` TF 변환으로 브로드캐스트합니다. [§6](#6-발행-토픽) 참고.

모든 데이터 토픽은 `SensorDataQoS`를 사용합니다. TCP(기본값) 및 UDP 전송, 센서 인터페이스 v1.0.x와 v1.1.0 이후의 E2E CRC 검증(수신 프레임으로 형식 판별, 파라미터 없음), SOME/IP-TP 세그먼트 재조립, 인터페이스 v1.2.0 RDI 헤더·묶음 메시지의 스캔 조립, 센서 포즈 TF 브로드캐스트, `ros2 param set`을 통한 런타임 필터 조정을 지원합니다.

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
│   │       ├── ReferenceTarget.msg ← 참조 표적 특성
│   │       └── E2eStatus.msg       ← 메시지별 E2E 판정 결과
│   └── afi920_driver/
│       ├── src/afi920_node.cpp     ← 메인 컴포저블 노드
│       ├── config/                 ← 파라미터 YAML, afi920.rviz, multi_sensor.rviz, dds_profile.xml
│       └── launch/
│           ├── afi920.launch.py        ← 단일 센서
│           └── multi_sensor.launch.py  ← 센서 4개 (front, rear, left, right)
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
ros2 topic echo /afi920/E2E --once      # result 0 = OK, crc_format 1 = CRC-32, 2 = CRC-64/XZ
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
| `/afi920/E2E` | `afi920_msgs/E2eStatus` | SensorDataQoS | 30 Hz (RDI + SHI + SPI) |

`detection_count`는 RViz 오버레이용 `TEXT_VIEW_FACING` 마커(`Pts: <n>`)이며, `PointCloud2`와 함께 항상 발행합니다.

**DDS 프로파일:** 두 launch 파일은 노드 환경에 `FASTRTPS_DEFAULT_PROFILES_FILE`을 `config/dds_profile.xml`로 지정합니다. 이 프로파일은 Fast DDS 내장 transport(공유 메모리 포함)를 끄고, 송수신 버퍼 1 MB와 최대 메시지 65000 B인 UDPv4 transport 1개만 사용합니다. 따라서 같은 호스트의 구독자도 UDP loopback으로 수신합니다.

**DataQualifier:** DataQualifier 1(기록 미완료. 헤더 메시지만 오며 검출이 없고 헤더 필드는 직전 헤더를 반복합니다)과 6(직전 스캔 재송신) RDI 프레임은 새 측정값이 아닙니다. 이 프레임은 `PointCloud2`와 `detection_count`에서 제외하며, `RDI` 토픽에는 `data_qualifier`와 함께 발행합니다.

**E2E:** E2E 헤더를 가진 RDI/SHI/SPI 수신 메시지마다 `E2eStatus` 메시지 1건을 발행하며, `e2e_strict`로 폐기한 프레임도 포함합니다. 인터페이스 v1.2.0 RDI는 스캔당 1건을 발행하며, 이미 닫힌 스캔의 늦은 메시지는 버리고 보고하지 않습니다. `publish_health`나 `publish_performance`가 `false`이면 SHI나 SPI를 수신하지 않으므로 보고도 없습니다. `header.stamp`는 대응하는 `PointCloud2`/`RDI`/`SHI`/`SPI` 메시지의 stamp와 같으므로 두 메시지를 시각으로 정확히 짝지을 수 있습니다. [§9](#afi920_msgse2estatus--메시지별-e2e-판정) 참고.

**TF 브로드캐스트:** `publish_tf`가 `true`(기본값)이고 `publish_performance`가 `true`이면, 노드는 각 SPI 프레임에서 받은 센서 장착 포즈를 `/tf`(`tf2_msgs/TFMessage`)에 `parent_frame_id → frame_id` 변환(기본 `base_link → afi920`)으로 브로드캐스트합니다. 고정 프레임(fixed frame)을 `parent_frame_id`로 설정하면 RViz/소비자가 각 센서의 클라우드를 장착 위치에 배치할 수 있습니다. 이 자세는 auto-alignment 보정을 반영하지 않은 설정 장착 자세이며, 센서가 설정을 적용하기 전에는 모든 값이 0입니다. `publish_performance`가 `false`이면 SPI 포즈가 없으므로 TF는 브로드캐스트되지 않습니다(경고 로그 출력).

**좌표 규약:** 센서 프레임 FLU (X=전방, Y=좌측, Z=상방) — REP-103 준수

---

## 7. 전체 노드 파라미터

"동적 변경 불가" 파라미터는 기동 시 한 번 읽습니다. 실행 중 `ros2 param set`은 `cannot be changed at runtime` 사유로 거부되며, 변경하려면 노드를 재시작하세요.

### 연결 설정 (동적 변경 불가 — 런치 시 설정)

| 파라미터 | 타입 | 기본값 | 설명 |
|---------|------|-------|------|
| `sensor_ip` | string | `192.168.10.150` | 레이더 IP 주소 |
| `host_ip` | string | `auto` | 센서 스트림 목적지 호스트 IP (`auto` = 센서 경로에서 자동 탐지). 인터페이스 v1.0.x는 기동 시 `SetNetworkConfig`로 기록합니다. v1.1.0 이후는 `transport_mode`가 `udp`일 때 센서의 UDP 목적지와 비교합니다. 수신 소켓은 이 주소에 bind하지 않습니다 |
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
| `min_range` | double | `0.0` m | 이 거리 미만 감지 제거 (`PointCloud2`와 `RDI`에 적용) |
| `max_range` | double | `300.0` m | 이 거리 초과 감지 제거 (`PointCloud2`와 `RDI`에 적용) |
| `min_snr` | double | `0.0` dB | 이 SNR 미만 감지 제거 (`PointCloud2`와 `RDI`에 적용) |
| `min_existence_probability` | double | `0.0` % | 이 신뢰도 미만 감지 제거 (`PointCloud2`와 `RDI`에 적용) |

### E2E 무결성 (동적 변경 가능)

| 파라미터 | 타입 | 기본값 | 설명 |
|---------|------|-------|------|
| `validate_e2e` | bool | `true` | 모든 RDI/SHI/SPI 프레임의 E2E CRC 검증(인터페이스 v1.1.0 이후 CRC-32, v1.0.x CRC-64/XZ). 결과는 `E2E`로 발행하며, CRC 불일치는 로그로 남기고 프레임을 발행합니다 |
| `e2e_strict` | bool | `false` | `true`이면 E2E Length 또는 Data ID 불일치 프레임을 폐기합니다. CRC 불일치로는 폐기하지 않습니다 |

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

`multi_sensor.launch.py`는 항상 노드 4개(`front`, `rear`, `left`, `right`)를 각각 독립된 네임스페이스로 실행합니다. 센서가 없는 노드는 TCP 연결을 계속 재시도합니다([§14](#14-문제-해결-체크리스트) 참조). 센서가 4대보다 적으면 센서마다 `afi920.launch.py`를 실행하세요.

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
| `validate_e2e` | `true` | 데이터 스트림 E2E CRC 검증(결과는 `E2E`로 발행, 폐기 없음) |
| `e2e_strict` | `false` | E2E Length/Data ID 불일치 프레임 폐기 |
| `parent_frame` | `base_link` | 공통 부모 TF 프레임; 각 포즈가 `parent_frame → <pos>_frame`로 브로드캐스트됨 |
| `publish_tf` | `true` | 각 센서 장착 포즈(SPI)를 TF로 브로드캐스트 |
| `config_file` | `""` | YAML 설정 파일 경로(예: `config/multi_sensor.yaml`). 지정하면 노드는 이 파일의 파라미터만 사용하며, 다른 인자(`<pos>_ip`, `transport_mode` 등)는 적용하지 않습니다 |

`publish_detections`, `publish_health`, `publish_performance`는 이 런치 파일에서 모든 센서에 대해 `true`로 고정되어 있습니다(인자로 노출되지 않음). 이를 변경하려면 `config_file`을 사용하세요.

### 포트

`multi_sensor.launch.py`는 센서별 포트를 노출하거나 설정하지 **않습니다**. 포트 규칙은 전송 방식에 따라 다릅니다.

| 전송 방식 | 포트 규칙 |
|-----------|-----------|
| `tcp` | 모든 센서가 동일한 기본 포트(30509/30510/30511)를 그대로 쓸 수 있습니다. 각 노드는 자신의 센서 IP로 연결하며, 호스트 측은 임시 포트를 사용합니다. |
| `udp` | 센서마다 서로 다른 호스트 포트로 송신해야 **합니다**. 노드는 UDP 포트에서 모든 주소의 datagram을 수신하며 센서 IP로 거르지 않으므로, 두 센서가 같은 호스트 포트로 송신하면 수신 데이터가 섞입니다. |

UDP에서는 각 레이더의 스트림 목적지 포트를 설정(진단 모드, `SetNetworkConfig`, 재시작)한 뒤, 센서마다 `transport_mode: "udp"`와 일치하는 `data_port`/`shi_port`/`spi_port`를 담은 `config_file`을 전달하세요. `config_file`을 지정하면 `transport_mode` launch 인자는 적용되지 않습니다. 예시는 다음과 같으며 포트 값은 설명용입니다.

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

## 9. 커스텀 메시지 정의

### `afi920_msgs/DetectionArray` — RDI 프레임

```
std_msgs/Header header
uint8  sensor_id
uint32 message_counter
uint64 timestamp_measurement_ns      # 센서 PTP 타임스탬프 (나노초)
uint8  data_qualifier                # 0=normal 1=not_available (기록 미완료. 검출 없음,
                                     # 헤더 필드는 직전 헤더 반복) 6=invalid (재송신)
                                     # 2–5는 송출하지 않습니다. 인터페이스 v1.0.x는 5=test_mode를 송출할 수 있습니다

float32 velocity_ambiguity_begin     # m/s, 센서가 스캔마다 산출 (-17.21 ~ -12.05)
float32 velocity_ambiguity_end       # m/s, 센서가 스캔마다 산출 (12.05 ~ 17.21);
                                     # Detection.radial_velocity는 해소된 속도이며 이 범위를 벗어날 수 있습니다
float32 range_ambiguity_begin        # m    (항상 0, wire에 없음)
float32 range_ambiguity_end          # m    (항상 0)
float32 azimuth_ambiguity_begin      # rad  (항상 0)
float32 azimuth_ambiguity_end        # rad  (항상 0)
float32 elevation_ambiguity_begin    # rad  (항상 0)
float32 elevation_ambiguity_end      # rad  (항상 0)

uint16 max_detections                # RecognisedDetectionsCapability (인터페이스 v1.2.0은 2048)
uint8  recognised_detections_status  # 레거시, 항상 0
afi920_msgs/Detection[] detections   # 드라이버 필터를 통과한 검출
```

### `afi920_msgs/Detection` — 단일 감지

```
# 식별
uint16  detection_id
uint16  object_id_reference    # 항상 0 (추적 없음)
uint8   existence_probability  # 1–100 %: 90–100 실검출, 70–89 고스트 의심, 50–69 고스트 가능성 높음,
                               # 1–49 고스트. 98/97/95/93은 거리/각도/거리+각도/속도 형상추정점
float32 timestamp_difference   # s, timestamp_measurement_ns에서 해당 검출 측정 구간
                               # 중앙까지의 시간 (header.stamp 기준이 아님)

# 위치 (구면 좌표, 센서 수신값)
float32 radial_distance        # m (설계 범위 0–300, 포화하지 않음)
float32 azimuth                # rad (설계 범위 ±60°. ±61°까지의 검출이 나올 수 있음)
float32 elevation              # rad (설계 범위 ±15°. ±16°까지의 검출이 나올 수 있음)
float32 radial_distance_error  # m, Range Mode와 측정 구간별 고정 정확도 (0.025–0.35)
float32 azimuth_error          # rad, 고정 정확도 0.001745 (0.1°)
float32 elevation_error        # rad, 고정 정확도 0.001745 (0.1°)

# 위치 (구면→데카르트 변환값; X=전방, Y=좌측, Z=상방)
float32 x                      # m
float32 y                      # m
float32 z                      # m

# 속도
float32 radial_velocity        # m/s (음수 = 접근)
float32 radial_velocity_error  # m/s, 고정 정확도 0.065

# 신호 품질
float32 rcs                    # dBsm (레이더 단면적, -20 ~ 40)
float32 rcs_error              # 레거시, 항상 0
float32 snr                    # dB (0 ~ 254)
float32 snr_error              # 레거시, 항상 0

# 확률
uint8   multi_target_probability        # 레거시, 항상 0
uint16  ambiguity_grouping_id           # 항상 0 (미지원)
uint8   detection_ambiguity_probability # 레거시, 항상 0
uint8   free_space_probability          # 레거시, 항상 0

# 분류 (레거시, 항상 0)
uint8   num_classifications
uint8[8] classification_type   # 0=none 1=noise 2=obstacle 3=under 4=over 5=nearest 6=strongest
uint8[8] classification_confidence  # 슬롯별 0–100

# 디버그 필드 (레거시, 항상 0. wire에 없음)
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
uint8  data_qualifier          # 0=normal 6=invalid (재송신)

# 동작 모드 (현재 모드. 센서는 1개를 송신하며 배열 용량은 11)
uint8   num_operation_modes
uint8[11] operation_modes
# 값: 0=normal_dual_range  1=normal_long_range  2=normal_middle_range
#     3=normal_short_range 4=normal_ultra_long_range  5=normal_ultra_short_range
#     6=normal_ultra_dual_range  10=degradation  20=diagnostic  21=programming
#     100=fault  200=initialising

# 장애 상태
uint8 defect_recognised        # 0=fully_functional  1=not_fully (2=out_of_order는 v1.2.0에서 송출하지 않음)
uint8 defect_reason            # 0=none  1=supply_voltage  2=hardware  3=thermal  5=calibration
                               # 7=mechanical  10=out_of_time_sync  11=externally_disturbed

# 전원 & 온도
uint8 supply_voltage_status    # 0=low  1=pre_low  2=ok  3=pre_high  4=high
uint8 temperature_status       # 0=under  1=pre_under  2=ok  3=pre_over  4=over

# 입력 신호 상태 (센서는 4개를 송신)
uint8   num_input_signal_statuses
uint8[4] input_signal_types     # 0=csii_sensor_operation 1=environmental_information
                                # 2=vehicle_state 3=sensor_pose
uint8[4] input_signal_statuses  # 0=valid 1=implausible 2=missing 3=out_of_range 4=timeout

# 시간 동기화
uint8   time_sync_status       # 0=ok  1=out_of_limits  2=timeout  3=not_synchronized
float32 time_sync_offset       # 레거시 필드, 항상 0.0

# 캘리브레이션 (병렬 배열, 센서는 3개 구성요소를 송신)
uint8   num_calibration_components
uint8[3] calibration_components # 0=intrinsic  1=extrinsic_horizontal  2=extrinsic_vertical
uint8[3] calibration_statuses   # 0=calibrated  1=not_calibrated  2=degraded
uint8[3] calibration_states     # 0=completed 1=not_performed 2=in_progress 3=failed
```

### `afi920_msgs/SensorPerformance` — SPI 프레임

```
std_msgs/Header header
uint8  sensor_id
uint32 message_counter
uint64 timestamp_measurement_ns
uint8  data_qualifier                # 0=normal 6=invalid (재송신)
uint8  vehicle_coordinate_system     # 1=road_level (항상. 0=rear_axle은 송출하지 않음)

afi920_msgs/SensorPose sensor_pose
afi920_msgs/FovSegment[]        fov_segments           # DUAL(0)/UDR(6)은 2개([0] 좁은 LR, [1] 넓은 SR),
                                                        # 그 외 모드 1개, 설정 적용 전 0개
afi920_msgs/RecognisableObject[] recognisable_objects   # 센서는 0개를 송신
afi920_msgs/ReferenceTarget[]    reference_targets      # 센서는 0개를 송신
```

### `afi920_msgs/SensorPose`

```
float32 origin_x / origin_y / origin_z          # 장착 위치 (m)
float32 origin_error_x / origin_error_y / origin_error_z  # 레거시, 항상 0
float32 yaw / pitch / roll                       # 설정 장착 방향 (rad),
                                                 # auto-alignment 보정 미반영
float32 yaw_error / pitch_error                  # auto-alignment 오정렬 추정각 (rad),
                                                 # ±3°(±0.0524 rad)로 제한
float32 roll_error                               # 레거시, 항상 0
```

### `afi920_msgs/FovSegment`

```
float32 azimuth_begin / azimuth_end     # rad
float32 elevation_begin / elevation_end # rad
uint8   blockage_status                 # 0=none  1=partial  2=full  (3=unknown은 v1.2.0에서 송출하지 않음)
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

### `afi920_msgs/E2eStatus` — 메시지별 E2E 판정

```
std_msgs/Header header        # stamp = 대응 데이터 메시지의 stamp
                              # (data_published가 false이면 판정 시각)
uint8  stream                 # 0=RDI  1=SHI  2=SPI
uint8  result                 # 0=ok  1=crc_mismatch  2=crc_zero (통과 아님)  3=not_checked
uint8  crc_format             # 0=unknown  1=CRC-32 (v1.1.0 이후)  2=CRC-64/XZ (v1.0.x);
                              # v1.2.0 스캔은 처음 통과한 메시지 기준
bool   length_ok              # E2E Length가 페이로드 크기와 같음
bool   data_id_ok             # E2E Data ID가 스트림과 일치
uint32 e2e_counter
uint32 missing_before         # 직전 메시지 이후 건너뛴 E2E Counter 개수
bool   sensor_restarted       # E2E Counter 감소 (v1.0.x는 TCP 재연결 시에도 설정)
bool   data_published         # false: 데이터 메시지 미발행 (e2e_strict 폐기, 파싱 실패,
                              # 헤더 없는 스캔, 해당 프레임의 발행 대상 없음)
uint32 message_counter        # ISO 23150 MessageCounter (파싱 전 0)
uint8  data_qualifier         # ISO 23150 DataQualifier (파싱 전 0)
```

메시지 손실은 `missing_before`로 계수합니다. 이 값은 센서가 송신하지 않은 주기(헤더 B를 보내지 못한 주기의 다음 주기, Fault Mode나 access mode 정지로 보류한 주기)도 포함하므로 손실 메시지 수의 상한입니다. 인터페이스 v1.2.0에서는 건너뛴 스캔 수를 계수하며, 스캔 안에서 잃은 묶음은 `partial` 경고로 나타납니다. 인터페이스 v1.1.0 이후 센서는 재송신 시 MessageCounter를 반복하므로 MessageCounter를 손실 판정에 사용하지 않습니다. 인터페이스 v1.0.x는 TCP 연결마다 E2E Counter가 다시 시작하므로 재연결 시 `sensor_restarted`가 설정됩니다.

인터페이스 v1.2.0에서는 한 스캔의 헤더와 묶음 메시지가 E2E Counter 하나를 공유하므로 `E2eStatus`를 RDI 스캔당 1건 발행합니다. `result`는 구성 메시지 판정 중 최악 값(CRC 불일치, CRC 0, 정상 순)이며, 검사에 실패한 메시지가 하나라도 있으면 `length_ok`와 `data_id_ok`는 false입니다. `e2e_strict`로 폐기한 메시지는 `data_published`가 false인 개별 메시지로 발행합니다. 헤더 메시지를 하나도 받지 못한 스캔은 `data_published`가 false이고 `message_counter`가 0인 메시지로 발행합니다.

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
| 10 | `radial_distance_error` | m | Range Mode와 측정 구간별 고정 정확도 |
| 11 | `azimuth_angle_error` | rad | 고정 정확도 0.001745 |
| 12 | `elevation_angle_error` | rad | 고정 정확도 0.001745 |
| 13 | `radial_velocity_error` | m/s | 고정 정확도 0.065 |
| 14 | `existence_probability` | % | 1–100 감지 신뢰도 (90–100 실검출, 98·97·95·93은 형상추정점) |
| 15 | `multi_target_probability` | % | 레거시, 항상 0 |
| 16 | `detection_id` | — | 프레임 내 고유 ID |
| 17 | `sensor_id` | — | 소스 센서 식별자 |
| 18 | `intensity` | dBsm | = `radar_cross_section`; RViz 컬러맵용 |

---

## 11. 동적 파라미터 튜닝

아래 파라미터는 노드 실행 중에도 변경할 수 있으며 재시작이 필요 없습니다. 그 밖의 파라미터를 실행 중에 설정하면 거부됩니다(`cannot be changed at runtime`).

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

**YAML로 저장 후 재사용** (Foxy의 `ros2 param dump`는 기본으로 파일을 쓰므로 표준 출력으로 받으려면 `--print`를 붙이세요):
```bash
ros2 param dump /afi920/afi920_driver > afi920_params.yaml
# 런치 시 적용:
ros2 launch afi920_driver afi920.launch.py config_file:=afi920_params.yaml
```

---

## 12. 진단 도구

### `live_data_check.py` — 엔드투엔드 센서 검증

하나 이상의 센서에 대해 연결 및 데이터 흐름 전체를 시험합니다(인터페이스 v1.1.0, v1.2.0 대상. SDK 시험은 v1.0.x를 다루지 않습니다).

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

# SDK 경로 직접 지정 (탐색 순서: --sdk-root, AFI920_SDK_ROOT, 저장소와 나란한 afi920_sdk.
# --sdk-root는 현재 디렉터리 기준입니다)
python3 live_data_check.py --sdk-root ../../afi920_sdk

# Discovery broadcast에 쓸 로컬 IP, SDK 시험에 쓸 Python 지정
python3 live_data_check.py --bind 192.168.10.100 --python python3
```

테스트 항목: Discovery (UDP 30520) · Config API · RDI/SHII/SPI 스트림 · SOME/IP + E2E 무결성 · CSII 차량 입력 (UDP 30501)

### `rdi_diag.py` — 바이트 레벨 RDI 프로토콜 디버거

원시 프레임을 캡처하여 SOME/IP 프레이밍, E2E CRC(드라이버와 같은 규칙으로 CRC-32 또는 CRC-64/XZ), RDI 헤더/감지 필드를 검증합니다.

```bash
# UDP (기본값. --port로 모든 송신원의 datagram을 수신하며 --sensor-ip는 쓰지 않습니다)
python3 rdi_diag.py --port 30509 --max-frames 5

# TCP
python3 rdi_diag.py --sensor-ip 192.168.10.150 --tcp --max-frames 5
```

출력 내용: SOME/IP 헤더 · E2E CRC 결과와 일치 형식 · RDI 헤더 필드 · 처음 10개 감지 데이터 및 NaN 진단

---

## 13. 포트 & 프로토콜 레퍼런스

| 포트 | 프로토콜 | 방향 | 용도 |
|-----|---------|------|------|
| 30500 | TCP | ↔ | Config API (센서 IP, 스트림 목적지) |
| 30509 | TCP/UDP | ← | RDI — 포인트 클라우드 (Event ID `0x8002`, 인터페이스 v1.2.0은 묶음 Event `0x8100`–`0x814B` 추가) |
| 30510 | TCP/UDP | ← | SHII — 상태 정보 (Event ID `0x8003`) |
| 30511 | TCP/UDP | ← | SPI — 성능 정보 (Event ID `0x8004`) |
| 30501 | UDP | → | CSII 차량 입력 (`live_data_check.py` 전용, v1.1.0 이후 UDP) |
| 30520 | UDP | ↔ | Discovery (`live_data_check.py` 전용) |

**수신 버퍼 크기** (TCP는 애플리케이션 버퍼이자 커널 `SO_RCVBUF` 값입니다. UDP는 datagram 1건 단위 애플리케이션 버퍼이며, 모든 UDP 소켓은 커널 버퍼 16 MB를 별도로 요청합니다):
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
E2E 헤더 (20B · 빅 엔디안 · CRC / Length / Counter / Data ID)
    ↑
SOME/IP 헤더 (16B · 빅 엔디안, 분할된 단일 메시지 RDI는 4B TP 헤더 추가)
    ↑
TCP / UDP
```

**센서 인터페이스 호환:** 형식은 수신 프레임으로 판별하며, 형식을 지정하는 파라미터는 없습니다.

| 항목 | 인터페이스 v1.0.x | 인터페이스 v1.1.0 | 인터페이스 v1.2.0 |
|------|------------------|------------------|------------------|
| 센서 펌웨어 | v3.0.0 – v3.1.0 | v3.1.1 | v3.1.2 |
| 스캔당 RDI 검출 수 | Capability 4096 또는 8192 (드라이버는 8192까지 수용) | 2048 | 2048 (EntitiesPerChunk 27, 묶음 최대 76개) |
| E2E CRC 필드 (0–7 바이트) | 헤더 8–19 바이트와 페이로드 대상 CRC-64/XZ | 0–3 바이트는 0, 4–7 바이트는 페이로드 대상 IEEE 802.3 CRC-32 | v1.1.0과 같습니다 |
| RDI 프레이밍 | 스캔당 Event `0x8002` 메시지 1건, UDP는 SOME/IP-TP 분할 | v1.0.x와 같습니다 | SOME/IP Interface Version `0x02`: 헤더 A(`0x8002`), 묶음 k(`0x8100`+k, k = 0–75), 헤더 B(`0x8002`). SOME/IP-TP를 쓰지 않습니다 |
| RDI E2E Data ID | `0x60008002` | `0x60008002` | 헤더 2건은 `0x60008002`, 묶음 k는 `0x60008100`+k |
| Config API 프레임 | SOME/IP 헤더 + JSON | SOME/IP 헤더 + E2E 헤더 + JSON | v1.1.0과 같습니다 |
| 기동 시 스트림 목적지 | 노드 기동 시 한 번 `SetNetworkConfig`로 적용 (노드보다 늦게 켜진 센서는 노드를 재시작해야 합니다) | `GetNetworkConfig`로 조회. 차이는 경고로 출력하며 센서 설정을 변경하지 않습니다 | v1.1.0과 같습니다 |
| 의미가 다른 필드 | `existence_probability` 표시값 100/60/40. DataQualifier 0/5/6. SHII operation mode `150`이 나올 수 있음. 입력 신호 상태 1개. E2E Counter는 스트림별이며 TCP 연결마다 1부터 시작 | `existence_probability`는 확률 × 100(보강 검출 95/90). DataQualifier 0/1/6. 입력 신호 상태 4개 | §9 참조 |
| 메시지 손실 판정 | E2E Counter | E2E Counter | E2E Counter (한 스캔의 메시지가 공유) |
| `PointCloud2` 제외 프레임 | DataQualifier 1, 6 (5 = test mode는 발행) | DataQualifier 1, 6 | DataQualifier 1, 6 (헤더 메시지 기준) |

인터페이스 v1.1.0 이후에서 스트림 프로토콜, 포트, 목적지를 변경하려면 진단 모드 진입, `SetNetworkConfig`, 센서 재시작이 필요합니다. 드라이버는 이 절차를 수행하지 않습니다.

**인터페이스 v1.2.0 RDI 스캔 조립** (Interface Overview §3.2.3):

| 규칙 | 드라이버 동작 |
|------|--------------|
| 헤더 본문 | Detection Header(36B) + List Layout(EntitiesPerChunk u16, ChunkCount u16) |
| 묶음 본문 | Chunk Header(MessageCounter u32, FirstEntityIndex u16, EntityCount u16) + EntityCount × 51B 검출 |
| 스캔 키 | E2E Counter. 다른 E2E Counter의 메시지를 받으면 열린 스캔을 닫습니다 |
| 헤더 A / 헤더 B | 스캔의 첫 `0x8002`는 헤더 A, 두 번째는 헤더 B입니다. E2E 검사를 통과한 헤더 사본을 우선 사용합니다 |
| 스캔 종료 | 헤더 1건과 ChunkCount건의 묶음 수신, 헤더 B 수신, 다른 E2E Counter 수신, 첫 메시지 후 100 ms 경과 중 먼저 오는 시점 (TCP와 UDP 모두 RDI 수신 타임아웃 20 ms로 보장합니다. TCP 재연결 중에는 재연결 완료까지 종료가 지연됩니다) |
| 늦은 메시지 | 직전에 닫힌 스캔 2건의 메시지는 버립니다 |
| 무효 스캔 | EntitiesPerChunk가 0이거나 ChunkCount가 76을 넘거나 본문이 40B 미만인 헤더는 수락하지 않으며, 그 스캔은 헤더 없는 스캔으로 처리합니다. 헤더 본문의 처음 40B 뒤 바이트는 무시합니다 |
| 묶음 검사 | 크기가 EntityCount와 맞지 않거나, EntityCount가 EntitiesPerChunk를 넘거나, MessageCounter가 헤더와 다르거나, 인덱스 또는 슬롯(FirstEntityIndex / EntitiesPerChunk)이 ChunkCount 이상이거나, FirstEntityIndex가 EntitiesPerChunk의 배수가 아닌 묶음은 제외하고 로그를 출력합니다. 두 묶음이 같은 슬롯을 가리키면 E2E 검사를 통과한 묶음을 남깁니다 |
| 부분 스캔 | 수신한 검출을 FirstEntityIndex 순서대로 `PointCloud2`와 `RDI`로 발행하고 `partial` 경고를 출력합니다. 센서는 94 ms 송신 마감 안에 보내지 못한 묶음을 TCP와 UDP 모두에서 생략합니다 |
| 헤더 없는 스캔 | `PointCloud2`와 `RDI`로 발행하지 않으며, `data_published`가 false인 `E2E` 메시지로 보고합니다 |

**멀티 센서:** TCP에서는 센서를 **IP**(및 ROS 네임스페이스)로 구분하며, 모든 센서가
동일한 기본 포트(30509/30510/30511)를 사용할 수 있습니다. UDP에서는 센서마다 서로 다른 호스트
포트를 사용해야 합니다([§8 포트](#포트) 참조).
`multi_sensor.launch.py`는 센서별 포트 오프셋을 설정하지 않습니다.

---

## 14. 문제 해결 체크리스트

| 증상 | 확인 사항 |
|-----|---------|
| `ping` 실패 | 호스트 IP가 `192.168.10.x` 대역인가? 이더넷 연결됨? |
| 노드 시작하나 토픽 없음 | `sensor_ip` 올바름? 포트 30509 접근 가능? TCP에서 노드는 1 s부터 30 s까지 늘어나는 간격으로 연결을 계속 재시도하며 실패할 때마다 로그를 출력합니다 |
| 토픽은 있으나 메시지 없음 | `ros2 topic hz`가 0 Hz → 포트 30509–30511 방화벽 규칙 확인 |
| `RDI` 감지 배열이 항상 비어 있음 | FoV 내 물체 있음? `min_range`/`max_range`, `min_snr` 확인 |
| 노드 로그에 E2E CRC 불일치 경고 | 프레임은 계속 발행됩니다. `ros2 topic echo /afi920/E2E`의 `result`, `crc_format`을 확인하고, UDP에서 지속되면 TCP로 전환하세요 (`transport_mode:=tcp`) |
| `Sensor stream config differs from driver parameters` 경고 | 인터페이스 v1.1.0 이후 센서의 스트림 프로토콜, 포트, 또는 (UDP) 목적지 IP가 `host_ip`와 다름 → `transport_mode`와 포트 파라미터를 센서에 맞추거나, 센서를 재설정한 뒤 재시작하세요 |
| Fixed Frame `base_link`에서 RViz 점군 미표시 | 센서 frame으로 가는 TF 없음 → `publish_tf`와 `publish_performance`를 `true`로 유지하거나 변환을 직접 송출하세요 |
| 센서 frame이 두 자세 사이를 오감 | 드라이버와 URDF 또는 다른 노드가 같은 변환을 송출함 → `publish_tf:=false`를 지정하세요 |
| RDI 없음, SHI/SPI 정상 | `SHI`의 `operation_modes[0]`이 100(Fault Mode)이거나, 하드웨어 또는 보정 결함 조건에서 20/200이면 센서는 조건이 유지되는 동안 RDI를 송신하지 않습니다 → `defect_reason`을 확인하세요. 보정 CRC 실패는 부팅 동안 유지되며, 보정 레코드를 다시 기록(0x00A6)하고 센서를 재시작해야 해소됩니다 |
| `RDI scan ... partial` 경고 | 인터페이스 v1.2.0 묶음 메시지 누락 → 스캔은 수신한 검출만으로 발행됩니다. UDP에서는 네트워크 손실이 원인일 수 있습니다(`transport_mode:=tcp` 시도). 두 전송 방식 모두에서 센서는 송신 마감 안에 보내지 못한 묶음을 생략합니다 |
| 멀티 센서: 토픽 누락 | 각 `<pos>_ip` 접근 가능 여부와 네임스페이스가 서로 다른지 확인하세요. 센서가 없는 노드는 재시도를 계속하므로, 보유한 센서만 `afi920.launch.py`로 실행하세요 |
| `colcon build` 실패 | `afi920_msgs` 먼저 빌드; `$ROS_DISTRO` 소싱 확인 |
| `host_ip:=auto`가 잘못된 NIC 선택 | `host_ip:=192.168.10.100`으로 명시 지정 |
| 높은 지연 / 패킷 손실 | TCP를 권장합니다. 모든 UDP 메시지가 1,472B 이하이므로 점보 프레임은 필요 없습니다(v1.0.x/v1.1.0 RDI는 SOME/IP-TP로 분할) |

---

## 빠른 참조 카드

```
빌드     → colcon build --packages-select afi920_msgs afi920_driver
런치     → ros2 launch afi920_driver afi920.launch.py sensor_ip:=<IP>
토픽     → /afi920/PointCloud2  /afi920/RDI  /afi920/SHI  /afi920/SPI  /afi920/E2E
튜닝     → ros2 param set /afi920/afi920_driver min_range 1.0  (라이브)
멀티센서 → ros2 launch afi920_driver multi_sensor.launch.py front_ip:=... rear_ip:=...
검증     → python3 tools/live_data_check.py [IP] [--duration N]
디버그   → python3 tools/rdi_diag.py [--port 30509] | --tcp --sensor-ip <IP>
```

**다음으로 읽어볼 파일:**
- [src/afi920_driver/launch/afi920.launch.py](src/afi920_driver/launch/afi920.launch.py)
- [src/afi920_driver/launch/multi_sensor.launch.py](src/afi920_driver/launch/multi_sensor.launch.py)
- [src/afi920_msgs/msg/Detection.msg](src/afi920_msgs/msg/Detection.msg)
- [tools/live_data_check.py](tools/live_data_check.py)
