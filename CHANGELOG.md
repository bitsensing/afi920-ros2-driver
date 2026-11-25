# Changelog

All notable changes to the AFI920 ROS 2 Driver will be documented in this file.

---

## Unreleased

### Isaac ROS Point Cloud Topic

The driver can publish a point cloud that Isaac ROS NITROS point cloud nodes accept. It is off by default, and existing topics, message types and parameters are unchanged.

- **PointCloud2XYZI** — With the new parameter `publish_pointcloud_xyzi` (default `false`), the driver also publishes `<ns>/PointCloud2XYZI`: the points of `PointCloud2` with the fields `x`, `y`, `z`, `intensity` (FLOAT32, 16 B/point), the same header and the same filters. Frames without points are not published on it.
- **Padding** — The new parameter `xyzi_pad_multiple` (default 16) pads the point count of `PointCloud2XYZI` up to a multiple of its value with points at `x = y = 0`, `z = +inf`; `is_dense` is then `false`. `0` or `1` turns padding off. Isaac ROS 4.1 `PointCloudToFlatScanNode` processes only the first floor(N/16)×16 points.
- **afi920_isaac.launch.py** — Starts the driver with `PointCloud2XYZI` and the Isaac ROS `PointCloudToFlatScanNode` and `FlatScantoLaserScanNode` (`<ns>/flatscan`, `<ns>/scan`), with `input_qos: SENSOR_DATA` and z bounds of ±1000 m. Isaac ROS is needed at run time only; the package does not depend on it. Verified with Isaac ROS 4.1 on DGX Spark (Ubuntu 24.04, aarch64, Jazzy).
- **afi920.launch.py, config.yaml, default_params.yaml** — Gain `publish_pointcloud_xyzi` and `xyzi_pad_multiple`.

### Fixes

- **Sensor pose TF on /tf_static** — The mounting pose `parent_frame_id → frame_id` is published on `/tf_static` when the first SPI message arrives and whenever the pose or `frame_id` changes, instead of on `/tf` with every SPI message. The `/tf` transform carried the SPI reception time, up to one SPI period (100 ms) older than the `PointCloud2` stamps, so a transform at a `PointCloud2` stamp failed with an extrapolation error and RViz with Fixed Frame `base_link` alternated between OK and Error.

---

## v2.2.0

### Sensor Interface v1.2.0 Compatibility

The driver receives interface v1.2.0 sensors next to v1.0.x and v1.1.0, following AFI920 SDK v2.2.0. The RDI framing is detected from received frames, and no new parameter is required. Existing topics, message types and parameters are unchanged.

- **RDI header/chunk messages** — A v1.2.0 scan arrives as header A (Event `0x8002`), chunk messages (Events `0x8100`–`0x814B`) and header B, marked by SOME/IP Interface Version `0x02`, each with its own E2E header (Data ID `0x60008002` for the headers, `0x60008100`+k for chunk k) and no SOME/IP-TP. The driver assembles them by E2E Counter into one `PointCloud2` and one `RDI` message per scan, following Interface Overview §3.2.3: late messages of the last two closed scans are ignored, and a scan closes on completion, on header B, on another E2E Counter or 100 ms after its first message.
- **Partial scans** — A scan with missing chunk messages is published with the detections that arrived and logged as `partial`. A chunk whose size, MessageCounter or index does not match the header is rejected and logged. A scan without any header message is not published.
- **E2E topic** — On v1.2.0 one `E2eStatus` is published per RDI scan with the worst result of its messages. A message dropped by `e2e_strict` is reported on its own with `data_published` false, as is a scan without a header message. `E2eStatus.msg` gains comments only.
- **Loss detection** — The RDI E2E Counter is tracked once per scan.
- **RDI UDP receive timeout** — 20 ms, so the 100 ms scan timeout holds when the sensor stops sending.
- **rdi_diag.py** — Assembles v1.2.0 scans by E2E Counter and header/chunk Event ID and prints the chunk count, rejected chunks and messages that failed the E2E check.
- **Logging** — The `RDI recv` progress line is printed every 1000 datagrams instead of every 100.
- **RDI field values** — The message comments and the guides follow the interface v1.2.0 field definitions. The driver passes the values through unchanged. `velocity_ambiguity_begin` / `velocity_ambiguity_end` are computed per scan by the sensor (−17.21 to −12.05 m/s and 12.05 to 17.21 m/s), and `radial_velocity` is the resolved velocity, which can lie outside that domain. `existence_probability` is 1 to 100 (90–100 real, 70–89 suspected ghost, 50–69 likely ghost, 1–49 ghost; 98, 97, 95 and 93 mark range, angle, range and angle, and velocity shape-estimation points). `ambiguity_grouping_id` is always 0.

### Fixes

- **TCP RDI scan close** — The RDI TCP stream now waits at most 20 ms for the next message, as the UDP path does, so the 100 ms scan close also fires over TCP when the sensor stops sending. SHI and SPI are unchanged.
- **Chunk placement** — A v1.2.0 chunk whose `FirstEntityIndex` is not a multiple of EntitiesPerChunk, whose slot is not below ChunkCount, or whose EntityCount exceeds EntitiesPerChunk is rejected. When two chunks claim the same slot, the one that passed the E2E check is kept. Kept chunks are placed in `FirstEntityIndex` order (Interface Overview §3.2.3). The rejection warning names every reason.
- **Header length** — A v1.2.0 header body longer than 40 B is accepted and its first 40 B are used; only a body shorter than 40 B makes the scan invalid (Interface Overview §3.2.3).
- **Repeated header datagram** — A v1.2.0 header message with the same SOME/IP Session ID as a header already received for the scan is treated as a duplicate, not as header B. Over UDP, a header A delivered twice no longer closes the scan before its chunks arrive, which published a scan without detections while every E2E field reported success.
- **Single-message detection limit** — Interface v1.0.x/v1.1.0 single-message RDI is accepted up to 8192 detections (was 4096). In the vendored `bts_iso23150_rdi.h`, `BTS_RDI_MAX_DETECTIONS` is now 2048 (interface v1.2.0) and the new `BTS_RDI_LEGACY_MAX_DETECTIONS` is 8192.
- **TCP reconnect** — A TCP stream that cannot connect at startup now retries forever with backoff from 1 s to 30 s, like a stream that loses its connection; the node no longer shuts down when no TCP stream connects. Each attempt is logged. Shutdown no longer waits for the backoff. A UDP port that cannot be bound disables that stream; the node shuts down only when no UDP port can be bound.
- **Runtime parameter changes** — `ros2 param set` on a parameter that is read only at startup is now rejected with the reason `<name> cannot be changed at runtime; restart the node` instead of reporting success without effect.
- **Build on newer ROS 2** — The tf2 and tf2_ros headers are included as `.hpp` when available and as `.h` otherwise, so the driver builds on Foxy, Humble, Jazzy, Kilted and Lyrical (which no longer ships the `.h` headers). `CMakeLists.txt` links modern CMake targets instead of `ament_target_dependencies()`, which is deprecated from Kilted, and falls back to the classic variables for packages that export no target on Foxy. The node runtime test sets up rclcpp per test, so it also runs with the gtest 1.8 used on Foxy. `run.sh` sources the newest installed distribution first (lyrical, kilted, jazzy, humble, foxy) when `ROS_DISTRO` is not set.
- **Parameter files** — `config/config.yaml` and `config/default_params.yaml` use the `/**:` key, so `config_file:=` applies them in any namespace. With the previous `afi920_driver:` key they were not applied under the default namespace `afi920`.
- **Documentation** — Message comments, README and guides follow the interface v1.2.0 field definitions: SHII operation mode, defect reason and calibration state values, DataQualifier 6, fields that are always 0, fixed accuracy error fields, and `timestamp_difference` measured from `timestamp_measurement_ns`. Topic names, `host_ip`, filter scope, multi-sensor UDP ports, firmware versions per interface and troubleshooting entries are corrected. Field layouts are unchanged. Jazzy (Ubuntu 24.04, aarch64) is listed as a supported platform. The README states that topics use best-effort `SensorDataQoS`. Troubleshooting adds stream ports that differ from the defaults and the one-TCP-client-per-stream rule, and states that operation mode 20 (Diagnostic) alone does not stop RDI.

### Sensor Interface v1.1.0 Compatibility

The driver works with sensors on interface v1.0.x and v1.1.0+. The format is detected from received frames, and no new parameter is required.

- **E2E CRC** — RDI/SHI/SPI frames are verified with IEEE 802.3 CRC-32 over the payload (v1.1.0+) or CRC-64/XZ (v1.0.x). The detected format is logged once. A CRC field of 0 is never a pass and is reported as `result` 2 (`RESULT_CRC_ZERO`).
- **E2E CRC handling** — CRC mismatches are logged and frames are always published. `e2e_strict` now drops frames on E2E Length or Data ID mismatch only.
- **E2E topic** — New `E2E` topic (`afi920_msgs/E2eStatus`) carries the verification result of every RDI/SHI/SPI message, including frames that were dropped. Its `header.stamp` equals the stamp of the matching `PointCloud2`/`RDI`/`SHI`/`SPI` message. Existing topics and message types are unchanged.
- **Config API** — The driver sends `GetNetworkConfig` with the v1.1.0 E2E header and falls back to the v1.0.x frame when the sensor does not answer. v1.0.x sensors receive `SetNetworkConfig` as before. On v1.1.0+ sensors the driver compares the stream protocol, port and destination with its parameters and logs differences without changing the sensor.
- **Loss detection** — Message loss on RDI/SHI/SPI is counted from the E2E Counter. A counter decrease is logged as a sensor restart. MessageCounter is no longer used for loss detection because v1.1.0+ repeats it on resend.
- **DataQualifier** — RDI frames with DataQualifier 1 (not available) or 6 (resend of the previous scan) are excluded from `PointCloud2` and the detection count marker. They are still published on `RDI` with `data_qualifier`.
- **rdi_diag.py** — The E2E CRC is verified with the same rule as the driver (CRC-32 for v1.1.0+, CRC-64/XZ for v1.0.x), and the matched format is printed.

---

## v2.1.0

### Sensor Pose TF and Visualization

- **TF broadcast** — The sensor mounting pose from each SPI message is broadcast on `/tf` as `parent_frame_id → frame_id` (default `base_link → afi920`). New parameters `parent_frame_id` and `publish_tf` (default `true`), and new `multi_sensor.launch.py` arguments `parent_frame` and `publish_tf`. The pose is not broadcast when `publish_performance` is `false`. Set `publish_tf:=false` when the vehicle URDF or another node already publishes the sensor frames. New dependencies: `geometry_msgs`, `tf2`, `tf2_ros`.
- **Multi-sensor RViz configuration** — `config/multi_sensor.rviz` shows the four `/radar_<pos>/PointCloud2` topics with Fixed Frame `base_link`. `./run.sh --multi` opens it.
- **Integration guide** — `AFI920_ROS2_GUIDE.md` and `AFI920_ROS2_GUIDE_ko.md` describe every parameter, message definition, and protocol detail.

---

## v2.0.1 — Initial Release

First public release on GitHub.

### Features

- **18-field PointCloud2** — All fields FLOAT32, includes position/velocity/RCS/SNR/accuracy/probability/ID (Point stride: 72 bytes)
- **Full RDI Publish** — Publishes all ISO 23150 RDI fields as DetectionArray custom messages
- **Health Monitoring** — Real-time sensor health monitoring via SHII stream
- **Performance Monitoring** — Provides sensor pose, FOV blockage, and detection performance info via SPI stream
- **Multi-sensor** — Simultaneous operation of up to 4 sensors with namespace-based isolation
- **Dynamic Filtering** — Runtime dynamic adjustment of Range, SNR, and Existence Probability filters
- **Composable Node** — Zero-copy Composable Node with intra-process communication support
- **TCP/UDP Transport Mode** — Supports TCP (default) and UDP transport modes
- **SOME/IP-TP** — Segment reassembly support for large RDI frames
- **Enhanced Security** — Strengthened input validation

### Supported Platforms

- Ubuntu 20.04 (Foxy Fitzroy) — x86_64, aarch64
- Ubuntu 22.04 (Humble Hawksbill) — x86_64, aarch64

### Custom Messages (8)

- `Detection.msg`, `DetectionArray.msg` — RDI data
- `HealthInfo.msg` — SHII sensor health information
- `SensorPerformance.msg`, `SensorPose.msg`, `FovSegment.msg` — SPI performance/pose/FOV
- `RecognisableObject.msg`, `ReferenceTarget.msg` — SPI detection targets

### Tools

- `run.sh` — One-click convenience script for build/run/RViz2
- `afi920.rviz` — Default RViz2 configuration file
