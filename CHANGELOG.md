# Changelog

All notable changes to the AFI920 ROS 2 Driver will be documented in this file.

---

## v2.2.0

### Sensor Interface v1.1.0 Compatibility

The driver works with sensors on interface v1.0.x and v1.1.0+. The format is detected from received frames, and no new parameter is required.

- **E2E CRC** — RDI/SHI/SPI frames are verified with IEEE 802.3 CRC-32 over the payload (v1.1.0+) or CRC-64/XZ (v1.0.x). The detected format is logged once. A CRC field of 0 is reported as not verified.
- **E2E CRC handling** — CRC mismatches are logged and frames are always published. `e2e_strict` now drops frames on E2E Length or Data ID mismatch only.
- **E2E topic** — New `E2E` topic (`afi920_msgs/E2eStatus`) carries the verification result of every RDI/SHI/SPI message, including frames that were dropped. Its `header.stamp` equals the stamp of the matching `PointCloud2`/`RDI`/`SHI`/`SPI` message. Existing topics and message types are unchanged.
- **Config API** — The driver sends `GetNetworkConfig` with the v1.1.0 E2E header and falls back to the v1.0.x frame when the sensor does not answer. v1.0.x sensors receive `SetNetworkConfig` as before. On v1.1.0+ sensors the driver compares the stream protocol, port and destination with its parameters and logs differences without changing the sensor.
- **Loss detection** — Message loss on RDI/SHI/SPI is counted from the E2E Counter. A counter decrease is logged as a sensor restart. MessageCounter is no longer used for loss detection because v1.1.0+ repeats it on resend.
- **DataQualifier** — RDI frames with DataQualifier 1 (not available) or 6 (resend of the previous scan) are excluded from `PointCloud2` and the detection count marker. They are still published on `RDI` with `data_qualifier`.
- **rdi_diag.py** — The E2E CRC is verified with the same rule as the driver (CRC-32 for v1.1.0+, CRC-64/XZ for v1.0.x), and the matched format is printed.

### Sensor Pose TF and Visualization

- **TF broadcast** — The sensor mounting pose from each SPI message is broadcast on `/tf` as `parent_frame_id → frame_id` (default `base_link → afi920`). New parameters `parent_frame_id` and `publish_tf` (default `true`), and new `multi_sensor.launch.py` arguments `parent_frame` and `publish_tf`. The pose is not broadcast when `publish_performance` is `false`. Set `publish_tf:=false` when the vehicle URDF or another node already publishes the sensor frames. New dependencies: `geometry_msgs`, `tf2`, `tf2_ros`.
- **Multi-sensor RViz configuration** — `config/multi_sensor.rviz` shows the four `/radar_<pos>/PointCloud2` topics with Fixed Frame `base_link`. `./run.sh --multi` opens it.
- **Integration guide** — `AFI920_ROS2_GUIDE.md` and `AFI920_ROS2_GUIDE_ko.md` describe every parameter, message definition, and protocol detail.

---

## v2.0.1 — Initial Release

First public release on GitHub.

### Features

- **18-field PointCloud2** — All fields FLOAT32, includes position/velocity/RCS/SNR/variance/probability/ID (Point stride: 72 bytes)
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

### Custom Messages (12)

- `Detection.msg`, `DetectionArray.msg` — RDI data
- `HealthInfo.msg` — SHII sensor health information
- `SensorPerformance.msg`, `SensorPose.msg`, `FovSegment.msg` — SPI performance/pose/FOV
- `RecognisableObject.msg`, `ReferenceTarget.msg` — SPI detection targets

### Tools

- `run.sh` — One-click convenience script for build/run/RViz2
- `afi920.rviz` — Default RViz2 configuration file
