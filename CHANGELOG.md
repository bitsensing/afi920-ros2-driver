# Changelog

All notable changes to the AFI920 ROS 2 Driver will be documented in this file.

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
