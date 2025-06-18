# gzenflow

> ⚠️ **This package is still under development and not feature-complete.**
>
> * Zenoh Config changes in runtime not possible at the momen -> needed feature.
> * API and functionality may change without notice.

**Dynamic GStreamer & Zenoh Stream Control in ROS 2**

This package enables adaptive control of video and network streams in ROS 2 based on the current network condition. It supports:

* **GStreamer-based camera streams** (webcams, CSI cameras, Seek Thermal)
* **ROS 2 → GStreamer** image-topic streaming
* **Zenoh Bridge** for ROS 2 ↔ DDS communication

---

## 📦 Prerequisites

### System

* Ubuntu 22.04 or later (tested on 22.04)
* ROS 2 Humble (also compatible with Foxy, Galactic)

### System Packages

```bash
sudo apt update
sudo apt install -y \
  python3-opencv python3-gst-1.0 python3-yaml python3-gi \
  gir1.2-gstreamer-1.0 gir1.2-gst-plugins-base-1.0 \
  python3-zenoh iperf3 zenoh-bridge-ros2dds \
  ros-$(ros2 pkg prefix rclpy)/lib*/python3*
# see package.xml for additional ROS 2 dependencies
```

### Python Packages

```bash
pip3 install pyyaml psutil iperf3
```

### Optional: Seek Thermal GStreamer Plugin

To stream from a Seek Thermal camera, build and install the openseekthermal plugin:

```bash
git clone https://github.com/StefanFabian/openseekthermal.git
cd openseekthermal/openseekthermal_gstreamer
mkdir build && cd build
cmake .. && make && sudo make install
```

For details, see: [https://github.com/StefanFabian/openseekthermal](https://github.com/StefanFabian/openseekthermal)

---

## 🚀 Installation & Build

1. Clone this repo into your ROS 2 workspace:

   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/Schlizor/gzenflow.git
   ```
2. Build with colcon:

   ```bash
   cd ~/ros2_ws
   colcon build --packages-select gzenflow gzenflow_interfaces
   source install/setup.bash
   ```

---

## ⚙️ Configuration

Configuration is defined in `config/config.yaml`. A full example is provided at `config/example.yaml`:

### Parameter Reference

#### global

* **`target_ip`** (string): IP address for Zenoh Bridge and RTP sinks.

#### streams (mapping)

Defines one or more streams. Keys under `streams` are user-defined identifiers.

Common fields:

* **`type`** (string): Type of stream:

  * `gstreamer` → standard camera (`v4l2src`)
  * `thermal` → Seek Thermal camera (`openseekthermalsrc`)
  * `ros2_gstreamer` → ROS 2 image topic to GStreamer
  * `zenoh` → Zenoh Bridge publish topic
* **`port`** (int): UDP port for RTP stream.
* **`device`** (string, optional): device path, e.g. `/dev/video0`.
* \*\*`width`\*\*\*\*, \*\***`height`** (int, optional): resolution; if omitted, native camera resolution is used.
* \*\*`min_bitrate`\*\*\*\*, \*\***`max_bitrate`** (int): bitrate range in kbps.
* **`allowed_in_state`** (list\[string]): network states in which the stream runs.

  * `EXCELLENT`, `GOOD`, `DEGRADED`, `CRITICAL`, `DISABLED`, `ALL`

##### Options for `thermal`

* **`normalize`** (bool): normalize frame values (default: `true`).
* **`normalize-frame-count`** (int): number of frames for contrast normalization (default: `32`).
* **`skip-invalid-frames`** (bool): skip calibration frames (default: `true`).
* **`serial`** (string): camera serial number (optional, for multiple devices).

##### Options for `ros2_gstreamer`

* **`topic`** (string): ROS 2 image topic, e.g. `/camera/image_raw`.
* **`framerate`** (int): frames per second for pulling the topic.

---

## ▶️ Running

Use the provided ROS 2 launch file:

```bash
ros2 launch gzenflow flow.launch.py
```

Alternatively, start components manually:

1. **Network Manager**:

   ```bash
   ros2 run gzenflow network_manager
   ```
2. **Flow Controller (Bridge + GStreamer)**:

   ```bash
   ros2 run gzenflow bridge_controller
   ```
3. **ROS 2 → GStreamer** (optional):

   ```bash
   ros2 run gzenflow ros2_gstreamer_streamer
   ```

---

## Gscam2

Here is an example if you want to use gscam2 for a not thermal camera on the other pc: 

```bash
ros2 run gscam gscam_node \
  --ros-args \
  -p gscam_config:="udpsrc port=5005 caps=\"application/x-rtp, media=video, encoding-name=H264, payload=96\" ! \
                    rtph264depay ! avdec_h264 ! videoconvert ! video/x-raw,format=RGB" \
  -p image_encoding:=rgb8 \
  -p frame_id:=camera_frame \
  -p camera_name:=udp_cam \
  -p camera_info_url:=file:///home/thomas/.ros/camera_info/udp_cam.yaml \
  -p use_gst_timestamps:=true \
  -p preroll:=false \
  -p sync_sink:=false \
  -p use_intra_process:=false \
  -r /camera/image_raw:=/udp_cam/image_raw \
  -r /camera/camera_info:=/udp_cam/camera_info

```

## 🛠 Troubleshooting

* **Plugin not found** (`openseekthermalsrc`): Verify installation of the openseekthermal plugin; see [https://github.com/StefanFabian/openseekthermal](https://github.com/StefanFabian/openseekthermal)
* **Zenoh Bridge**: Install `zenoh-bridge-ros2dds`; see [https://github.com/eclipse-zenoh/zenoh-bridge-ros2dds](https://github.com/eclipse-zenoh/zenoh-bridge-ros2dds)
* **Permissions**: Check udev rules for `/dev/video*` devices.
* **Zenoh Bridge won't start**: Inspect `bridge_config.json5` for correct `target_ip`.
* **Missing ROS 2 dependencies**: Run `rosdep install --from-paths src --ignore-src -r -y`.

---

## 🤝 Credits

* **zenoh-bridge-ros2dds** (Eclipse Foundation) – [https://github.com/eclipse-zenoh/zenoh-bridge-ros2dds](https://github.com/eclipse-zenoh/zenoh-bridge-ros2dds)
* **openseekthermal** GStreamer plugin (Stefan Fabian) – [https://github.com/StefanFabian/openseekthermal](https://github.com/StefanFabian/openseekthermal)

