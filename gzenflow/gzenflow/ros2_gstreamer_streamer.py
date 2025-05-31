import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import yaml
from pathlib import Path
import gi
from ament_index_python.packages import get_package_share_directory
from gzenflow_interfaces.msg import NetworkState

gi.require_version('Gst', '1.0')
from gi.repository import Gst

def classify_bandwidth(bw_mbps: float) -> str:
    if bw_mbps > 40:
        return "EXCELLENT"
    elif bw_mbps > 20:
        return "GOOD"
    elif bw_mbps > 5:
        return "DEGRADED"
    elif bw_mbps > 1:
        return "CRITICAL"
    else:
        return "DISABLED"

def get_bitrate_for_state(min_bitrate: int, max_bitrate: int, state: str) -> int:
    scale = {
        "EXCELLENT": 1.0,
        "GOOD": 0.7,
        "DEGRADED": 0.4,
        "CRITICAL": 0.1,
        "DISABLED": 0.0
    }.get(state, 1.0)
    return int(min_bitrate + scale * (max_bitrate - min_bitrate))

class Ros2Gstreamer(Node):
    def __init__(self):
        super().__init__('ros2_gstreamer_streamer')

        self.config_path = (
            Path(get_package_share_directory('gzenflow'))
            / 'config'
            / 'config.yaml'
        )

        try:
            config = yaml.safe_load(self.config_path.read_text())
            self.get_logger().info(f"ROS2GSTREAMER: Config geladen von {self.config_path}")
        except Exception as e:
            self.get_logger().error(f"ROS2GSTREAMER: Fehler beim Laden der Config '{self.config_path}': {e}")
            return

        self.target_ip = config.get('global', {}).get('target_ip', '127.0.0.1')

        self.create_subscription(
            NetworkState,
            'network_state',
            self._network_callback,
            10
        )

        streams = config.get('streams', {})
        if not streams:
            self.get_logger().warning('ROS2GSTREAMER: Keine Streams in Konfiguration gefunden.')

        self.bridge = CvBridge()
        Gst.init(None)

        self.pipelines = {}

        for name, params in streams.items():
            if params.get('type') != 'ros2_gstreamer':
                continue

            topic       = params['topic']
            port        = params['port']
            width       = params.get('width', 640)
            height      = params.get('height', 480)
            framerate   = params.get('framerate', 30)
            min_bitrate = params.get('min_bitrate', 100)  # in kbps
            max_bitrate = params.get('max_bitrate', 1000) # in kbps

            # Anfangs-Bitrate = max
            init_bitrate = max_bitrate

            caps = f"video/x-raw,format=BGR,width={width},height={height},framerate={framerate}/1"

            pipeline_str = (
                f"appsrc name=src_{name} is-live=true block=true format=time caps={caps} "
                f"! videoconvert "
                f"! x264enc name=enc_{name} bitrate={init_bitrate} tune=zerolatency "
                f"! rtph264pay config-interval=1 pt=96 "
                f"! udpsink host={self.target_ip} port={port}"
            )

            pipeline = Gst.parse_launch(pipeline_str)
            pipeline.set_state(Gst.State.PLAYING)
            appsrc  = pipeline.get_by_name(f"src_{name}")
            encoder = pipeline.get_by_name(f"enc_{name}")

            # Pipeline-Info speichern
            self.pipelines[name] = {
                'pipeline': pipeline,
                'appsrc': appsrc,
                'encoder': encoder,
                'min_bitrate': min_bitrate,
                'max_bitrate': max_bitrate
            }

            # Subscriber auf Image-Topic
            self.create_subscription(
                Image,
                topic,
                lambda msg, src=appsrc: self._image_callback(msg, src),
                10
            )

            self.get_logger().info(
                f"ROS2GSTREAMER: Stream '{name}' auf Topic '{topic}' → {self.target_ip}:{port} "
                f"(Init-Bitrate: {init_bitrate} kbps)"
            )

    def _image_callback(self, msg: Image, appsrc):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            buf    = Gst.Buffer.new_wrapped(cv_img.tobytes())
            appsrc.emit('push-buffer', buf)
        except Exception as e:
            self.get_logger().error(f"ROS2GSTREAMER: Fehler im Image-Callback von '{appsrc.get_name()}': {e}")

    def _network_callback(self, state: NetworkState):
        # Zustand aus msg.state und msg.bandwidth_mbps bestimmen
        if state.state != 'OK':
            net_state = 'DISABLED'
        else:
            net_state = classify_bandwidth(state.bandwidth_mbps)

        self.get_logger().info(
            f"ROS2GSTREAMER: Netzwerkzustand: {net_state} ({state.bandwidth_mbps:.2f} Mbps)"
        )

        for name, info in self.pipelines.items():
            min_b = info['min_bitrate']
            max_b = info['max_bitrate']
            new_bitrate = get_bitrate_for_state(min_b, max_b, net_state)
            try:
                info['encoder'].set_property('bitrate', new_bitrate)
                self.get_logger().info(
                    f"ROS2GSTREAMER: Bitrate '{name}' angepasst: {new_bitrate} kbps"
                )
            except Exception as e:
                self.get_logger().error(
                    f"ROS2GSTREAMER: Fehler beim Setzen der Bitrate für '{name}': {e}"
                )


def main(args=None):
    rclpy.init(args=args)
    node = Ros2Gstreamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for info in node.pipelines.values():
            info['pipeline'].set_state(Gst.State.NULL)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

