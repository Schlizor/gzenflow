import rclpy
from rclpy.node import Node
from pathlib import Path
from std_msgs.msg import String
from gzenflow_interfaces.msg import NetworkState
from gzenflow.zenoh_config_builder import generate_zenoh_bridge_config
from gzenflow.gstreamer_streamer import GStreamerStreamer
from gzenflow.bridge_manager import BridgeManager
from ament_index_python.packages import get_package_share_directory
import yaml

def classify_bandwidth(bw_mbps):
    if bw_mbps > 50:
        return "EXCELLENT"
    elif bw_mbps > 30:
        return "GOOD"
    elif bw_mbps > 10:
        return "DEGRADED"
    elif bw_mbps > 3:
        return "CRITICAL"
    else:
        return "DISABLED"

def get_bitrate_for_state(min_bitrate, max_bitrate, state):
    scale = {
        "EXCELLENT": 1.0,
        "GOOD": 0.7,
        "DEGRADED": 0.4,
        "CRITICAL": 0.1,
        "DISABLED": 0.0
    }.get(state, 1.0)
    return int(min_bitrate + scale * (max_bitrate - min_bitrate))

class FlowController(Node):
    def __init__(self):
        super().__init__('bridge_controller')

        pkg_share = get_package_share_directory('gzenflow')
        config_dir = Path(pkg_share) / 'config'
        self.config_path = config_dir / 'config.yaml'
        self.output_path = config_dir / 'bridge_config.json5'

        self.bridge = BridgeManager(self.get_logger(), self.config_path, self.output_path)
        self.gstreamer = GStreamerStreamer(self.config_path, self.get_logger())

        self.get_logger().info("Warte auf Netzwerkdaten...")
        self.subscription = self.create_subscription(
            NetworkState,
            'network_state',
            self.network_callback,
            10
        )
        #self.bridge.start_bridge()
        self.bridge.generate_and_start_bridge("EXCELLENT")
        
    def network_callback(self, msg: NetworkState):
        if msg.state != "OK":
            self.get_logger().warn("WARNUNG: Netzwerkzustand nicht OK oder unbekannt! Verwende DISABLED Zustand")
            current_state = "DISABLED"
        else:
            current_state = classify_bandwidth(msg.bandwidth_mbps)

        self.get_logger().info(f"🔄 Netzwerkzustand: {current_state} ({msg.bandwidth_mbps:.2f} Mbit/s)")

        #self.bridge.generate_and_restart_bridge(current_state)

        with open(self.config_path, 'r') as f:
            config = yaml.safe_load(f)

        for name, stream in config.get('streams', {}).items():
            stype = stream.get('type')
            # Unterstütze sowohl normale GStreamer- als auch Thermal-Streams
            if stype not in ('gstreamer', 'thermal'):
                continue

            allowed = stream.get('allowed_in_state', [])
            if "ALL" not in allowed and current_state not in allowed:
                # Stream nicht erlaubt, stoppe ihn
                self.gstreamer.stop_stream(name)
                continue

            min_br = stream.get('min_bitrate', 1000)
            max_br = stream.get('max_bitrate', 4000)
            bitrate = get_bitrate_for_state(min_br, max_br, current_state)

            device = stream.get('device', '/dev/video0')
            port = stream.get('port')

            if stype == 'gstreamer':
                self.gstreamer.start_or_update_stream(
                    name, port, device, bitrate,
                    stream_type='gstreamer'
                )
            else:
                thermal_kwargs = {
                    'normalize': stream.get('normalize', True),
                    'normalize-frame-count': stream.get('normalize-frame-count', 32),
                    'skip-invalid-frames': stream.get('skip-invalid-frames', True),
                    'serial': stream.get('serial', '')
                }
                self.gstreamer.start_or_update_stream(
                    name, port, device, bitrate,
                    stream_type='thermal',
                    **thermal_kwargs
                )

def main(args=None):
    rclpy.init(args=args)
    node = FlowController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



