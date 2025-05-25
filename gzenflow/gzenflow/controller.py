# controller.py
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

         # Config-Pfade aus dem gzenflow-Package-Share
        pkg_share = get_package_share_directory('gzenflow')
        config_dir = Path(pkg_share) / 'config'
        self.config_path = config_dir / 'config.yaml'
        self.output_path = config_dir / 'bridge_config.json5'

# Initialisiere BridgeManager und GStreamerStreamer
        self.bridge = BridgeManager(self.get_logger(), self.config_path, self.output_path)
        self.gstreamer = GStreamerStreamer(self.config_path, self.get_logger())

        self.get_logger().info("📡 Warte auf Netzwerkdaten...")
        self.subscription = self.create_subscription(
            NetworkState,
            'network_state',
            self.network_callback,
            10
        )
        self.bridge.start_bridge()

    def network_callback(self, msg: NetworkState):
        if msg.state != "OK":
            self.get_logger().warn("⚠️ Netzwerk nicht erreichbar. Bridge wird deaktiviert.")
            current_state = "DISABLED"
        else:
            current_state = classify_bandwidth(msg.bandwidth_mbps)

        self.get_logger().info(f"🔄 Netzwerkzustand: {current_state} ({msg.bandwidth_mbps:.2f} Mbit/s)")

      #  self.bridge.generate_and_restart_bridge(current_state)

        # Bridge-Konfiguration aktualisieren und starten
        self.bridge.generate_and_restart_bridge(current_state)
        

        # # GStreamer-Streams aktivieren oder pausieren
        with open(self.config_path, 'r') as f:
            config = yaml.safe_load(f)

        for name, stream in config.get('streams', {}).items():
            if stream.get('type') != 'gstreamer':
                continue
            allowed = stream.get('allowed_in_state', [])
            if "ALL" in allowed or current_state in allowed:
                bitrate = get_bitrate_for_state(stream['min_bitrate'], stream['max_bitrate'], current_state)
                self.gstreamer.start_or_update_stream(name, stream['port'], stream['device'], bitrate)
            else:
                self.gstreamer.stop_stream(name)

def main(args=None):
    rclpy.init(args=args)
    node = FlowController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



# # controller.py
# import rclpy
# from rclpy.node import Node
# from pathlib import Path
# from std_msgs.msg import String
# from gzenflow_interfaces.msg import NetworkState
# from gzenflow.zenoh_config_builder import generate_zenoh_bridge_config
# from gzenflow.gstreamer_streamer import GStreamerStreamer
# #from gzenflow.bridge_streamer import BridgeStreamer
# import subprocess
# import yaml
# import time
#
# def classify_bandwidth(bw_mbps):
#     if bw_mbps > 40:
#         return "EXCELLENT"
#     elif bw_mbps > 20:
#         return "GOOD"
#     elif bw_mbps > 5:
#         return "DEGRADED"
#     elif bw_mbps > 1:
#         return "CRITICAL"
#     else:
#         return "DISABLED"
#
# def get_bitrate_for_state(min_bitrate, max_bitrate, state):
#     scale = {
#         "EXCELLENT": 1.0,
#         "GOOD": 0.7,
#         "DEGRADED": 0.4,
#         "CRITICAL": 0.1,
#         "DISABLED": 0.0
#     }.get(state, 1.0)
#     return int(min_bitrate + scale * (max_bitrate - min_bitrate))
#
# class FlowController(Node):
#     def __init__(self):
#         super().__init__('bridge_controller')
#         self.config_dir = Path("/home/thomas/workspaces/master_ws/src/gzenflow/config")
#         self.config_path = self.config_dir / "config.yaml"
#         self.output_path = self.config_dir / "bridge_config.json5"
#
#         self.gstreamer = GStreamerStreamer(self.config_path, self.get_logger())
#         # self.bridge = BridgeStreamer(self.config_path, self.get_logger())
#         # self.bridge.start()
#
#         self.get_logger().info("📡 Warte auf Netzwerkdaten...")
#         self.subscription = self.create_subscription(
#             NetworkState,
#             'network_state',
#             self.network_callback,
#             10
#         )
#
#     def network_callback(self, msg: NetworkState):
#         if msg.state != "OK":
#             self.get_logger().warn("⚠️ Netzwerk nicht erreichbar. Bridge wird deaktiviert.")
#             current_state = "DISABLED"
#         else:
#             current_state = classify_bandwidth(msg.bandwidth_mbps)
#
#         self.get_logger().info(f"🔄 Netzwerkzustand: {current_state} ({msg.bandwidth_mbps:.2f} Mbit/s)")
#
#         generate_zenoh_bridge_config(
#             yaml_path=str(self.config_path),
#             output_path=str(self.output_path),
#             current_state=current_state
#         )
#         #self.restart_bridge()
#
#         # GStreamer
#         with open(self.config_path, 'r') as f:
#             config = yaml.safe_load(f)
#
#         for name, stream in config.get('streams', {}).items():
#             if stream.get('type') != 'gstreamer':
#                 continue
#             allowed = stream.get('allowed_in_state', [])
#             if "ALL" in allowed or current_state in allowed:
#                 bitrate = get_bitrate_for_state(stream['min_bitrate'], stream['max_bitrate'], current_state)
#                 self.gstreamer.start_or_update_stream(name, stream['port'], stream['device'], bitrate)
#             else:
#                 self.gstreamer.stop_stream(name)
#
#     def start_bridge(self):
#         self.get_logger().info(f"🚀 Starte zenoh-bridge-ros2dds mit {self.output_path}")
#         self.bridge_proc = subprocess.Popen([
#             "zenoh-bridge-ros2dds",
#             "-c", str(self.output_path)
#         ])
#
#     def restart_bridge(self):
#         if hasattr(self, 'bridge_proc'):
#             self.get_logger().info("🔁 Stoppe bestehende Bridge...")
#             self.bridge_proc.terminate()
#             self.bridge_proc.wait()
#             self.start_bridge()
#
# def main(args=None):
#     rclpy.init(args=args)
#     node = FlowController()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()
#
# if __name__ == '__main__':
#     main()
#
#
