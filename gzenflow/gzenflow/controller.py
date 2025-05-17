# controller.py
import rclpy
from rclpy.node import Node
from pathlib import Path
from std_msgs.msg import String
from gzenflow_interfaces.msg import NetworkState
from gzenflow.zenoh_config_builder import generate_zenoh_bridge_config
from gzenflow.gstreamer_streamer import GStreamerStreamer
import subprocess
import yaml
import time

# Zustandsgrenzen in Mbit/s
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

# Bitrate-Skalierung
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
        self.config_dir = Path("/home/thomas/workspaces/master_ws/src/gzenflow/config")
        self.config_path = self.config_dir / "config.yaml"
        self.output_path = self.config_dir / "bridge_config.json5"

        self.gstreamer = GStreamerStreamer(self.config_path, self.get_logger())


        self.get_logger().info("📡 Warte auf Netzwerkdaten...")
        self.subscription = self.create_subscription(
            NetworkState,
            'network_state',
            self.network_callback,
            10
        )

    def network_callback(self, msg: NetworkState):
        if msg.state != "OK":
            self.get_logger().warn("⚠️ Netzwerk nicht erreichbar. Bridge wird deaktiviert.")
            current_state = "DISABLED"
        else:
            current_state = classify_bandwidth(msg.bandwidth_mbps)

        self.get_logger().info(f"🔄 Netzwerkzustand: {current_state} ({msg.bandwidth_mbps:.2f} Mbit/s)")

        # Bridge-Konfiguration
        generate_zenoh_bridge_config(
            yaml_path=str(self.config_path),
            output_path=str(self.output_path),
            current_state=current_state
        )
        self.restart_bridge()

        # GStreamer
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

    def start_bridge(self):
        self.get_logger().info(f"🚀 Starte zenoh-bridge-ros2dds mit {self.output_path}")
        self.bridge_proc = subprocess.Popen([
            "zenoh-bridge-ros2dds",
            "-c", str(self.output_path)
        ])

    def restart_bridge(self):
        if hasattr(self, 'bridge_proc'):
            self.get_logger().info("🔁 Stoppe bestehende Bridge...")
            self.bridge_proc.terminate()
            self.bridge_proc.wait()
            self.start_bridge()

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
# import time
# import subprocess
#
# from gzenflow.bridge_manager import BridgeManager
# from gzenflow.gstreamer_streamer import GStreamerStreamer
#
#
# class FlowController(Node):
#     def __init__(self):
#         super().__init__('bridge_controller')
#
#         # 📁 Konfigurationspfade
#         config_dir = Path("/home/thomas/workspaces/master_ws/src/gzenflow/config")
#         config_yaml = config_dir / "config.yaml"
#         bridge_json5 = config_dir / "bridge_config.json5"
#
#         # 🧠 Komponenten initialisieren
#         self.bridge_manager = BridgeManager(self.get_logger(), config_yaml, bridge_json5)
#         self.gstreamer = GStreamerStreamer(config_yaml, self.get_logger())
#
#         # ✅ Zustand: "ALL" erlaubt alle definierten Streams
#         self.get_logger().info("✅ Aktiviere Bridge und GStreamer-Streams für Zustand: ALL")
#         self.bridge_manager.generate_and_restart_bridge("ALL")
#         self.gstreamer.update_streams("ALL")  # <-- Diese Zeile war vorher auskommentiert
#
#         self.get_logger().info("🎉 Initialisierung abgeschlossen.")
#
#
#         # PHASE 2: GOOD
#         #self.get_logger().info("✅ Phase 2: Aktiviere zulässige Streams und Bridge für 30 Sekunden")
#         #self.gstreamer.update_streams("ALL")
#         #self.bridge_manager.generate_and_restart_bridge("ALL")
#
#
# def main(args=None):
#     rclpy.init(args=args)
#     controller = FlowController()
#     rclpy.spin(controller)
#     controller.destroy_node()
#     rclpy.shutdown()
#
#
# if __name__ == '__main__':
#     main()
#
# import rclpy
# from rclpy.node import Node
# from gzenflow.zenoh_config_builder import generate_zenoh_bridge_config
# from pathlib import Path
# import subprocess
# import time
#
# class FlowController(Node):
#     def __init__(self):
#         super().__init__('bridge_controller')
#
#         config_dir = Path("/home/thomas/workspaces/master_ws/src/gzenflow/config")
#         config_path = config_dir / "config.yaml"
#         output_path = config_dir / "bridge_config.json5"
#
#         # Phase 1: ALLES BLOCKIEREN
#         self.get_logger().info(f"🚫 Blockiere Bridge für 30 Sekunden")
#         generate_zenoh_bridge_config(
#             yaml_path=str(config_path),
#             output_path=str(output_path),
#             current_state="ALL"  # Zustand, bei dem kein Stream erlaubt ist
#         )
#         self.start_bridge(output_path)
#
#
#     def start_bridge(self, config_path):
#         self.get_logger().info(f"🚀 Starte zenoh-bridge-ros2dds mit {config_path}")
#         try:
#             self.bridge_proc = subprocess.Popen([
#                 "zenoh-bridge-ros2dds",
#                 "-c", str(config_path)
#             ])
#         except FileNotFoundError:
#             self.get_logger().error("❌ zenoh-bridge-ros2dds nicht gefunden. Bitte sicherstellen, dass es im $PATH liegt")
#
#
#     def restart_bridge(self, config_path):
#         self.get_logger().info("🔁 Stoppe bestehende Bridge...")
#         if hasattr(self, 'bridge_proc'):
#             self.bridge_proc.terminate()
#             self.bridge_proc.wait()
#             time.sleep(2)
#         self.start_bridge(config_path)
#
# def main(args=None):
#     rclpy.init(args=args)
#     node = FlowController()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()
# controller.py


#
#
#
#
# import rclpy
# from rclpy.node import Node
# from ament_index_python.packages import get_package_share_directory
# from gzenflow.zenoh_config_builder import generate_zenoh_bridge_config
# import os
# from pathlib import Path
# import subprocess
#
# class FlowController(Node):
#     def __init__(self):
#         super().__init__('bridge_controller')
#
#         config_dir = Path("/home/thomas/workspaces/master_ws/src/gzenflow/config")
#         config_path = config_dir / "config.yaml"
#         output_path = config_dir / "bridge_config.json5"
#
#         self.get_logger().info(f"🔧 Erzeuge Zenoh-Bridge-Konfiguration aus: {config_path}")
#         generate_zenoh_bridge_config(
#             yaml_path=str(config_path),
#             output_path=str(output_path),
#             current_state="GOOD")
#
#         self.get_logger().info(f"🚀 Starte zenoh-bridge-ros2dds mit {output_path}")
#         try:
#             subprocess.Popen([
#                 "zenoh-bridge-ros2dds",
#                 "-c", str(output_path)
#             ])
#         except FileNotFoundError:
#             self.get_logger().error("❌ zenoh-bridge-ros2dds nicht gefunden. Bitte sicherstellen, dass es im $PATH liegt.")
#
# def main(args=None):
#     rclpy.init(args=args)
#     node = FlowController()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()
#
#
