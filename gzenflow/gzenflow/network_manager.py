# network_manager.py
import rclpy
from rclpy.node import Node
from gzenflow_interfaces.msg import NetworkState
import iperf3
import yaml
from pathlib import Path
import time
import psutil

class NetworkManager(Node):
    def __init__(self):
        super().__init__('network_manager')
        self.publisher_ = self.create_publisher(NetworkState, 'network_state', 10)
        self.declare_parameter('config_path', '/home/thomas/workspaces/master_ws/src/gzenflow/config/config.yaml')

        config_path = Path(self.get_parameter('config_path').value)
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        self.target_ip = config.get('global', {}).get('target_ip', '127.0.0.1')

        self.get_logger().info(f"NETWORKMANAGER: Starte NetworkManager mit Ziel-IP: {self.target_ip}") 
        self.measure_bandwidth()
        self.timer = self.create_timer(20.0, self.measure_bandwidth)

    def measure_bandwidth(self):
        self.get_logger().info("NETWORKMANAGER: Messe Netzwerkbandbreite...")
        msg = NetworkState()

        try:
            client = iperf3.Client()
            client.server_hostname = self.target_ip
            client.port = 5201
            client.duration = 2
            client.protocol = 'tcp'

            result = client.run()
            if result.error:
                raise RuntimeError(result.error)
            msg.state = "OK"
            msg.bandwidth_mbps = result.sent_Mbps
            self.get_logger().info(f"NETWORKMANAGER:iperf3-Bandbreite: {msg.bandwidth_mbps:.2f} Mbit/s")

        except Exception as e:
            self.get_logger().warn(f"NETWORKMANAGER: iperf3 fehlgeschlagen: {e}")
            self.get_logger().info("NETWORKMANAGER: Verwende Fallback!")
            msg.state = "FALLBACK"
            msg.bandwidth_mbps = self.estimate_bandwidth_locally()
            self.get_logger().info(f"NETWORK MANAGER: Geschätzte Bandbreite (Fallback): {msg.bandwidth_mbps:.2f} Mbit/s")

        self.publisher_.publish(msg)

    def estimate_bandwidth_locally(self, duration=3):
        net1 = psutil.net_io_counters()
        time.sleep(duration)
        net2 = psutil.net_io_counters()
        bytes_sent = net2.bytes_sent - net1.bytes_sent
        bytes_recv = net2.bytes_recv - net1.bytes_recv
        total_bytes = bytes_sent + bytes_recv
        mbps = (total_bytes * 8 / 1_000_000) / duration
        return mbps

def main(args=None):
    rclpy.init(args=args)
    node = NetworkManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
