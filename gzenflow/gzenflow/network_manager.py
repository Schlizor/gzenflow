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
        self.get_logger().info(f"NETWORKMANAGER: Starte mit Ziel-IP: {self.target_ip}")
        self.measure_network_quality()
        self.timer = self.create_timer(15.0, self.measure_network_quality)

    def measure_network_quality(self):
        self.get_logger().info("NETWORKMANAGER: Messe Netzwerkqualität...")
        msg = NetworkState()
        try:
            tcp_client = iperf3.Client()
            tcp_client.server_hostname = self.target_ip
            tcp_client.port = 5201
            tcp_client.duration = 2
            tcp_client.protocol = 'tcp'
            result = tcp_client.run()
            if result.error:
                raise RuntimeError(result.error)
            msg.state = "OK"
            msg.bandwidth_mbps = round(result.sent_Mbps, 2)
            self.get_logger().info(f"TCP-Bandbreite: {msg.bandwidth_mbps:.2f} Mbit/s")
        except Exception as e:
            self.get_logger().warn(f"TCP-Messung fehlgeschlagen: {e}")
            msg.state = "FALLBACK"
            msg.bandwidth_mbps = round(self.estimate_bandwidth_locally(), 2)
        msg.jitter_ms = 0.0
        msg.packet_loss_pct = 0.0

        time.sleep(0.5)

        if self.target_ip != "127.0.0.1" and msg.bandwidth_mbps > 10.0:
            try:
                udp_client = iperf3.Client()
                udp_client.server_hostname = self.target_ip
                udp_client.port = 5201
                udp_client.duration = 2
                udp_client.protocol = 'udp'
                udp_client.bandwidth = 1_000_000
                udp_client.blksize = 1400
                result = udp_client.run()
                if result.error:
                    raise RuntimeError(result.error)
                if result.lost_percent is not None:
                    msg.packet_loss_pct = float(round(result.lost_percent, 2))
                else:
                    msg.packet_loss_pct = 0.0

                if result.jitter_ms is not None:
                    msg.jitter_ms = float(round(result.jitter_ms, 2))
                else:
                    msg.jitter_ms = 0.0

                self.get_logger().info(f"UDP: Jitter: {msg.jitter_ms:.2f} ms, Packet Loss: {msg.packet_loss_pct:.2f}%")
                del udp_client
            except Exception as e:
                self.get_logger().warn(f"UDP-Messung fehlgeschlagen: {e}")
        else:
            self.get_logger().info("UDP-Messung übersprungen")
        self.publisher_.publish(msg)

    def estimate_bandwidth_locally(self, duration=3):
        net1 = psutil.net_io_counters()
        time.sleep(duration)
        net2 = psutil.net_io_counters()
        bytes_sent = net2.bytes_sent - net1.bytes_sent
        bytes_recv = net2.bytes_recv - net1.bytes_recv
        total_bytes = bytes_sent + bytes_recv
        mbps = (total_bytes * 8 / 1_000_000) / duration
        self.get_logger().info(f"Lokale Schätzung: {mbps:.2f} Mbit/s")
        return mbps

def main(args=None):
    rclpy.init(args=args)
    node = NetworkManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
