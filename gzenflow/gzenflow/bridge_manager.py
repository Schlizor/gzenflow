import subprocess
import time
import yaml
import json

class BridgeManager:
    def __init__(self, logger, config_path, output_path):
        self.logger = logger
        self.config_path = config_path
        self.output_path = output_path
        self.bridge_proc = None
        self.last_state = None
 
    def generate_and_start_bridge(self, current_state):
        self.logger.info(f"ZENOH BRIDGE: Erzeuge Konfiguration für Zustand: {current_state}")
        self.generate_zenoh_config(current_state)
        self.start_bridge()

    def generate_and_restart_bridge(self, current_state):
        if self.last_state != current_state:
            self.logger.info(f"ZENOH BRIDGE: Aktualisiere Zenoh-Bridge-Konfiguration für Zustand: {current_state}")        
            self.generate_zenoh_config(current_state)
            self.restart_bridge()
            self.last_state = current_state
        else :
            self.logger.info(f"ZENOH BRIDGE: Zenoh-Bridge-Konfiguration bleibt unverändert für Zustand: {current_state}")

    def start_bridge(self):
        self.logger.info(f"ZENOH BRIDGE: Starte zenoh-bridge-ros2dds mit Konfiguration: {self.output_path}")
        self.bridge_proc = subprocess.Popen([
            "zenoh-bridge-ros2dds",
            "-c", str(self.output_path)
        ])

    def restart_bridge(self):
        if self.bridge_proc:
            self.logger.info("ZENOH BRIDGE: Stoppe bestehende Bridge...")
            self.bridge_proc.terminate()
            self.bridge_proc.wait()
            time.sleep(2)
        self.start_bridge()

    def generate_zenoh_config(self, current_state):
        self.logger.info(f"ZENOH BRIDGE: Generating Zenoh-Bridge configuration for state: {current_state}")

        with open(self.config_path, "r") as f:
            config = yaml.safe_load(f)

        target_ip = config.get("global", {}).get("target_ip", "127.0.0.1")
        streams = config.get("streams", {})

        allowed_publishers = []
        pub_freqs = []
        for name, stream in streams.items():
            if stream.get("type") == "zenoh":
                allowed_states = stream.get("allowed_in_state", [])
                if current_state in allowed_states:
                    topic = stream.get("topic")
                    if topic:
                        allowed_publishers.append(topic)
                        freq = stream.get("frequency")
                        if isinstance(freq, (int, float)):
                            pub_freqs.append(f"{topic}={freq}")
        self.logger.info(f"Allowed Zenoh publishers: {allowed_publishers}")
        if pub_freqs:
            self.logger.info(f"ZENOH BRIDGE: Applying pub_max_frequencies: {pub_freqs}")

        bridge_config = {
            "mode": "peer",
            "plugins": {
                "ros2dds": {
                    "allow": {
                        "publishers": allowed_publishers,
                        "subscribers": [".*"],
                        "service_servers": [".*"],
                        "service_clients": [".*"],
                        "action_servers": [".*"],
                        "action_clients": [".*"],
                    },
                },
                "rest": { "http_port": 8000 }
            },
            #"connect": { "endpoints": [f"tcp/{target_ip}:7447"] },
            "scouting": {
                "multicast": {
                    "enabled": True,
                    "interface": "auto",
                    "autoconnect": { "router": [], "peer": ["router", "peer"] },
                    "listen": True
                },
                "gossip": {
                    "enabled": True,
                    "multihop": True,
                    "autoconnect": { "router": [], "peer": ["router", "peer"] }
                }
            }
        }


        if pub_freqs:
            bridge_config["plugins"]["ros2dds"]["pub_max_frequencies"] = pub_freqs

        with open(self.output_path, "w") as f:
            json.dump(bridge_config, f, indent=2)
        self.logger.info(f"ZENOH BRIDGE: Saved Zenoh-Bridge config to: {self.output_path}")

