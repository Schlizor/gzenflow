# bridge_manager.py
#from gzenflow.zenoh_config_builder import generate_zenoh_bridge_config
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
        self.logger.info(f"ZENOH BRIDGE: Erzeuge Zenoh-Bridge-Konfiguration für Zustand: {current_state}")

        with open(self.config_path, "r") as f:
            config = yaml.safe_load(f)

        target_ip = config.get("global", {}).get("target_ip", "127.0.0.1")
        streams = config.get("streams", {})

        allowed_publishers = []
        for name, stream in streams.items():
            if stream.get("type") == "zenoh":
                allowed_states = stream.get("allowed_in_state", [])
                if current_state in allowed_states:
                    topic = stream.get("topic")
                    if topic:
                        allowed_publishers.append(topic)
        self.logger.info(f"✅ Erlaubte Streams: {allowed_publishers}")
       
        bridge_config = {
                "plugins": {
                    "ros2dds": {
                        "allow": {
                            "publishers": allowed_publishers,
                            "subscribers": [".*"],
                            "service_servers": [".*"],
                            "service_clients": [".*"],
                            "action_servers": [".*"],
                            "action_clients": [".*"],
                        }
                    },
                    "rest": {
                        "http_port": 8000
                    }
                },
                "connect": {
                    "endpoints": [f"tcp/{target_ip}:7447"]
                },
                "scouting": {
                    "multicast": {
                        "enabled": True,
                        "address": "224.0.0.224:7446",
                        "interface": "auto",
                        "autoconnect": {
                            "router": [],
                            "peer": ["router", "peer"]
                        },
                        "listen": True
                    },
                    "gossip": {
                        "enabled": True,
                        "multihop": False,
                        "autoconnect": {
                            "router": [],
                            "peer": ["router", "peer"]
                        }
                    }
                },
            }



        with open(self.output_path, "w") as f:
            json.dump(bridge_config, f, indent=2)
        self.logger.info(f"ZENOH BRIDGE: Zenoh Bridge-Konfiguration gespeichert unter: {self.output_path}")

