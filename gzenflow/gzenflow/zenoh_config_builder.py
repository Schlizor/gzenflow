import yaml
import json
from pathlib import Path

def generate_zenoh_bridge_config(yaml_path, output_path, current_state="ALL"):
    with open(yaml_path, "r") as f:
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
    print(f"ZENOH_CONFIG_GENERATOR: Erlaubte Streams: {allowed_publishers}")

    bridge_config = {
        "plugins": {
            "ros2dds": {
                "allow": {
                    "publishers": allowed_publishers,
                    "subscribers": [],
                    "service_servers": [],
                    "service_clients": [],
                    "action_servers": [],
                    "action_clients": [],
                }
            }
        },
        "connect": {
            "endpoints": [f"tcp/{target_ip}:7447"]
        }
    }

    with open(output_path, "w") as f:
        json.dump(bridge_config, f, indent=2)
    print(f"ZENOH_CONFIG_GENERATOR: Zenoh Bridge-Konfiguration gespeichert unter: {output_path}")

def main():
    generate_zenoh_bridge_config("config.yaml", "bridge_config.json5", current_state="GOOD")

if __name__ == "__main__":
    main()

