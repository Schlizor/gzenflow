# bridge_manager.py
from gzenflow.zenoh_config_builder import generate_zenoh_bridge_config
import subprocess
import time

class BridgeManager:
    def __init__(self, logger, config_path, output_path):
        self.logger = logger
        self.config_path = config_path
        self.output_path = output_path
        self.bridge_proc = None

    def generate_and_start_bridge(self, current_state):
        self.logger.info(f"🔧 Erzeuge Zenoh-Bridge-Konfiguration für Zustand: {current_state}")
        generate_zenoh_bridge_config(
            yaml_path=str(self.config_path),
            output_path=str(self.output_path),
            current_state=current_state
        )
        self.start_bridge()

    def generate_and_restart_bridge(self, current_state):
        self.logger.info(f"♻️ Aktualisiere Zenoh-Bridge-Konfiguration für Zustand: {current_state}")
        generate_zenoh_bridge_config(
            yaml_path=str(self.config_path),
            output_path=str(self.output_path),
            current_state=current_state
        )
        self.restart_bridge()

    def start_bridge(self):
        self.logger.info(f"🚀 Starte zenoh-bridge-ros2dds mit Konfiguration: {self.output_path}")
        self.bridge_proc = subprocess.Popen([
            "zenoh-bridge-ros2dds",
            "-c", str(self.output_path)
        ])

    def restart_bridge(self):
        if self.bridge_proc:
            self.logger.info("🔁 Stoppe bestehende Bridge...")
            self.bridge_proc.terminate()
            self.bridge_proc.wait()
            time.sleep(2)
        self.start_bridge()
