import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject

import yaml
from pathlib import Path


class GStreamerStreamer:
    def __init__(self, config_path, logger):
        self.config_path = Path(config_path)
        self.logger = logger
        self.active_pipelines = {}
        self.config = yaml.safe_load(self.config_path.read_text())
        self.target_ip = self.config.get("global", {}).get("target_ip", "127.0.0.1")

        if not Gst.is_initialized():
            Gst.init(None)

    def start_or_update_stream(self, name, port, device, bitrate):
        entry = self.active_pipelines.get(name)

        if entry:
            self.logger.info(f"GSTREAMER: Aktualisiere Bitrate für {name} auf Port {port} und Gerät {device}")
            self.change_bitrate(name, bitrate)
            self.resume_stream(name)
            return

        stream_config = {
            "type": "gstreamer",
            "port": port,
            "device": device,
            "min_bitrate": bitrate,
            "max_bitrate": bitrate,  
            "allowed_in_state": ["ALL"], 
        }
        self._start_pipeline(name, stream_config)

    def update_streams(self, current_state):
        self._stop_all()

        streams = self.config.get("streams", {})

        for name, stream in streams.items():
            if stream.get("type") != "gstreamer":
                continue

            allowed = stream.get("allowed_in_state", [])
            if current_state not in allowed:
                self.logger.info(f"GSTREAMER: Deaktiviere Stream {name} für Zustand {current_state}")
                continue

            self._start_pipeline(name, stream)

    def _start_pipeline(self, name, stream):
        device = stream.get("device", "/dev/video0")
        port = stream.get("port") 
        min_br = stream.get("min_bitrate", 1000)
        max_br = stream.get("max_bitrate", 4000)
        width = stream.get("width", 640)
        height = stream.get("height", 480)

        pipeline_description = (
            f"v4l2src device={device} ! "
            f"video/x-raw,width={width},height={height} ! "
            f"videoconvert ! "
            f"x264enc bitrate={max_br} tune=zerolatency speed-preset=ultrafast name=encoder ! "
            f"rtph264pay config-interval=1 pt=96 ! "
            f"udpsink host={self.target_ip} port={port}"
        )

        pipeline = Gst.parse_launch(pipeline_description)
        pipeline.set_state(Gst.State.PLAYING)

        self.active_pipelines[name] = {
            "pipeline": pipeline,
            "bitrate": max_br,
            "encoder": pipeline.get_by_name("encoder"),
        }

        self.logger.info(f"GSTREAMER: GStreamer-Stream {name} gestartet auf Port {port} und Bitratre {max_br} kbps")

    def change_bitrate(self, name, new_bitrate):
        entry = self.active_pipelines.get(name)
        if not entry:
            self.logger.warning(f"GSTREAMER: Kein Stream mit Namen {name} gefunden")
            return

        encoder = entry["encoder"]
        if encoder:
            encoder.set_property("bitrate", new_bitrate)
            self.logger.info(f"GSTREAMER: Bitrate von {name} auf {new_bitrate} gesetzt")
            entry["bitrate"] = new_bitrate

    def pause_stream(self, name):
        entry = self.active_pipelines.get(name)
        if entry:
            entry["pipeline"].set_state(Gst.State.PAUSED)
            self.logger.info(f"GSTREAMER: Pausiere Stream {name}")

    def resume_stream(self, name):
        entry = self.active_pipelines.get(name)
        if entry:
            entry["pipeline"].set_state(Gst.State.PLAYING)
            self.logger.info(f"GSTREAMER: Fortsetze Stream {name}")

    def stop_stream(self, name):
        entry = self.active_pipelines.pop(name, None)
        if entry:
            pipeline = entry["pipeline"]
            pipeline.set_state(Gst.State.NULL)
            self.logger.info(f"GSTREAMER: Stoppe Stream {name}")

    def _stop_all(self):
        for name in list(self.active_pipelines.keys()):
            self.stop_stream(name)

