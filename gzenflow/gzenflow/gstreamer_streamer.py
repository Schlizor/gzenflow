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

    def start_or_update_stream(self, name, port, device, bitrate, stream_type="gstreamer", **kwargs):
        entry = self.active_pipelines.get(name)

        if entry:
            self.logger.info(f"GSTREAMER: Aktualisiere Bitrate für {name} auf Port {port} und Gerät {device}")
            self.change_bitrate(name, bitrate)
            self.resume_stream(name)
            return

        stream_config = {
            "type": stream_type,
            "port": port,
            "device": device,
            "min_bitrate": bitrate,
            "max_bitrate": bitrate,
            "allowed_in_state": ["ALL"],
        }
        stream_config.update(kwargs)

        if stream_type == "thermal":
            self._start_thermal_pipeline(name, stream_config)
        else:
            self._start_pipeline(name, stream_config)

    def update_streams(self, current_state):
        self._stop_all()

        streams = self.config.get("streams", {})

        for name, stream in streams.items():
            stype = stream.get("type")
            allowed = stream.get("allowed_in_state", [])
            if current_state not in allowed:
                self.logger.info(f"GSTREAMER: Deaktiviere Stream {name} für Zustand {current_state}")
                continue

            if stype == "gstreamer":
                self._start_pipeline(name, stream)
            elif stype == "thermal":
                self._start_thermal_pipeline(name, stream)
            else:
                continue

    def _start_pipeline(self, name, stream):
        device = stream.get("device", "/dev/video0")
        port = stream.get("port")
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

        try:
            pipeline = Gst.parse_launch(pipeline_description)
            state_change = pipeline.set_state(Gst.State.PLAYING)
            if state_change == Gst.StateChangeReturn.FAILURE:
                self.logger.error(f"GSTREAMER: Konnte GStreamer-Stream {name} nicht starten")
                return
        except Exception as e:
            self.logger.error(f"GSTREAMER: Fehler beim Starten der Pipeline für {name}: {e}")
            return

        self.active_pipelines[name] = {
            "type": "gstreamer",
            "pipeline": pipeline,
            "bitrate": max_br,
            "encoder": pipeline.get_by_name("encoder"),
        }

        self.logger.info(f"GSTREAMER: GStreamer-Stream {name} gestartet auf Port {port} und Bitrate {max_br} kbps")

    def _start_thermal_pipeline(self, name, stream):
        port = stream.get("port")
        max_br = stream.get("max_bitrate", 4000)
        width = stream.get("width", 320)
        height = stream.get("height", 240)

        if not Gst.ElementFactory.find("openseekthermalsrc"):
            self.logger.error(f"GSTREAMER: openseekthermalsrc Plugin nicht gefunden. Bitte sicherstellen, dass es installiert ist.")
            return

        norm = str(stream.get("normalize", True)).lower()
        nfc = stream.get("normalize-frame-count", 32)
        skip = str(stream.get("skip-invalid-frames", True)).lower()
        serial = stream.get("serial", "")
        src = (
            f"openseekthermalsrc normalize={norm} "
            f"normalize-frame-count={nfc} skip-invalid-frames={skip} "
            + (f'serial="{serial}" ' if serial else "")
        )

        pipeline_description = (
            f"{src}! video/x-raw,format=GRAY16_LE,width={width},height={height},framerate=15/1 ! "
            f"videoconvert ! video/x-raw,format=BGR ! "
            f"x264enc bitrate={max_br} tune=zerolatency speed-preset=ultrafast name=encoder ! "
            f"rtph264pay config-interval=1 pt=96 ! "
            f"udpsink host={self.target_ip} port={port}"
        )

        try:
            pipeline = Gst.parse_launch(pipeline_description)
            state_change = pipeline.set_state(Gst.State.PLAYING)
            if state_change == Gst.StateChangeReturn.FAILURE:
                self.logger.error(f"GSTREAMER: Konnte Thermal-Stream {name} nicht starten")
                return
        except Exception as e:
            self.logger.error(f"GSTREAMER: Fehler beim Starten des Thermal-Streams {name}: {e}")
            return

        self.active_pipelines[name] = {
            "type": "thermal",
            "pipeline": pipeline,
            "bitrate": max_br,
            "encoder": pipeline.get_by_name("encoder"),
        }

        self.logger.info(f"GSTREAMER: OpenSeekThermal-Stream {name} gestartet auf Port {port} und Bitrate {max_br} kbps")

    def change_bitrate(self, name, new_bitrate):
        entry = self.active_pipelines.get(name)
        if not entry:
            self.logger.warning(f"GSTREAMER: Kein Stream mit Namen {name} gefunden")
            return

        encoder = entry.get("encoder")
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
            entry["pipeline"].set_state(Gst.State.NULL)
            self.logger.info(f"GSTREAMER: Stoppe Stream {name}")

    def _stop_all(self):
        for name in list(self.active_pipelines.keys()):
            self.stop_stream(name)

# import gi
# gi.require_version('Gst', '1.0')
# from gi.repository import Gst
#
# import yaml
# from pathlib import Path
#
#
# class GStreamerStreamer:
#     def __init__(self, config_path, logger):
#         self.config_path = Path(config_path)
#         self.logger = logger
#         self.active_pipelines = {}
#         self.config = yaml.safe_load(self.config_path.read_text())
#         self.target_ip = self.config.get("global", {}).get("target_ip", "127.0.0.1")
#
#         if not Gst.is_initialized():
#             Gst.init(None)
#
#     def start_or_update_stream(self, name, port, device, bitrate, stream_type="gstreamer", **kwargs):
#         entry = self.active_pipelines.get(name)
#
#         if entry:
#             self.logger.info(f"GSTREAMER: Aktualisiere Bitrate für {name} auf Port {port} und Gerät {device}")
#             self.change_bitrate(name, bitrate)
#             self.resume_stream(name)
#             return
#
#         stream_config = {
#             "type": stream_type,
#             "port": port,
#             "device": device,
#             "min_bitrate": bitrate,
#             "max_bitrate": bitrate,
#             "allowed_in_state": ["ALL"],
#         }
#         # Merge any additional plugin-specific options
#         stream_config.update(kwargs)
#
#         if stream_type == "thermal":
#             self._start_thermal_pipeline(name, stream_config)
#         else:
#             self._start_pipeline(name, stream_config)
#
#     def update_streams(self, current_state):
#         self._stop_all()
#
#         streams = self.config.get("streams", {})
#
#         for name, stream in streams.items():
#             stype = stream.get("type")
#             allowed = stream.get("allowed_in_state", [])
#             if current_state not in allowed:
#                 self.logger.info(f"GSTREAMER: Deaktiviere Stream {name} für Zustand {current_state}")
#                 continue
#
#             if stype == "gstreamer":
#                 self._start_pipeline(name, stream)
#             elif stype == "thermal":
#                 self._start_thermal_pipeline(name, stream)
#             else:
#                 # handle other types if needed
#                 continue
#
#     def _start_pipeline(self, name, stream):
#         device = stream.get("device", "/dev/video0")
#         port = stream.get("port")
#         max_br = stream.get("max_bitrate", 4000)
#         width = stream.get("width", 640)
#         height = stream.get("height", 480)
#
#         pipeline_description = (
#             f"v4l2src device={device} ! "
#             f"video/x-raw,width={width},height={height} ! "
#             f"videoconvert ! "
#             f"x264enc bitrate={max_br} tune=zerolatency speed-preset=ultrafast name=encoder ! "
#             f"rtph264pay config-interval=1 pt=96 ! "
#             f"udpsink host={self.target_ip} port={port}"
#         )
#
#         pipeline = Gst.parse_launch(pipeline_description)
#         pipeline.set_state(Gst.State.PLAYING)
#
#         self.active_pipelines[name] = {
#             "type": "gstreamer",
#             "pipeline": pipeline,
#             "bitrate": max_br,
#             "encoder": pipeline.get_by_name("encoder"),
#         }
#
#         self.logger.info(f"GSTREAMER: GStreamer-Stream {name} gestartet auf Port {port} und Bitrate {max_br} kbps")
#
#     def _start_thermal_pipeline(self, name, stream):
#         port = stream.get("port")
#         max_br = stream.get("max_bitrate", 4000)
#         width = stream.get("width", 320)
#         height = stream.get("height", 240)
#
#         # OpenSeekThermal plugin options
#         norm = str(stream.get("normalize", True)).lower()
#         nfc = stream.get("normalize-frame-count", 32)
#         skip = str(stream.get("skip-invalid-frames", True)).lower()
#         serial = stream.get("serial", "")
#         src = (
#             f"openseekthermalsrc normalize={norm} "
#             f"normalize-frame-count={nfc} skip-invalid-frames={skip} "
#             + (f'serial="{serial}" ' if serial else "")
#         )
#
#         pipeline_description = (
#             f"{src}! video/x-raw,format=GRAY16_LE,width={width},height={height},framerate=15/1 ! "
#             f"videoconvert ! video/x-raw,format=BGR ! "
#             f"x264enc bitrate={max_br} tune=zerolatency speed-preset=ultrafast name=encoder ! "
#             f"rtph264pay config-interval=1 pt=96 ! "
#             f"udpsink host={self.target_ip} port={port}"
#         )
#
#         pipeline = Gst.parse_launch(pipeline_description)
#         pipeline.set_state(Gst.State.PLAYING)
#
#         self.active_pipelines[name] = {
#             "type": "thermal",
#             "pipeline": pipeline,
#             "bitrate": max_br,
#             "encoder": pipeline.get_by_name("encoder"),
#         }
#
#         self.logger.info(f"GSTREAMER: OpenSeekThermal-Stream {name} gestartet auf Port {port} und Bitrate {max_br} kbps")
#
#     def change_bitrate(self, name, new_bitrate):
#         entry = self.active_pipelines.get(name)
#         if not entry:
#             self.logger.warning(f"GSTREAMER: Kein Stream mit Namen {name} gefunden")
#             return
#
#         encoder = entry.get("encoder")
#         if encoder:
#             encoder.set_property("bitrate", new_bitrate)
#             self.logger.info(f"GSTREAMER: Bitrate von {name} auf {new_bitrate} gesetzt")
#             entry["bitrate"] = new_bitrate
#
#     def pause_stream(self, name):
#         entry = self.active_pipelines.get(name)
#         if entry:
#             entry["pipeline"].set_state(Gst.State.PAUSED)
#             self.logger.info(f"GSTREAMER: Pausiere Stream {name}")
#
#     def resume_stream(self, name):
#         entry = self.active_pipelines.get(name)
#         if entry:
#             entry["pipeline"].set_state(Gst.State.PLAYING)
#             self.logger.info(f"GSTREAMER: Fortsetze Stream {name}")
#
#     def stop_stream(self, name):
#         entry = self.active_pipelines.pop(name, None)
#         if entry:
#             entry["pipeline"].set_state(Gst.State.NULL)
#             self.logger.info(f"GSTREAMER: Stoppe Stream {name}")
#
#     def _stop_all(self):
#         for name in list(self.active_pipelines.keys()):
#             self.stop_stream(name)
