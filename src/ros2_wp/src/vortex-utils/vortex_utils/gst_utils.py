import gi
import numpy as np

gi.require_version('Gst', '1.0')
gi.require_version('GstApp', '1.0')
from gi.repository import GLib, Gst


class H264Decoder:
    """Decodes H.264 streams using GStreamer."""

    _gst_initialized = False

    def __init__(self):
        """Initializes the H.264 decoder and sets up the GStreamer pipeline."""
        # Ensure GStreamer is initialized only once
        if not H264Decoder._gst_initialized:
            Gst.init(None)
            H264Decoder._gst_initialized = True

        pipeline_desc = (
            "appsrc name=mysrc is-live=true ! "
            "h264parse ! "
            "avdec_h264 ! "
            "videoconvert ! video/x-raw,format=BGR ! "
            "appsink name=appsink"
        )

        self._pipeline = Gst.parse_launch(pipeline_desc)
        self.appsrc = self._pipeline.get_by_name("mysrc")
        self._appsink = self._pipeline.get_by_name("appsink")

        self._appsink.set_property("emit-signals", True)
        self._appsink.set_property("sync", False)
        self._appsink.connect("new-sample", self._on_new_sample)

        self._bus = self._pipeline.get_bus()
        self._bus.add_signal_watch()
        self._bus.connect("message", self._on_bus_message)

        self._main_loop = None
        self.decoded_frames = []
        self.max_frames = 3  # Keep only the last 3 frames here

    def start(self):
        """Starts the GStreamer pipeline and runs the main event loop."""
        self._pipeline.set_state(Gst.State.PLAYING)
        self._main_loop = GLib.MainLoop()
        try:
            self._main_loop.run()
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()

    def stop(self):
        """Stops the GStreamer pipeline and cleans up resources."""
        if self._pipeline:
            self._pipeline.set_state(Gst.State.NULL)
        if self._main_loop is not None:
            self._main_loop.quit()
            self._main_loop = None

    def push_data(self, data: bytes):
        """Pushes H.264 encoded data into the pipeline for decoding."""
        if not self.appsrc:
            raise RuntimeError(
                "The pipeline's appsrc element was not found or initialized."
            )
        gst_buffer = Gst.Buffer.new_allocate(None, len(data), None)
        gst_buffer.fill(0, data)
        self.appsrc.emit("push-buffer", gst_buffer)

    def _on_bus_message(self, bus, message):
        """Handles messages from the GStreamer bus."""
        msg_type = message.type
        if msg_type == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print(f"GStreamer ERROR: {err}, debug={debug}")
            self.stop()
        elif msg_type == Gst.MessageType.EOS:
            print("End-Of-Stream reached.")
            self.stop()

    def _on_new_sample(self, sink):
        """Processes a new decoded video frame from the appsink."""
        sample = sink.emit("pull-sample")
        if not sample:
            return Gst.FlowReturn.ERROR

        buf = sample.get_buffer()
        caps_format = sample.get_caps().get_structure(0)
        width = caps_format.get_value("width")
        height = caps_format.get_value("height")

        success, map_info = buf.map(Gst.MapFlags.READ)
        if not success:
            return Gst.FlowReturn.ERROR

        frame_data = np.frombuffer(map_info.data, dtype=np.uint8)
        channels = len(frame_data) // (width * height)  # typically 3 (BGR) or 4 (BGRA)
        frame_data_reshaped = frame_data.reshape((height, width, channels))

        self.decoded_frames.append(frame_data_reshaped.copy())

        if len(self.decoded_frames) > self.max_frames:
            self.decoded_frames.pop(0)

        buf.unmap(map_info)
        return Gst.FlowReturn.OK
