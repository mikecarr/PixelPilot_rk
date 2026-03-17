#pragma once

#include <functional>
#include <memory>
#include <vector>
#include <string>
#include <atomic>
#include <thread>

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>

#include "../gstrtpreceiver.h"  // VideoCodec

// Decoded NV12 frame delivered to the callback.
// Layout: Y plane (width*height bytes) immediately followed by
//         interleaved UV plane (width*height/2 bytes, half-resolution).
// Total size: width * height * 3/2 bytes.
struct RpiDecodedFrame {
    std::vector<uint8_t> nv12;  // Y plane + UV plane (NV12 / semi-planar YUV420)
    int width  = 0;
    int height = 0;
};

// Wraps a GStreamer decode pipeline (appsrc → parse → decode → videoconvert →
// appsink). Receives raw H264/H265 access units from GstRtpReceiver and
// delivers decoded RGBA frames via callback.
//
// H.265: uses v4l2slh265dec (RPi 5 hardware decoder)
// H.264: uses avdec_h264   (software, Cortex-A76 handles 1080p30 fine)
class VideoDecoderRpi {
public:
    using FrameCallback = std::function<void(const RpiDecodedFrame&)>;

    explicit VideoDecoderRpi(VideoCodec codec, FrameCallback cb);
    ~VideoDecoderRpi();

    // Feed one access unit (from GstRtpReceiver::NEW_FRAME_CALLBACK).
    // Thread-safe: may be called from any thread.
    void push_nal(std::shared_ptr<std::vector<uint8_t>> nal);

    // Reset the IDR gate so the next call to push_nal discards frames until
    // a keyframe arrives. Call this on stream-down to avoid replay latency
    // when the stream reconnects.
    void reset_idr_gate() { seen_idr_ = false; }

    // Start / stop the pipeline.
    bool start();
    void stop();

private:
    static GstFlowReturn on_new_sample_cb(GstAppSink* sink, gpointer userdata);
    GstFlowReturn on_new_sample();

    VideoCodec    codec_;
    FrameCallback cb_;

    GstElement*   pipeline_  = nullptr;
    GstAppSrc*    appsrc_    = nullptr;
    GstAppSink*   appsink_   = nullptr;
    GstBus*       bus_       = nullptr;

    std::atomic<bool> running_{false};
    std::atomic<bool> seen_idr_{false};  // IDR gate: discard pre-IDR frames
    std::atomic<bool> need_idr_{false};  // resync gate: pipeline fell behind, wait for IDR flush
    std::thread       bus_thread_;
    uint64_t          pts_ns_{0};   // monotonically increasing PTS
};
