#include "video_decoder.h"

#include <spdlog/spdlog.h>
#include <gst/video/video.h>

VideoDecoderRpi::VideoDecoderRpi(VideoCodec codec, FrameCallback cb)
    : codec_(codec), cb_(std::move(cb)) {}

VideoDecoderRpi::~VideoDecoderRpi() {
    stop();
}

bool VideoDecoderRpi::start() {
    // Build pipeline string based on codec.
    // Both paths end with videoconvert → RGBA appsink so the display layer
    // always receives plain RGBA pixels with no format negotiation needed.
    // H.265: v4l2slh265dec prefers DMABuf output which videoconvert can't handle.
    // Force NV12 system-memory output with an explicit capsfilter, then convert to RGBA.
    const char* parse_decode =
        (codec_ == VideoCodec::H265)
            ? "h265parse ! avdec_h265"
            : "h264parse ! avdec_h264";

    const char* src_caps =
        (codec_ == VideoCodec::H265)
            ? "video/x-h265,stream-format=byte-stream,alignment=au"
            : "video/x-h264,stream-format=byte-stream,alignment=au";

    // Output NV12 (semi-planar YUV420) directly from the software decoder.
    // Skipping videoconvert ! RGBA eliminates a full-frame colorspace conversion
    // (~5-10 ms at 1080p) and halves the bytes moved per frame (1.5 vs 4 bytes/pixel).
    // YUV→RGB conversion is done in the fragment shader at zero CPU cost.
    // max-bytes=0 disables the appsrc internal queue so stale pre-buffered
    // NALs (accumulated in the wfb-ng UDP socket before pixelpilot started)
    // are not replayed, keeping end-to-end latency low.
    std::string pipeline_str =
        std::string("appsrc name=src format=time is-live=true max-bytes=0 caps=") +
        src_caps + " ! " + parse_decode +
        " ! video/x-raw,format=NV12"
        " ! appsink name=sink emit-signals=false sync=false max-buffers=1 drop=true";

    spdlog::info("VideoDecoder: pipeline: {}", pipeline_str);

    GError* err = nullptr;
    pipeline_ = gst_parse_launch(pipeline_str.c_str(), &err);
    if (!pipeline_ || err) {
        spdlog::error("VideoDecoder: gst_parse_launch failed: {}",
                      err ? err->message : "unknown");
        if (err) g_error_free(err);
        return false;
    }

    appsrc_  = GST_APP_SRC(gst_bin_get_by_name(GST_BIN(pipeline_), "src"));
    appsink_ = GST_APP_SINK(gst_bin_get_by_name(GST_BIN(pipeline_), "sink"));

    if (!appsrc_ || !appsink_) {
        spdlog::error("VideoDecoder: could not find appsrc/appsink in pipeline");
        gst_object_unref(pipeline_); pipeline_ = nullptr;
        return false;
    }

    // Use callback-based sample delivery (no need for a pull thread).
    GstAppSinkCallbacks cbs{};
    cbs.new_sample = on_new_sample_cb;
    gst_app_sink_set_callbacks(appsink_, &cbs, this, nullptr);

    GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        spdlog::error("VideoDecoder: failed to set pipeline to PLAYING");
        gst_object_unref(appsrc_);  appsrc_  = nullptr;
        gst_object_unref(appsink_); appsink_ = nullptr;
        gst_object_unref(pipeline_); pipeline_ = nullptr;
        return false;
    }

    // Start a thread to monitor the GStreamer bus for errors.
    bus_ = gst_element_get_bus(pipeline_);
    bus_thread_ = std::thread([this]() {
        while (running_) {
            GstMessage* msg = gst_bus_timed_pop_filtered(bus_,
                200 * GST_MSECOND,
                (GstMessageType)(GST_MESSAGE_ERROR | GST_MESSAGE_WARNING | GST_MESSAGE_STATE_CHANGED));
            if (!msg) continue;
            switch (GST_MESSAGE_TYPE(msg)) {
                case GST_MESSAGE_ERROR: {
                    GError* err = nullptr; gchar* dbg = nullptr;
                    gst_message_parse_error(msg, &err, &dbg);
                    spdlog::error("VideoDecoder GStreamer error: {} ({})",
                                  err ? err->message : "?", dbg ? dbg : "");
                    if (err) g_error_free(err);
                    if (dbg) g_free(dbg);
                    break;
                }
                case GST_MESSAGE_WARNING: {
                    GError* err = nullptr; gchar* dbg = nullptr;
                    gst_message_parse_warning(msg, &err, &dbg);
                    spdlog::warn("VideoDecoder GStreamer warning: {} ({})",
                                 err ? err->message : "?", dbg ? dbg : "");
                    if (err) g_error_free(err);
                    if (dbg) g_free(dbg);
                    break;
                }
                default: break;
            }
            gst_message_unref(msg);
        }
    });

    running_ = true;
    spdlog::info("VideoDecoder: started ({}, NV12 output)",
                 codec_ == VideoCodec::H265 ? "h265/sw" : "h264/sw");
    return true;
}

void VideoDecoderRpi::stop() {
    if (!pipeline_) return;
    running_ = false;

    if (bus_thread_.joinable()) bus_thread_.join();
    if (bus_) { gst_object_unref(bus_); bus_ = nullptr; }

    gst_app_src_end_of_stream(appsrc_);
    gst_element_set_state(pipeline_, GST_STATE_NULL);

    gst_object_unref(appsrc_);   appsrc_   = nullptr;
    gst_object_unref(appsink_);  appsink_  = nullptr;
    gst_object_unref(pipeline_); pipeline_ = nullptr;
    spdlog::info("VideoDecoder: stopped");
}

// Scan a byte-stream access unit for an IRAP (keyframe) NAL unit.
// H.265 IRAP types: 16–21. H.264 IDR type: 5.
static bool access_unit_has_keyframe(const uint8_t* d, size_t sz, bool h265) {
    for (size_t i = 0; i + 4 < sz; i++) {
        if (d[i] == 0 && d[i+1] == 0 && d[i+2] == 0 && d[i+3] == 1 && i+5 <= sz) {
            uint8_t t = h265 ? ((d[i+4] >> 1) & 0x3F) : (d[i+4] & 0x1F);
            if (h265 ? (t >= 16 && t <= 21) : (t == 5)) return true;
        }
    }
    return false;
}

void VideoDecoderRpi::push_nal(std::shared_ptr<std::vector<uint8_t>> nal) {
    if (!running_ || !appsrc_ || !nal || nal->empty()) return;

    const bool is_h265 = (codec_ == VideoCodec::H265);
    const bool is_key  = access_unit_has_keyframe(nal->data(), nal->size(), is_h265);

    // IDR gate: discard all frames until the first keyframe.  This prevents
    // the decoder from processing a backlog of undecoded P-frames that
    // accumulated before stream-up, which would add seconds of latency.
    if (!seen_idr_) {
        if (!is_key) return;
        seen_idr_ = true;
        spdlog::info("VideoDecoder: first keyframe — starting decode");
    }

    // Resync gate: if the appsrc queue has grown large the pipeline is falling
    // behind the source rate.  Discard P-frames until the next IDR, then flush
    // all stale queued data so decode restarts clean at low latency.
    if (need_idr_) {
        if (!is_key) return;
        // Just push the IDR — no pipeline flush needed.  The old P-frames still
        // in the appsrc queue fail-decode almost instantly (no reference frame),
        // then the IDR gives a clean restart with no hard freeze.
        spdlog::info("VideoDecoder: resyncing on IDR (pipeline was behind)");
        need_idr_ = false;
    } else if (!is_key) {
        // Check appsrc queue depth.  If it is growing the decode thread cannot
        // keep up; flag that we need a fresh IDR to resync.
        guint64 queued = gst_app_src_get_current_level_bytes(appsrc_);
        if (queued > 500000) {
            spdlog::info("VideoDecoder: appsrc queue {}KB — waiting for IDR to resync",
                         queued / 1024);
            need_idr_ = true;
            return;
        }
    }

    GstBuffer* buf = gst_buffer_new_allocate(nullptr, nal->size(), nullptr);
    GstMapInfo map;
    gst_buffer_map(buf, &map, GST_MAP_WRITE);
    memcpy(map.data, nal->data(), nal->size());
    gst_buffer_unmap(buf, &map);

    // Use the actual monotonic clock as PTS so the pipeline sees real frame
    // timing (avoids artificial 30fps assumption mismatch when source is 60/90fps).
    pts_ns_ = (uint64_t)g_get_monotonic_time() * 1000ULL;  // µs → ns
    GST_BUFFER_PTS(buf)      = pts_ns_;
    GST_BUFFER_DURATION(buf) = GST_CLOCK_TIME_NONE;

    GstFlowReturn ret = gst_app_src_push_buffer(appsrc_, buf);
    if (ret != GST_FLOW_OK && ret != GST_FLOW_FLUSHING) {
        spdlog::warn("VideoDecoder: appsrc push returned {}", (int)ret);
    }
}

// static
GstFlowReturn VideoDecoderRpi::on_new_sample_cb(GstAppSink* /*sink*/, gpointer userdata) {
    return static_cast<VideoDecoderRpi*>(userdata)->on_new_sample();
}

GstFlowReturn VideoDecoderRpi::on_new_sample() {
    GstSample* sample = gst_app_sink_pull_sample(appsink_);
    if (!sample) return GST_FLOW_ERROR;

    GstCaps*   caps = gst_sample_get_caps(sample);
    GstBuffer* buf  = gst_sample_get_buffer(sample);

    // Extract width/height from caps on first frame (or if they change).
    if (!caps) {
        spdlog::warn("VideoDecoder: sample has no caps");
        gst_sample_unref(sample);
        return GST_FLOW_OK;
    }
    GstVideoInfo vinfo;
    if (caps && gst_video_info_from_caps(&vinfo, caps)) {
        RpiDecodedFrame frame;
        frame.width  = GST_VIDEO_INFO_WIDTH(&vinfo);
        frame.height = GST_VIDEO_INFO_HEIGHT(&vinfo);
        // NV12: Y plane (w*h) + interleaved UV plane (w*h/2) = w*h*3/2 bytes.
        size_t nv12_size = (size_t)frame.width * frame.height * 3 / 2;
        frame.nv12.resize(nv12_size);

        GstMapInfo map;
        if (gst_buffer_map(buf, &map, GST_MAP_READ)) {
            size_t copy_sz = std::min(nv12_size, map.size);
            memcpy(frame.nv12.data(), map.data, copy_sz);
            gst_buffer_unmap(buf, &map);

            static uint64_t frame_count = 0;
            if (++frame_count == 1 || frame_count % 100 == 0)
                spdlog::info("VideoDecoder: decoded frame {} ({}x{} {} bytes NV12)",
                             frame_count, frame.width, frame.height, copy_sz);

            cb_(frame);
        }
    }

    gst_sample_unref(sample);
    return GST_FLOW_OK;
}
