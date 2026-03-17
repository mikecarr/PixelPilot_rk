// main_rpi.cpp — PixelPilot entry point for Raspberry Pi 5
//
// Replaces the Rockchip MPP decode + DRM NV12 plane pipeline with:
//   GstRtpReceiver → VideoDecoderRpi (GStreamer) → VideoDisplay (GBM/EGL/GLES2)
//
// Everything else (OSD, LVGL, mavlink, wfbcli, DVR raw, menu, gsmenu)
// is unchanged from the Rockchip version.

#define MODULE_TAG "pixelpilot"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <unistd.h>
#include <signal.h>
#include <fstream>
#include <atomic>
#include <mutex>
#include <condition_variable>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/mman.h>

#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm_fourcc.h>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>
#include <spdlog/spdlog.h>

extern "C" {
#include "../main.h"
#include "../drm.h"
#include "../mavlink/common/mavlink.h"
#include "../mavlink.h"
#include "../input.h"
}

#include "../osd.h"
#include "../osd.hpp"
#include "../wfbcli.hpp"
#include "../dvr.h"
#include "../gstrtpreceiver.h"
#include "../scheduling_helper.hpp"
#include "../time_util.h"
#include "../os_mon.hpp"
#include "../pixelpilot_config.h"
#include "../WiFiRSSIMonitor.hpp"
#include "../gsmenu/gs_system.h"
#include "../gsmenu/air_actions.h"
#include "../gsmenu/gs_actions.h"
#include "../menu.h"
#include "video_decoder.h"
#include "video_display.h"

#include <iostream>

// ---------------------------------------------------------------------------
// Globals (shared with OSD, mavlink, wfbcli, gsmenu via extern declarations)
// ---------------------------------------------------------------------------
#define DEFAULT_CONFIG_PATH "/etc/pixelpilot/pixelpilot.yaml"
YAML::Node config;

#define MSG_FIFO_NAME "/run/pixelpilot.msg"

struct modeset_output* output_list = nullptr;
int drm_fd = 0;  // set to VideoDisplay's fd after init for OSD allocation

uint16_t wfb_port = 8003;
const char* wfb_api_host = "127.0.0.1";

bool mavlink_dvr_on_arm  = false;
bool osd_custom_message  = false;
bool disable_vsync       = false;
bool disable_gregidr     = false;
uint32_t refresh_frequency_ms = 1000;
// enable_osd is defined in osd.cpp

VideoCodec codec        = VideoCodec::H265;
uint16_t listen_port    = 5600;
const char* unix_socket = nullptr;
char* dvr_template      = nullptr;
// dvr_enabled is defined in dvr.cpp
Dvr* dvr_raw            = nullptr;
DvrMode dvr_mode        = DVR_MODE_RAW;
static int video_framerate = -1;
static bool dvr_filenames_with_sequence = false;
static int mp4_fragmentation_mode = 0;
static int64_t dvr_max_file_size = 4000000000LL;

OsSensors os_sensors;
MenuAction airactions[MAX_ACTIONS];
size_t airactions_count = 0;
MenuAction gsactions[MAX_ACTIONS];
size_t gsactions_count  = 0;

WiFiRSSIMonitor wifi_monitor;
extern enum RXMode RXMODE;

// OSD / display synchronisation (same as Rockchip version)
pthread_mutex_t video_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t  video_cond  = PTHREAD_COND_INITIALIZER;
// osd_update_ready is defined in osd.cpp
extern bool osd_update_ready;
extern pthread_mutex_t osd_mutex;

// Signal flags
int signal_flag = 0;
int return_value = 0;

// RPi-specific singletons
static VideoDisplay  g_display;
static VideoDecoderRpi* g_decoder = nullptr;

// ---------------------------------------------------------------------------
// Stubs: DVR re-encode functions (not implemented on RPi yet).
// The gsmenu calls these; they're no-ops until re-encode support is added.
// ---------------------------------------------------------------------------
extern "C" {
    void dvr_reenc_set_fps(int)         {}
    void dvr_reenc_set_osd(int)         {}
    void dvr_reenc_notify_colortrans(int) {}
    int  dvr_reenc_get_fps(void)        { return 30; }
    int  dvr_reenc_get_bitrate(void)    { return 8000; }
    int  dvr_reenc_get_osd(void)        { return 0; }
    int  dvr_reenc_get_codec(void)      { return 0; }
    int  dvr_reenc_get_resolution(void) { return 1; }
    int  dvr_get_mode(void)             { return (int)dvr_mode; }
    int  dvr_reenc_is_reenc(void)       { return 0; }
    void dvr_reenc_set_resolution(int)  {}
    void dvr_reenc_set_bitrate(int kbps) {
        spdlog::info("dvr_reenc_set_bitrate: re-encode not available on RPi yet");
    }
    void dvr_reenc_set_codec(int) {}

    void dvr_set_max_size(int mb) {
        dvr_max_file_size = (int64_t)mb * 1000000LL;
        if (dvr_raw) dvr_raw->set_max_file_size(dvr_max_file_size);
    }
    int dvr_get_max_size(void) { return (int)(dvr_max_file_size / 1000000LL); }

    void dvr_start_all(void) {
        dvr_enabled = 1;
        osd_publish_bool_fact("dvr.recording", nullptr, 0, true);
        if (dvr_raw) dvr_raw->start_recording();
    }
    void dvr_stop_all(void) {
        if (dvr_raw) dvr_raw->stop_recording();
        dvr_enabled = 0;
        osd_publish_bool_fact("dvr.recording", nullptr, 0, false);
    }
    void dvr_set_mode(int mode) {
        DvrMode new_mode = (DvrMode)mode;
        if (new_mode == DVR_MODE_REENCODE || new_mode == DVR_MODE_BOTH) {
            spdlog::warn("DVR re-encode not available on RPi yet; using raw mode");
            new_mode = DVR_MODE_RAW;
        }
        dvr_mode = new_mode;
    }
}

// ---------------------------------------------------------------------------
// Stubs: color-correction and DVR-playback functions (Rockchip-only features).
// gs_system.c / gs_dvr.c / gs_dvrplayer.c reference these via main.h.
// ---------------------------------------------------------------------------
bool   enable_live_colortrans  = false;
float  live_colortrans_offset  = -0.15f;
float  live_colortrans_gain    = 2.5f;
gamma_lut_controller lut_ctrl  = {};

void switch_pipeline_source(const char* /*source_type*/, const char* /*source_path*/) {}
void skip_duration(int64_t /*skip_ms*/) {}
void pause_playback()  {}
void resume_playback() {}

// ---------------------------------------------------------------------------
// Signal handlers
// ---------------------------------------------------------------------------
void sig_handler(int signum) {
    spdlog::info("Received signal {}", signum);
    signal_flag++;
    mavlink_thread_signal++;
    wfb_thread_signal++;
    osd_thread_signal++;
    if (dvr_raw) dvr_raw->shutdown();
    g_display.request_stop();
    return_value = signum;
}

void sigusr1_handler(int signum) {
    spdlog::info("Received signal {}", signum);
    if (dvr_enabled) {
        if (dvr_raw) dvr_raw->stop_recording();
        dvr_enabled = 0;
        osd_publish_bool_fact("dvr.recording", nullptr, 0, false);
    } else {
        dvr_enabled = 1;
        osd_publish_bool_fact("dvr.recording", nullptr, 0, true);
        if (dvr_raw) dvr_raw->start_recording();
    }
}

void sigusr2_handler(int /*signum*/) {
    disable_vsync = !disable_vsync;
    spdlog::info("disable_vsync: {}", disable_vsync);
}

// ---------------------------------------------------------------------------
// Helper: allocate template string with suffix before extension
// ---------------------------------------------------------------------------
static char* dvr_template_with_suffix(const char* tpl, const char* suffix) {
    std::string s(tpl);
    auto dot = s.rfind('.');
    if (dot != std::string::npos) s.insert(dot, suffix);
    else                           s.append(suffix);
    return strdup(s.c_str());
}

// ---------------------------------------------------------------------------
// Display thread — composites and flips at display rate
// ---------------------------------------------------------------------------
static void* display_thread(void* /*param*/) {
    pthread_setname_np(pthread_self(), "__DISPLAY");
    SchedulingHelper::set_thread_params_max_realtime("DISPLAY",
        SchedulingHelper::PRIORITY_REALTIME_MID);

    while (!g_display.stop_requested()) {
        // Pull pending OSD buffer if updated.
        pthread_mutex_lock(&osd_mutex);
        if (osd_update_ready && output_list) {
            struct modeset_buf* buf =
                &output_list->osd_bufs[output_list->osd_buf_switch];
            g_display.push_osd_frame(buf->map,
                                     (int)buf->width,
                                     (int)buf->height,
                                     (int)buf->stride);
            osd_update_ready = false;
        }
        pthread_mutex_unlock(&osd_mutex);

        if (!g_display.render()) {
            spdlog::error("Display render failed — exiting display thread");
            break;
        }
    }
    spdlog::info("Display thread done.");
    return nullptr;
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    spdlog::set_level(spdlog::level::info);

    // --- Load YAML config first (cmdline overrides it below) ----------------
    const char* config_path = DEFAULT_CONFIG_PATH;
    for (int i = 1; i < argc - 1; i++) {
        if (!strcmp(argv[i], "--config")) { config_path = argv[i + 1]; break; }
    }
    try {
        config = YAML::LoadFile(config_path);
        spdlog::info("Loaded config from {}", config_path);
    } catch (...) {
        spdlog::warn("Config file not found or invalid: {} — using defaults", config_path);
    }
    // Apply YAML defaults (cmdline will override below)
    if (config["codec"])       codec       = video_codec(config["codec"].as<std::string>().c_str());
    if (config["port"])        listen_port = config["port"].as<uint16_t>();
    if (config["dvr_template"]) {
        free(dvr_template);
        dvr_template = strdup(config["dvr_template"].as<std::string>().c_str());
    }
    if (config["osd"] && config["osd"]["refresh_frequency_ms"])
        refresh_frequency_ms = config["osd"]["refresh_frequency_ms"].as<uint32_t>();

    // --- Parse command line — mirrors Rockchip main.cpp flags ---------------
    const char* drm_node     = "/dev/dri/card1";  // card1=vc4-drm display; card0=v3d render-only
    uint32_t    mode_width   = 0;
    uint32_t    mode_height  = 0;
    uint32_t    mode_refresh = 0;
    const char* osd_config_path = nullptr;        // --osd-config <file>
    float       video_scale_factor = 1.0f;        // --video-scale (stored; unused on RPi for now)

    for (int i = 1; i < argc; i++) {
        const char* arg = argv[i];
        auto next = [&]() -> const char* { return (i + 1 < argc) ? argv[++i] : ""; };

        if (!strcmp(arg, "--config")) {
            next(); // already consumed above
        } else if (!strcmp(arg, "--codec")) {
            codec = video_codec(next());
        } else if (!strcmp(arg, "-p") || !strcmp(arg, "--port")) {
            listen_port = (uint16_t)atoi(next());
        } else if (!strcmp(arg, "--osd")) {
            // flag only — enables OSD and mavlink
            enable_osd = 1;
        } else if (!strcmp(arg, "--osd-config")) {
            osd_config_path = next();
        } else if (!strcmp(arg, "--osd-custom-message")) {
            osd_custom_message = true;
        } else if (!strcmp(arg, "--osd-refresh")) {
            refresh_frequency_ms = (uint32_t)atoi(next());
        } else if (!strcmp(arg, "--screen-mode")) {
            // Accepts both "1920x1080@60" and "1920 1080 60"
            const char* w = next();
            if (strchr(w, 'x')) {
                sscanf(w, "%ux%u@%u", &mode_width, &mode_height, &mode_refresh);
            } else {
                mode_width   = (uint32_t)atoi(w);
                mode_height  = (i + 1 < argc) ? (uint32_t)atoi(argv[++i]) : 0;
                mode_refresh = (i + 1 < argc) ? (uint32_t)atoi(argv[++i]) : 0;
            }
        } else if (!strcmp(arg, "--dvr")) {
            // --dvr [template] — template is optional; default to dated filename
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                dvr_template = strdup(argv[++i]);
            } else {
                dvr_template = strdup("/tmp/dvr_%Y-%m-%d_%H-%M-%S");
            }
            dvr_enabled = 1;
        } else if (!strcmp(arg, "--dvr-template")) {
            free(dvr_template);
            dvr_template = strdup(next());
        } else if (!strcmp(arg, "--dvr-start")) {
            dvr_enabled = 1;
        } else if (!strcmp(arg, "--dvr-framerate")) {
            video_framerate = atoi(next());
        } else if (!strcmp(arg, "--dvr-max-size")) {
            dvr_max_file_size = (int64_t)atoi(next()) * 1000000LL;
        } else if (!strcmp(arg, "--dvr-sequenced-files")) {
            dvr_filenames_with_sequence = true;
        } else if (!strcmp(arg, "--dvr-fmp4")) {
            mp4_fragmentation_mode = 1;
        } else if (!strcmp(arg, "--video-scale")) {
            video_scale_factor = (float)atof(next());
            if (video_scale_factor < 0.5f || video_scale_factor > 1.0f) {
                spdlog::warn("--video-scale {} out of range [0.5,1.0], clamping", video_scale_factor);
                video_scale_factor = std::max(0.5f, std::min(1.0f, video_scale_factor));
            }
        } else if (!strcmp(arg, "--wfb-api-port")) {
            wfb_port = (uint16_t)atoi(next());
        } else if (!strcmp(arg, "--wfb-api-host")) {
            wfb_api_host = next();
        } else if (!strcmp(arg, "--drm-node")) {
            drm_node = next();
        } else if (!strcmp(arg, "--screen-mode-list") || !strcmp(arg, "--list-modes")) {
            int tmp_fd;
            if (modeset_open(&tmp_fd, drm_node) == 0) {
                modeset_print_modes(tmp_fd);
                close(tmp_fd);
            }
            return 0;
        } else if (!strcmp(arg, "--verbose") || !strcmp(arg, "-v")) {
            spdlog::set_level(spdlog::level::debug);
        }
        // silently ignore unrecognised flags for forward-compat
    }

    // --- Signals ------------------------------------------------------------
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);
    signal(SIGUSR1, sigusr1_handler);
    signal(SIGUSR2, sigusr2_handler);
    signal(SIGPIPE, SIG_IGN);

    // --- GStreamer init ------------------------------------------------------
    gst_init(nullptr, nullptr);

    // --- VideoDisplay init --------------------------------------------------
    if (!g_display.init(drm_node, mode_width, mode_height, mode_refresh)) {
        spdlog::error("VideoDisplay init failed");
        return 1;
    }
    drm_fd = g_display.drm_fd();

    spdlog::info("Display: {}x{}",
                 g_display.display_width(), g_display.display_height());

    // --- Allocate OSD buffers (CPU-mapped DRM dumb buffers for LVGL/Cairo) --
    output_list = (struct modeset_output*)calloc(1, sizeof(struct modeset_output));
    output_list->mode.hdisplay   = (uint16_t)g_display.display_width();
    output_list->mode.vdisplay   = (uint16_t)g_display.display_height();
    output_list->video_crtc_width  = (int)g_display.display_width();
    output_list->video_crtc_height = (int)g_display.display_height();

    for (int i = 0; i < OSD_BUF_COUNT; i++) {
        output_list->osd_bufs[i].width  = g_display.display_width();
        output_list->osd_bufs[i].height = g_display.display_height();
        if (modeset_create_fb(drm_fd, &output_list->osd_bufs[i]) != 0) {
            spdlog::error("Failed to allocate OSD buffer {}", i);
            return 1;
        }
    }

    // --- DVR raw setup ------------------------------------------------------
    if (!dvr_template)
        dvr_template = strdup("/tmp/record_%Y-%m-%d_%H-%M-%S.mp4");

    dvr_thread_params dvr_params{};
    dvr_params.filename_template        = dvr_template;
    dvr_params.mp4_fragmentation_mode   = mp4_fragmentation_mode;
    dvr_params.dvr_filenames_with_sequence = dvr_filenames_with_sequence;
    dvr_params.video_framerate          = video_framerate;
    dvr_params.max_file_size            = dvr_max_file_size;
    dvr_params.video_p.codec            = codec;
    dvr_params.video_p.video_frm_width  = 0;  // updated on first frame
    dvr_params.video_p.video_frm_height = 0;

    dvr_raw = new Dvr(dvr_params);
    pthread_t tid_dvr_raw;
    pthread_create(&tid_dvr_raw, nullptr, Dvr::__THREAD__, dvr_raw);

    // --- OSD setup ----------------------------------------------------------
    nlohmann::json osd_config;
    // Load OSD JSON config from --osd-config path (or YAML fallback)
    if (!osd_config_path && config["osd"] && config["osd"]["config_file"])
        osd_config_path = config["osd"]["config_file"].as<std::string>().c_str();
    if (osd_config_path) {
        try {
            std::ifstream f(osd_config_path);
            osd_config = nlohmann::json::parse(f);
        } catch (...) {
            spdlog::warn("Could not parse OSD config {}", osd_config_path);
        }
    }
    if (osd_config.is_null()) osd_config = nlohmann::json::object();

    osd_thread_params osd_params{ output_list, drm_fd, osd_config };
    pthread_t tid_osd;
    pthread_create(&tid_osd, nullptr, __OSD_THREAD__, &osd_params);

    // --- gsmenu actions -------------------------------------------------------
    bool gsmenu_enabled = false;
    if (config["gsmenu"]) {
        if (config["gsmenu"]["enabled"])
            gsmenu_enabled = config["gsmenu"]["enabled"].as<bool>();
        if (gsmenu_enabled && config["gsmenu"]["actions"]) {
            if (config["gsmenu"]["actions"]["air"]) {
                const YAML::Node& n = config["gsmenu"]["actions"]["air"];
                for (size_t j = 0; j < n.size() && airactions_count < MAX_ACTIONS; j++) {
                    strncpy(airactions[airactions_count].label,  n[j]["name"].as<std::string>().c_str(),  MAX_LABEL_LEN - 1);
                    strncpy(airactions[airactions_count].action, n[j]["command"].as<std::string>().c_str(), MAX_ACTION_LEN - 1);
                    airactions_count++;
                }
            }
            if (config["gsmenu"]["actions"]["ground"]) {
                const YAML::Node& n = config["gsmenu"]["actions"]["ground"];
                for (size_t j = 0; j < n.size() && gsactions_count < MAX_ACTIONS; j++) {
                    strncpy(gsactions[j].label,  n[j]["name"].as<std::string>().c_str(),  MAX_LABEL_LEN - 1);
                    strncpy(gsactions[j].action, n[j]["command"].as<std::string>().c_str(), MAX_ACTION_LEN - 1);
                    gsactions_count++;
                }
            }
        }
    }

    // --- OS sensors ----------------------------------------------------------
    if (config["os_sensors"] && config["os_sensors"].IsMap()) {
        if (config["os_sensors"]["cpu"]) {
            auto cpu = config["os_sensors"]["cpu"];
            if (cpu.IsScalar() && cpu.as<std::string>() == "auto")
                os_sensors.discoverCPU();
            else
                os_sensors.addCPU();
        }
        if (config["os_sensors"]["power"]) {
            auto power = config["os_sensors"]["power"];
            if (power.IsScalar() && power.as<std::string>() == "auto") {
                os_sensors.discoverPower();
            } else {
                for (const auto& ps : power) {
                    std::string type    = ps["type"].as<std::string>();
                    std::string hwmon   = ps["hwmon_id"].as<std::string>();
                    os_sensors.addPower(type, hwmon);
                }
            }
        }
        if (config["os_sensors"]["temperature"]) {
            auto temperature = config["os_sensors"]["temperature"];
            if (temperature.IsScalar() && temperature.as<std::string>() == "auto") {
                os_sensors.discoverTemperature();
            } else {
                for (const auto& ts : temperature) {
                    std::string zone = ts["thermal_zone"].as<std::string>();
                    os_sensors.addTemperature(zone);
                }
            }
        }
    }

    // --- Restream manual IP --------------------------------------------------
    if (config["restream"] && config["restream"]["manual_ip"])
        restream_set_manual_ip(config["restream"]["manual_ip"].as<std::string>().c_str());

    // --- mavlink thread ------------------------------------------------------
    pthread_t tid_mavlink;
    pthread_create(&tid_mavlink, nullptr, __MAVLINK_THREAD__, nullptr);

    // --- wfbcli thread -------------------------------------------------------
    pthread_t tid_wfb;
    if (wfb_port) {
        wfb_thread_params* wfb_args = (wfb_thread_params*)malloc(sizeof *wfb_args);
        wfb_args->port = wfb_port;
        wfb_args->host = wfb_api_host;
        pthread_create(&tid_wfb, nullptr, __WFB_CLI_THREAD__, wfb_args);
    }

    // --- Display thread ------------------------------------------------------
    pthread_t tid_display;
    pthread_create(&tid_display, nullptr, display_thread, nullptr);

    // --- Video decoder + RTP receiver ---------------------------------------
    // Decoder callback: push RGBA frame to display and update video dimensions.
    auto decoder_cb = [](const RpiDecodedFrame& frame) {
        g_display.push_video_frame(frame.nv12.data(), frame.width, frame.height);
        // Publish video dimensions to OSD facts.
        osd_publish_uint_fact("video.width",  nullptr, 0, (uint32_t)frame.width);
        osd_publish_uint_fact("video.height", nullptr, 0, (uint32_t)frame.height);
        // Update output_list dimensions so gsmenu can read them.
        if (output_list) {
            output_list->video_frm_width  = (uint32_t)frame.width;
            output_list->video_frm_height = (uint32_t)frame.height;
        }
    };

    g_decoder = new VideoDecoderRpi(codec, decoder_cb);
    if (!g_decoder->start()) {
        spdlog::error("VideoDecoderRpi start failed");
        return 1;
    }

    // RTP receiver: feeds NAL units to decoder and (raw) DVR.
    GstRtpReceiver receiver(listen_port, codec);
    receiver.start_receiving([](std::shared_ptr<std::vector<uint8_t>> nal) {
        // Feed raw stream to DVR (no decode needed).
        if (dvr_raw) dvr_raw->frame(nal);
        // Feed to decoder.
        if (g_decoder) g_decoder->push_nal(nal);
    });

    spdlog::info("PixelPilot RPi listening on port {} ({})",
                 listen_port, codec == VideoCodec::H265 ? "h265" : "h264");

    // --- Main loop ----------------------------------------------------------
    while (!signal_flag && !g_display.stop_requested()) {
        usleep(100000);  // 100 ms
    }

    spdlog::info("Shutting down...");

    // Stop receiver first so no new NALs arrive.
    receiver.stop_receiving();

    // Stop decoder.
    g_decoder->stop();
    delete g_decoder;
    g_decoder = nullptr;

    // Stop display thread.
    g_display.request_stop();
    pthread_join(tid_display, nullptr);

    // Stop DVR.
    if (dvr_raw) { dvr_raw->shutdown(); pthread_join(tid_dvr_raw, nullptr); }

    // Stop OSD / mavlink / wfb threads.
    osd_thread_signal++;
    pthread_join(tid_osd, nullptr);
    pthread_join(tid_mavlink, nullptr);
    if (wfb_port) pthread_join(tid_wfb, nullptr);

    // Free OSD buffers.
    for (int i = 0; i < OSD_BUF_COUNT; i++)
        modeset_destroy_fb(drm_fd, &output_list->osd_bufs[i]);
    free(output_list);

    delete dvr_raw;
    free(dvr_template);

    spdlog::info("Goodbye.");
    return return_value;
}
