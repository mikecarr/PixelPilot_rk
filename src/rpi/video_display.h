#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <atomic>
#include <mutex>
#include <unordered_map>
#include <utility>
#include <vector>
#include <functional>

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES2/gl2.h>
#include <gbm.h>
#include <xf86drm.h>
#include <xf86drmMode.h>

// GBM + EGL + GLES2 compositor that outputs to a single DRM CRTC.
//
// Each rendered frame composites:
//   - Video layer : full-screen RGBA texture (from VideoDecoderRpi)
//   - OSD layer   : full-screen RGBA texture (from LVGL / Cairo render buffer)
//                   alpha-blended on top of video (source-over)
//
// The caller pushes new video pixels via push_video_frame() and new OSD pixels
// via push_osd_frame(). render() composites and flips to the screen; it must
// be called from the thread that owns the EGL context.
class VideoDisplay {
public:
    VideoDisplay() = default;
    ~VideoDisplay();

    // Open the DRM device, find a connected output, set up GBM + EGL + GLES2.
    // mode_width/mode_height/mode_refresh: preferred display mode (0 = use
    // connector's preferred mode).
    bool init(const char* drm_node, uint32_t mode_width, uint32_t mode_height,
              uint32_t mode_refresh);

    // Push a new decoded video frame (NV12: Y plane w*h bytes followed by
    // interleaved UV plane w*h/2 bytes).  Thread-safe; copies the pixel data.
    void push_video_frame(const uint8_t* nv12, int width, int height);

    // Push a new OSD frame (ARGB8888 / BGRA, stride bytes per row).
    // Pointer must remain valid until the call returns.  Thread-safe.
    void push_osd_frame(const uint8_t* argb, int width, int height, int stride);

    // Composite and flip one frame to the display.
    // Must be called from the thread that called init().
    // Returns false on fatal error.
    bool render();

    // Signal the render loop to stop (call from another thread, then join).
    void request_stop() { stop_requested_ = true; }
    bool stop_requested() const { return stop_requested_; }

    int drm_fd() const { return drm_fd_; }
    uint32_t display_width()  const { return mode_.hdisplay; }
    uint32_t display_height() const { return mode_.vdisplay; }

private:
    bool init_drm(const char* node, uint32_t w, uint32_t h, uint32_t refresh);
    bool init_gbm(const char* render_node);
    bool init_egl();
    bool init_gl();
    void cleanup();

    // Imports a GBM BO into the KMS device via DMA-buf and creates a DRM FB.
    // Returns the fb_id on success, 0 on failure.
    // Fills *out_gem_handle with the imported GEM handle (caller must close
    // it with drmCloseBufferHandle after the flip is complete).
    uint32_t import_bo_as_fb(struct gbm_bo* bo, uint32_t* out_gem_handle);

    // DRM (card1 — KMS display)
    int           drm_fd_      = -1;
    uint32_t      crtc_id_     = 0;
    uint32_t      connector_id_= 0;
    drmModeModeInfo mode_{};
    drmModeCrtc*  saved_crtc_  = nullptr;

    // GBM (renderD128 — v3d render node)
    int                 render_fd_ = -1;
    struct gbm_device*  gbm_dev_   = nullptr;
    struct gbm_surface* gbm_surf_  = nullptr;
    struct gbm_bo*      prev_bo_   = nullptr;

    // Cache of GBM BO → {fb_id, gem_handle} so we only call import_bo_as_fb()
    // once per BO.  GBM surfaces cycle through exactly 2 BOs, so this map
    // never holds more than 2 entries.
    std::unordered_map<struct gbm_bo*, std::pair<uint32_t, uint32_t>> bo_fb_cache_;

    // EGL
    EGLDisplay  egl_dpy_  = EGL_NO_DISPLAY;
    EGLContext  egl_ctx_  = EGL_NO_CONTEXT;
    EGLSurface  egl_surf_ = EGL_NO_SURFACE;
    EGLConfig   egl_cfg_  = nullptr;

    // GLES2
    GLuint prog_         = 0;
    GLuint y_tex_        = 0;   // NV12 Y plane  (GL_LUMINANCE,       w   × h  )
    GLuint uv_tex_       = 0;   // NV12 UV plane (GL_LUMINANCE_ALPHA, w/2 × h/2)
    GLuint osd_tex_      = 0;
    GLint  loc_y_        = -1;
    GLint  loc_uv_       = -1;
    GLint  loc_osd_      = -1;

    // Pending video frame (double-buffered via mutex)
    std::mutex             video_mtx_;
    std::vector<uint8_t>   video_pending_;
    int                    video_w_  = 0;
    int                    video_h_  = 0;
    bool                   video_dirty_ = false;

    // Pending OSD frame
    std::mutex             osd_mtx_;
    std::vector<uint8_t>   osd_pending_;
    int                    osd_w_    = 0;
    int                    osd_h_    = 0;
    bool                   osd_dirty_= false;

    std::atomic<bool>      stop_requested_{false};
    bool                   first_frame_  = true;
};
