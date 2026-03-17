#include "video_display.h"

#include <cstring>
#include <cstdlib>
#include <fcntl.h>
#include <unistd.h>
#include <drm_fourcc.h>
#include <spdlog/spdlog.h>

// ---------------------------------------------------------------------------
// GLES2 shaders
// ---------------------------------------------------------------------------
static const char* kVert = R"GLSL(
attribute vec2 a_pos;
attribute vec2 a_uv;
varying vec2 v_uv;
void main() {
    v_uv = a_uv;
    gl_Position = vec4(a_pos, 0.0, 1.0);
}
)GLSL";

// Composite: NV12 video (YUV→RGB in shader) source-under OSD (straight alpha).
// Y  texture: GL_LUMINANCE,       w   × h   — one byte per pixel
// UV texture: GL_LUMINANCE_ALPHA, w/2 × h/2 — Cb in .r, Cr in .a
// BT.601 full-range coefficients (standard for IP camera output).
static const char* kFrag = R"GLSL(
precision mediump float;
varying vec2 v_uv;
uniform sampler2D u_y;
uniform sampler2D u_uv;
uniform sampler2D u_osd;
void main() {
    float y  = texture2D(u_y,  v_uv).r;
    vec2  uv = texture2D(u_uv, v_uv).ra - vec2(0.5);
    float r = y + 1.402 * uv.y;
    float g = y - 0.344 * uv.x - 0.714 * uv.y;
    float b = y + 1.772 * uv.x;
    vec4 video = vec4(clamp(r, 0.0, 1.0), clamp(g, 0.0, 1.0), clamp(b, 0.0, 1.0), 1.0);
    vec4 osd   = texture2D(u_osd, v_uv);
    gl_FragColor = osd + video * (1.0 - osd.a);
}
)GLSL";

// Full-screen quad: two triangles covering NDC [-1,1]
static const GLfloat kQuadVerts[] = {
    // x      y      u     v
    -1.0f, -1.0f,  0.0f, 1.0f,   // bottom-left  (UV y flipped: 0→1 top→bottom)
     1.0f, -1.0f,  1.0f, 1.0f,   // bottom-right
    -1.0f,  1.0f,  0.0f, 0.0f,   // top-left
     1.0f,  1.0f,  1.0f, 0.0f,   // top-right
};

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static GLuint compile_shader(GLenum type, const char* src) {
    GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, nullptr);
    glCompileShader(s);
    GLint ok = GL_FALSE;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (ok != GL_TRUE) {
        char log[512];
        glGetShaderInfoLog(s, sizeof(log), nullptr, log);
        spdlog::error("VideoDisplay shader error: {}", log);
        glDeleteShader(s);
        return 0;
    }
    return s;
}

static GLuint link_program(GLuint vs, GLuint fs) {
    GLuint p = glCreateProgram();
    glAttachShader(p, vs);
    glAttachShader(p, fs);
    glBindAttribLocation(p, 0, "a_pos");
    glBindAttribLocation(p, 1, "a_uv");
    glLinkProgram(p);
    GLint ok = GL_FALSE;
    glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (ok != GL_TRUE) {
        char log[512];
        glGetProgramInfoLog(p, sizeof(log), nullptr, log);
        spdlog::error("VideoDisplay program link error: {}", log);
        glDeleteProgram(p);
        return 0;
    }
    return p;
}

// ---------------------------------------------------------------------------
// DRM helper: export a GBM BO from the render node as a DMA-buf prime fd,
// import it into the KMS device (drm_fd_), and register a DRM framebuffer.
//
// This bridges the split RPi 5 architecture:
//   renderD128 (v3d) → GBM/EGL render → DMA-buf export
//   card1 (vc4-drm)  → DMA-buf import → drmModeAddFB2 → page-flip
//
// Returns fb_id on success, 0 on failure.
// Sets *out_gem_handle to the imported GEM handle; caller must call
// drmCloseBufferHandle(drm_fd_, *out_gem_handle) after the flip.
// ---------------------------------------------------------------------------
uint32_t VideoDisplay::import_bo_as_fb(struct gbm_bo* bo, uint32_t* out_gem_handle) {
    int prime_fd = gbm_bo_get_fd(bo);
    if (prime_fd < 0) {
        spdlog::error("VideoDisplay: gbm_bo_get_fd failed");
        return 0;
    }

    uint32_t gem_handle = 0;
    if (drmPrimeFDToHandle(drm_fd_, prime_fd, &gem_handle) != 0) {
        spdlog::error("VideoDisplay: drmPrimeFDToHandle failed: {}", strerror(errno));
        close(prime_fd);
        return 0;
    }
    close(prime_fd);

    uint32_t w      = gbm_bo_get_width(bo);
    uint32_t h      = gbm_bo_get_height(bo);
    uint32_t stride = gbm_bo_get_stride(bo);

    uint32_t handles[4] = { gem_handle, 0, 0, 0 };
    uint32_t pitches[4] = { stride,     0, 0, 0 };
    uint32_t offsets[4] = { 0,          0, 0, 0 };
    uint32_t fb_id = 0;

    if (drmModeAddFB2(drm_fd_, w, h, DRM_FORMAT_XRGB8888,
                      handles, pitches, offsets, &fb_id, 0) != 0) {
        spdlog::error("VideoDisplay: drmModeAddFB2 failed: {}", strerror(errno));
        drmCloseBufferHandle(drm_fd_, gem_handle);
        return 0;
    }

    *out_gem_handle = gem_handle;
    return fb_id;
}

// ---------------------------------------------------------------------------
// init_drm
// ---------------------------------------------------------------------------
bool VideoDisplay::init_drm(const char* node, uint32_t want_w, uint32_t want_h,
                             uint32_t want_refresh) {
    drm_fd_ = open(node, O_RDWR | O_CLOEXEC);
    if (drm_fd_ < 0) {
        spdlog::error("VideoDisplay: cannot open {}: {}", node, strerror(errno));
        return false;
    }

    drmSetClientCap(drm_fd_, DRM_CLIENT_CAP_UNIVERSAL_PLANES, 1);

    // Attempt to acquire DRM master (needed for modeset).
    // Fails gracefully if a compositor holds master — caller gets a clear error
    // from drmModeSetCrtc later. To take over the display, stop the compositor
    // first (e.g. `systemctl --user stop labwc`).
    if (drmSetMaster(drm_fd_) != 0) {
        spdlog::warn("VideoDisplay: drmSetMaster failed ({}). "
                     "If a Wayland/X11 compositor is running, stop it first.",
                     strerror(errno));
    }

    drmModeRes* res = drmModeGetResources(drm_fd_);
    if (!res) {
        spdlog::error("VideoDisplay: drmModeGetResources failed");
        return false;
    }

    bool found = false;
    for (int i = 0; i < res->count_connectors && !found; i++) {
        drmModeConnector* conn = drmModeGetConnector(drm_fd_, res->connectors[i]);
        if (!conn) continue;
        if (conn->connection != DRM_MODE_CONNECTED || conn->count_modes == 0) {
            drmModeFreeConnector(conn);
            continue;
        }

        // Find CRTC for this connector.
        drmModeEncoder* enc = nullptr;
        if (conn->encoder_id)
            enc = drmModeGetEncoder(drm_fd_, conn->encoder_id);

        uint32_t crtc_id = 0;
        if (enc && enc->crtc_id) {
            crtc_id = enc->crtc_id;
        } else {
            for (int ei = 0; ei < conn->count_encoders && !crtc_id; ei++) {
                enc = drmModeGetEncoder(drm_fd_, conn->encoders[ei]);
                if (!enc) continue;
                for (int ci = 0; ci < res->count_crtcs && !crtc_id; ci++) {
                    if (enc->possible_crtcs & (1u << ci))
                        crtc_id = res->crtcs[ci];
                }
                drmModeFreeEncoder(enc);
                enc = nullptr;
            }
        }
        if (enc) drmModeFreeEncoder(enc);

        if (!crtc_id) {
            drmModeFreeConnector(conn);
            continue;
        }

        // Pick display mode.
        int mode_idx = 0;
        if (want_w > 0 && want_h > 0) {
            int best = -1;
            for (int m = 0; m < conn->count_modes; m++) {
                drmModeModeInfo& mi = conn->modes[m];
                if (mi.hdisplay == want_w && mi.vdisplay == want_h &&
                    (want_refresh == 0 || mi.vrefresh == want_refresh)) {
                    if (best < 0 || !(mi.flags & DRM_MODE_FLAG_INTERLACE))
                        best = m;
                }
            }
            if (best >= 0) mode_idx = best;
        }

        mode_         = conn->modes[mode_idx];
        crtc_id_      = crtc_id;
        connector_id_ = conn->connector_id;
        saved_crtc_   = drmModeGetCrtc(drm_fd_, crtc_id_);
        found = true;

        spdlog::info("VideoDisplay: using connector {} crtc {} mode {}x{}@{}",
                     connector_id_, crtc_id_,
                     mode_.hdisplay, mode_.vdisplay, mode_.vrefresh);
        drmModeFreeConnector(conn);
    }
    drmModeFreeResources(res);

    if (!found) {
        spdlog::error("VideoDisplay: no connected display found");
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// init_gbm — opens the v3d render node and creates a GBM rendering surface.
// No GBM_BO_USE_SCANOUT: buffers are shared with the KMS device via DMA-buf.
// ---------------------------------------------------------------------------
bool VideoDisplay::init_gbm(const char* render_node) {
    render_fd_ = open(render_node, O_RDWR | O_CLOEXEC);
    if (render_fd_ < 0) {
        spdlog::error("VideoDisplay: cannot open render node {}: {}",
                      render_node, strerror(errno));
        return false;
    }

    gbm_dev_ = gbm_create_device(render_fd_);
    if (!gbm_dev_) {
        spdlog::error("VideoDisplay: gbm_create_device failed on {}", render_node);
        return false;
    }

    // GBM_FORMAT_XRGB8888: opaque format; vc4 scanout plane supports it.
    // GBM_BO_USE_LINEAR: force linear memory layout so vc4-drm can scan out
    // the buffer correctly after DMA-buf import from the v3d render node.
    // Without this the V3D GPU may use a tiled layout that vc4-drm misinterprets,
    // producing horizontal colored band artifacts.
    gbm_surf_ = gbm_surface_create(gbm_dev_,
                                   mode_.hdisplay, mode_.vdisplay,
                                   GBM_FORMAT_XRGB8888,
                                   GBM_BO_USE_RENDERING | GBM_BO_USE_LINEAR);
    if (!gbm_surf_) {
        spdlog::error("VideoDisplay: gbm_surface_create failed");
        return false;
    }
    spdlog::info("VideoDisplay: GBM surface {}x{} on {}", mode_.hdisplay, mode_.vdisplay, render_node);
    return true;
}

// ---------------------------------------------------------------------------
// init_egl
// ---------------------------------------------------------------------------
bool VideoDisplay::init_egl() {
    // EGL display from GBM device.
    PFNEGLGETPLATFORMDISPLAYEXTPROC get_platform_display =
        (PFNEGLGETPLATFORMDISPLAYEXTPROC)eglGetProcAddress("eglGetPlatformDisplayEXT");

    egl_dpy_ = get_platform_display
        ? get_platform_display(EGL_PLATFORM_GBM_KHR, gbm_dev_, nullptr)
        : eglGetDisplay((EGLNativeDisplayType)gbm_dev_);

    if (egl_dpy_ == EGL_NO_DISPLAY) {
        spdlog::error("VideoDisplay: eglGetDisplay failed");
        return false;
    }
    if (!eglInitialize(egl_dpy_, nullptr, nullptr)) {
        spdlog::error("VideoDisplay: eglInitialize failed");
        return false;
    }
    eglBindAPI(EGL_OPENGL_ES_API);

    const EGLint cfg_attrs[] = {
        EGL_SURFACE_TYPE,    EGL_WINDOW_BIT,
        EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
        EGL_RED_SIZE,   8,
        EGL_GREEN_SIZE, 8,
        EGL_BLUE_SIZE,  8,
        EGL_ALPHA_SIZE, 8,
        EGL_NONE
    };
    EGLint n = 0;
    if (!eglChooseConfig(egl_dpy_, cfg_attrs, &egl_cfg_, 1, &n) || n == 0) {
        spdlog::error("VideoDisplay: eglChooseConfig failed");
        return false;
    }

    const EGLint ctx_attrs[] = { EGL_CONTEXT_CLIENT_VERSION, 2, EGL_NONE };
    egl_ctx_ = eglCreateContext(egl_dpy_, egl_cfg_, EGL_NO_CONTEXT, ctx_attrs);
    if (egl_ctx_ == EGL_NO_CONTEXT) {
        spdlog::error("VideoDisplay: eglCreateContext failed (0x{:x})", eglGetError());
        return false;
    }

    egl_surf_ = eglCreateWindowSurface(egl_dpy_, egl_cfg_,
                                       (EGLNativeWindowType)gbm_surf_, nullptr);
    if (egl_surf_ == EGL_NO_SURFACE) {
        spdlog::error("VideoDisplay: eglCreateWindowSurface failed (0x{:x})", eglGetError());
        return false;
    }

    if (!eglMakeCurrent(egl_dpy_, egl_surf_, egl_surf_, egl_ctx_)) {
        spdlog::error("VideoDisplay: eglMakeCurrent failed (0x{:x})", eglGetError());
        return false;
    }

    spdlog::info("VideoDisplay: EGL ready — GL vendor={} renderer={}",
                 (const char*)glGetString(GL_VENDOR),
                 (const char*)glGetString(GL_RENDERER));
    return true;
}

// ---------------------------------------------------------------------------
// init_gl
// ---------------------------------------------------------------------------
bool VideoDisplay::init_gl() {
    GLuint vs = compile_shader(GL_VERTEX_SHADER,   kVert);
    GLuint fs = compile_shader(GL_FRAGMENT_SHADER, kFrag);
    if (!vs || !fs) return false;
    prog_ = link_program(vs, fs);
    glDeleteShader(vs);
    glDeleteShader(fs);
    if (!prog_) return false;

    loc_y_   = glGetUniformLocation(prog_, "u_y");
    loc_uv_  = glGetUniformLocation(prog_, "u_uv");
    loc_osd_ = glGetUniformLocation(prog_, "u_osd");

    // Y texture: full-resolution luminance.
    glGenTextures(1, &y_tex_);
    glBindTexture(GL_TEXTURE_2D, y_tex_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    // UV texture: half-resolution interleaved chroma (Cb in R, Cr in A).
    glGenTextures(1, &uv_tex_);
    glBindTexture(GL_TEXTURE_2D, uv_tex_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    glGenTextures(1, &osd_tex_);
    glBindTexture(GL_TEXTURE_2D, osd_tex_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    // Seed OSD texture with transparent black so the first frame before any
    // OSD data arrives is just the video.
    std::vector<uint8_t> blank(mode_.hdisplay * mode_.vdisplay * 4, 0);
    glBindTexture(GL_TEXTURE_2D, osd_tex_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA,
                 mode_.hdisplay, mode_.vdisplay, 0,
                 GL_RGBA, GL_UNSIGNED_BYTE, blank.data());

    // Seed Y and UV textures with dark blue (NV12: Y=41, Cb=110, Cr=240).
    // Visible indicator that rendering is active while waiting for first frame.
    std::vector<uint8_t> y_seed(mode_.hdisplay * mode_.vdisplay, 41);
    glBindTexture(GL_TEXTURE_2D, y_tex_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE,
                 mode_.hdisplay, mode_.vdisplay, 0,
                 GL_LUMINANCE, GL_UNSIGNED_BYTE, y_seed.data());

    // UV seed: interleaved Cb=110, Cr=240 → dark blue in BT.601.
    std::vector<uint8_t> uv_seed((mode_.hdisplay / 2) * (mode_.vdisplay / 2) * 2);
    for (size_t i = 0; i < uv_seed.size(); i += 2) { uv_seed[i] = 110; uv_seed[i+1] = 240; }
    glBindTexture(GL_TEXTURE_2D, uv_tex_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE_ALPHA,
                 mode_.hdisplay / 2, mode_.vdisplay / 2, 0,
                 GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, uv_seed.data());

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glViewport(0, 0, mode_.hdisplay, mode_.vdisplay);
    return true;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
bool VideoDisplay::init(const char* drm_node, uint32_t mode_w, uint32_t mode_h,
                        uint32_t mode_refresh) {
    if (!init_drm(drm_node, mode_w, mode_h, mode_refresh)) return false;
    // RPi 5: card1 = vc4-drm KMS display; renderD128 = v3d GPU.
    // We render on renderD128 and share buffers with card1 via DMA-buf.
    if (!init_gbm("/dev/dri/renderD128")) return false;
    if (!init_egl()) return false;
    if (!init_gl())  return false;
    // Release context from this (main) thread so render() can acquire it
    // on the display thread.
    eglMakeCurrent(egl_dpy_, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    return true;
}

void VideoDisplay::push_video_frame(const uint8_t* nv12, int w, int h) {
    std::lock_guard<std::mutex> lk(video_mtx_);
    video_w_ = w;
    video_h_ = h;
    video_pending_.assign(nv12, nv12 + (size_t)w * h * 3 / 2);
    video_dirty_ = true;
}

void VideoDisplay::push_osd_frame(const uint8_t* argb, int w, int h, int stride) {
    std::lock_guard<std::mutex> lk(osd_mtx_);
    osd_w_ = w;
    osd_h_ = h;
    // Convert BGRA (DRM ARGB8888 on little-endian) → RGBA for GL upload.
    osd_pending_.resize((size_t)w * h * 4);
    for (int y = 0; y < h; y++) {
        const uint8_t* src = argb + y * stride;
        uint8_t*       dst = osd_pending_.data() + (size_t)y * w * 4;
        for (int x = 0; x < w; x++) {
            dst[0] = src[2]; // R ← B
            dst[1] = src[1]; // G
            dst[2] = src[0]; // B ← R
            dst[3] = src[3]; // A
            src += 4; dst += 4;
        }
    }
    osd_dirty_ = true;
}

bool VideoDisplay::render() {
    // Ensure EGL context is current on this thread.
    // init() releases the context after GL setup so it can be acquired here
    // (on the dedicated display thread) on the first call.
    if (eglGetCurrentContext() != egl_ctx_) {
        if (!eglMakeCurrent(egl_dpy_, egl_surf_, egl_surf_, egl_ctx_)) {
            spdlog::error("VideoDisplay: eglMakeCurrent failed: 0x{:x}", eglGetError());
            return false;
        }
    }

    // Upload any pending NV12 video frame (Y plane then UV plane).
    {
        std::lock_guard<std::mutex> lk(video_mtx_);
        if (video_dirty_ && video_w_ > 0 && video_h_ > 0) {
            const uint8_t* y_plane  = video_pending_.data();
            const uint8_t* uv_plane = y_plane + (size_t)video_w_ * video_h_;

            glBindTexture(GL_TEXTURE_2D, y_tex_);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE,
                         video_w_, video_h_, 0,
                         GL_LUMINANCE, GL_UNSIGNED_BYTE, y_plane);

            glBindTexture(GL_TEXTURE_2D, uv_tex_);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE_ALPHA,
                         video_w_ / 2, video_h_ / 2, 0,
                         GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, uv_plane);

            video_dirty_ = false;
        }
    }

    // Upload any pending OSD frame.
    {
        std::lock_guard<std::mutex> lk(osd_mtx_);
        if (osd_dirty_ && osd_w_ > 0 && osd_h_ > 0) {
            glBindTexture(GL_TEXTURE_2D, osd_tex_);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA,
                         osd_w_, osd_h_, 0,
                         GL_RGBA, GL_UNSIGNED_BYTE, osd_pending_.data());
            osd_dirty_ = false;
        }
    }

    // Draw.
    glClear(GL_COLOR_BUFFER_BIT);
    glUseProgram(prog_);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, y_tex_);
    glUniform1i(loc_y_, 0);

    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, uv_tex_);
    glUniform1i(loc_uv_, 1);

    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_2D, osd_tex_);
    glUniform1i(loc_osd_, 2);

    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE,
                          4 * sizeof(GLfloat), kQuadVerts);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE,
                          4 * sizeof(GLfloat), kQuadVerts + 2);
    glEnableVertexAttribArray(1);

    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

    eglSwapBuffers(egl_dpy_, egl_surf_);

    // Lock the front GBM BO and flip to DRM via DMA-buf import.
    struct gbm_bo* bo = gbm_surface_lock_front_buffer(gbm_surf_);
    if (!bo) {
        spdlog::error("VideoDisplay: gbm_surface_lock_front_buffer failed");
        return false;
    }

    // Look up (or register) the DRM framebuffer for this BO.
    // GBM surfaces cycle through exactly 2 BOs so this cache hit rate is ~100%
    // after the first two frames, eliminating 4+ DRM syscalls per frame.
    uint32_t fb_id = 0;
    auto it = bo_fb_cache_.find(bo);
    if (it != bo_fb_cache_.end()) {
        fb_id = it->second.first;
    } else {
        uint32_t gem_handle = 0;
        fb_id = import_bo_as_fb(bo, &gem_handle);
        if (!fb_id) {
            gbm_surface_release_buffer(gbm_surf_, bo);
            return false;
        }
        bo_fb_cache_[bo] = { fb_id, gem_handle };
    }

    if (first_frame_) {
        // Initial modeset.
        if (drmModeSetCrtc(drm_fd_, crtc_id_, fb_id, 0, 0,
                           &connector_id_, 1, &mode_) != 0) {
            spdlog::error("VideoDisplay: drmModeSetCrtc failed: {}", strerror(errno));
            gbm_surface_release_buffer(gbm_surf_, bo);
            return false;
        }
        first_frame_ = false;
    } else {
        drmModePageFlip(drm_fd_, crtc_id_, fb_id,
                        DRM_MODE_PAGE_FLIP_EVENT, nullptr);
        // Wait for the flip event so we don't queue up more than one.
        drmEventContext ev{};
        ev.version = DRM_EVENT_CONTEXT_VERSION;
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(drm_fd_, &fds);
        struct timeval tv{ .tv_sec = 0, .tv_usec = 50000 };
        if (select(drm_fd_ + 1, &fds, nullptr, nullptr, &tv) > 0)
            drmHandleEvent(drm_fd_, &ev);
    }

    // Release the previous BO back to GBM; its fb_id stays alive in the cache.
    if (prev_bo_)
        gbm_surface_release_buffer(gbm_surf_, prev_bo_);
    prev_bo_ = bo;

    return true;
}

// ---------------------------------------------------------------------------
// Cleanup
// ---------------------------------------------------------------------------
VideoDisplay::~VideoDisplay() {
    cleanup();
}

void VideoDisplay::cleanup() {
    if (egl_dpy_ != EGL_NO_DISPLAY)
        eglMakeCurrent(egl_dpy_, egl_surf_, egl_surf_, egl_ctx_);

    if (y_tex_)  { glDeleteTextures(1, &y_tex_);  y_tex_  = 0; }
    if (uv_tex_) { glDeleteTextures(1, &uv_tex_); uv_tex_ = 0; }
    if (osd_tex_)   { glDeleteTextures(1, &osd_tex_);   osd_tex_   = 0; }
    if (prog_)      { glDeleteProgram(prog_);            prog_      = 0; }

    if (egl_surf_ != EGL_NO_SURFACE) {
        eglDestroySurface(egl_dpy_, egl_surf_); egl_surf_ = EGL_NO_SURFACE;
    }
    if (egl_ctx_ != EGL_NO_CONTEXT) {
        eglDestroyContext(egl_dpy_, egl_ctx_); egl_ctx_ = EGL_NO_CONTEXT;
    }
    if (egl_dpy_ != EGL_NO_DISPLAY) {
        eglTerminate(egl_dpy_); egl_dpy_ = EGL_NO_DISPLAY;
    }

    if (prev_bo_) {
        gbm_surface_release_buffer(gbm_surf_, prev_bo_);
        prev_bo_ = nullptr;
    }
    for (auto& [bo, fb_gem] : bo_fb_cache_) {
        drmModeRmFB(drm_fd_, fb_gem.first);
        drmCloseBufferHandle(drm_fd_, fb_gem.second);
    }
    bo_fb_cache_.clear();
    if (gbm_surf_) { gbm_surface_destroy(gbm_surf_); gbm_surf_ = nullptr; }
    if (gbm_dev_)  { gbm_device_destroy(gbm_dev_);   gbm_dev_  = nullptr; }
    if (render_fd_ >= 0) { close(render_fd_); render_fd_ = -1; }

    // Restore original CRTC.
    if (saved_crtc_ && drm_fd_ >= 0) {
        drmModeSetCrtc(drm_fd_, saved_crtc_->crtc_id,
                       saved_crtc_->buffer_id, saved_crtc_->x, saved_crtc_->y,
                       &connector_id_, 1, &saved_crtc_->mode);
        drmModeFreeCrtc(saved_crtc_); saved_crtc_ = nullptr;
    }

    if (drm_fd_ >= 0) { close(drm_fd_); drm_fd_ = -1; }
}
