/*
 * Drive the emulated VideoCore IV through the real Mesa vc4 driver.
 *
 * Renders off-screen with EGL's surfaceless platform into a framebuffer
 * object, so no display, no window system and no KMS are involved -- which
 * suits a model that has a GPU but none of the display blocks.
 *
 * Stages, each reported separately so a failure says how far the stack got:
 *   1. EGL initialises and a GLES2 context goes current on the vc4 driver
 *   2. glClear of a known colour, read back with glReadPixels
 *   3. a real compiled GLSL triangle, read back the same way
 *
 * Stage 3 is expected to fail its pixel check until the model has a
 * rasterizer; it is here because it makes Mesa emit genuine shaders, control
 * lists and VPM traffic for the model to be measured against.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <EGL/egl.h>
#include <GLES2/gl2.h>

#define W 16
#define H 16
/* Bigger than one 64x64 tile, and not a whole number of them. */
#define BIG 96
#define GL_RGBA8_OES 0x8058

static int failures;

static void report(const char *stage, int ok, const char *detail)
{
    printf("GLES: %-22s %s%s%s\n", stage, ok ? "PASS" : "FAIL",
           detail && *detail ? " -- " : "", detail ? detail : "");
    fflush(stdout);
    if (!ok) {
        failures++;
    }
}

static const char *vs_src =
    "attribute vec4 pos;\n"
    "void main() { gl_Position = pos; }\n";

static const char *fs_src =
    "precision mediump float;\n"
    "void main() { gl_FragColor = vec4(1.0, 0.0, 0.0, 1.0); }\n";

static GLuint compile(GLenum type, const char *src)
{
    GLuint s = glCreateShader(type);
    GLint ok = 0;

    glShaderSource(s, 1, &src, NULL);
    glCompileShader(s);
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[512] = { 0 };

        glGetShaderInfoLog(s, sizeof(log) - 1, NULL, log);
        printf("GLES: shader compile failed: %s\n", log);
        return 0;
    }
    return s;
}

int main(void)
{
    EGLDisplay dpy;
    EGLContext ctx;
    EGLConfig cfg;
    EGLint major, minor, n;
    GLuint fbo, rb, prog, vbo;
    unsigned char px[W * H * 4];
    static const EGLint cfg_attrs[] = {
        EGL_SURFACE_TYPE, EGL_PBUFFER_BIT,
        EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
        EGL_RED_SIZE, 8, EGL_GREEN_SIZE, 8, EGL_BLUE_SIZE, 8, EGL_ALPHA_SIZE, 8,
        EGL_NONE
    };
    static const EGLint ctx_attrs[] = {
        EGL_CONTEXT_CLIENT_VERSION, 2, EGL_NONE
    };
    static const GLfloat tri[] = {
        -0.9f, -0.9f, 0.0f, 1.0f,
         0.9f, -0.9f, 0.0f, 1.0f,
         0.0f,  0.9f, 0.0f, 1.0f,
    };

    /* Mesa picks its platform from the environment; surfaceless needs only a
     * render node. */
    setenv("EGL_PLATFORM", "surfaceless", 1);

    dpy = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    if (dpy == EGL_NO_DISPLAY) {
        report("egl display", 0, "eglGetDisplay returned EGL_NO_DISPLAY");
        return 1;
    }
    if (!eglInitialize(dpy, &major, &minor)) {
        report("egl init", 0, "eglInitialize failed");
        return 1;
    }
    printf("GLES: EGL %d.%d, vendor %s\n", major, minor,
           eglQueryString(dpy, EGL_VENDOR));

    if (!eglBindAPI(EGL_OPENGL_ES_API) ||
        !eglChooseConfig(dpy, cfg_attrs, &cfg, 1, &n) || n < 1) {
        report("egl config", 0, "no ES2 config");
        return 1;
    }
    ctx = eglCreateContext(dpy, cfg, EGL_NO_CONTEXT, ctx_attrs);
    if (ctx == EGL_NO_CONTEXT) {
        report("egl context", 0, "eglCreateContext failed");
        return 1;
    }
    if (!eglMakeCurrent(dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, ctx)) {
        report("egl make current", 0, "needs EGL_KHR_surfaceless_context");
        return 1;
    }

    printf("GLES: GL_VENDOR   = %s\n", glGetString(GL_VENDOR));
    printf("GLES: GL_RENDERER = %s\n", glGetString(GL_RENDERER));
    printf("GLES: GL_VERSION  = %s\n", glGetString(GL_VERSION));
    fflush(stdout);
    report("context on vc4", 1, (const char *)glGetString(GL_RENDERER));

    /* An off-screen colour target. */
    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    glGenRenderbuffers(1, &rb);
    glBindRenderbuffer(GL_RENDERBUFFER, rb);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8_OES, W, H);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                              GL_RENDERBUFFER, rb);
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        report("framebuffer object", 0, "incomplete");
        return 1;
    }
    report("framebuffer object", 1, "16x16 RGBA8");

    /* Stage 2: a clear.  This needs the tile buffer and the tile store, but
     * neither shading nor rasterization. */
    glViewport(0, 0, W, H);
    glClearColor(0.0f, 1.0f, 0.0f, 1.0f);       /* opaque green */
    glClear(GL_COLOR_BUFFER_BIT);
    glFinish();

    memset(px, 0x5a, sizeof(px));
    glReadPixels(0, 0, W, H, GL_RGBA, GL_UNSIGNED_BYTE, px);
    {
        unsigned i, bad = 0;

        for (i = 0; i < W * H; i++) {
            const unsigned char *p = px + i * 4;

            if (p[0] != 0x00 || p[1] != 0xff || p[2] != 0x00 || p[3] != 0xff) {
                bad++;
            }
        }
        if (bad) {
            char detail[96];

            snprintf(detail, sizeof(detail),
                     "%u/%u pixels wrong, first is %02x%02x%02x%02x",
                     bad, W * H, px[0], px[1], px[2], px[3]);
            report("glClear readback", 0, detail);
        } else {
            report("glClear readback", 1, "all pixels opaque green");
        }
    }

    /* Stage 3: a real compiled shader and a real draw. */
    {
        GLuint vs = compile(GL_VERTEX_SHADER, vs_src);
        GLuint fs = compile(GL_FRAGMENT_SHADER, fs_src);
        GLint linked = 0;

        if (!vs || !fs) {
            report("glsl compile", 0, "see log above");
            goto done;
        }
        report("glsl compile", 1, "vertex + fragment");

        prog = glCreateProgram();
        glAttachShader(prog, vs);
        glAttachShader(prog, fs);
        glBindAttribLocation(prog, 0, "pos");
        glLinkProgram(prog);
        glGetProgramiv(prog, GL_LINK_STATUS, &linked);
        if (!linked) {
            char log[512] = { 0 };

            glGetProgramInfoLog(prog, sizeof(log) - 1, NULL, log);
            report("glsl link", 0, log);
            goto done;
        }
        report("glsl link", 1, "program linked");
        glUseProgram(prog);

        glGenBuffers(1, &vbo);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof(tri), tri, GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0);

        glClearColor(0.0f, 0.0f, 1.0f, 1.0f);   /* blue background */
        glClear(GL_COLOR_BUFFER_BIT);
        glDrawArrays(GL_TRIANGLES, 0, 3);
        glFinish();
        report("draw submitted", glGetError() == GL_NO_ERROR,
               "glDrawArrays + glFinish");

        memset(px, 0x5a, sizeof(px));
        glReadPixels(0, 0, W, H, GL_RGBA, GL_UNSIGNED_BYTE, px);
        {
            /* The middle of the viewport is inside the triangle. */
            const unsigned char *c = px + ((H / 2) * W + W / 2) * 4;
            int is_red = c[0] == 0xff && c[1] == 0x00 && c[2] == 0x00;
            char detail[96];

            snprintf(detail, sizeof(detail),
                     "centre pixel %02x%02x%02x%02x (want ff0000ff)",
                     c[0], c[1], c[2], c[3]);
            report("triangle rasterized", is_red, detail);
        }
        {
            /*
             * One pixel proves nothing on its own -- a rasterizer that covered
             * the whole surface would pass it too.  The triangle has its apex
             * at the top centre and its base along the bottom, so the bottom
             * corners are inside and the top corners are not, and it should
             * cover roughly half the pixels.
             */
            unsigned i, red = 0;
            const unsigned char *tl = px + (0 * W + 0) * 4;
            const unsigned char *tr = px + (0 * W + (W - 1)) * 4;
            char detail[96];

            for (i = 0; i < W * H; i++) {
                if (px[i * 4] == 0xff && px[i * 4 + 1] == 0x00 &&
                    px[i * 4 + 2] == 0x00) {
                    red++;
                }
            }
            snprintf(detail, sizeof(detail),
                     "%u/%u pixels covered, corners %02x%02x%02x/%02x%02x%02x",
                     red, W * H, tl[0], tl[1], tl[2], tr[0], tr[1], tr[2]);
            report("triangle has a shape",
                   red > (W * H) / 5 && red < (W * H) * 3 / 4 &&
                   tl[2] == 0xff && tr[2] == 0xff,
                   detail);
        }
    }

    /*
     * Stage 4: a surface bigger than one 64x64 tile, so the render list loops
     * over several of them and the model has to place each tile's pixels
     * correctly (and clip the partial ones at the edges).
     */
    {
        GLuint big_fbo, big_rb;
        unsigned char *big = malloc(BIG * BIG * 4);
        unsigned x2, y2, red = 0, bad_corner = 0;

        if (!big) {
            report("multi-tile surface", 0, "out of memory");
            goto done;
        }
        glGenFramebuffers(1, &big_fbo);
        glBindFramebuffer(GL_FRAMEBUFFER, big_fbo);
        glGenRenderbuffers(1, &big_rb);
        glBindRenderbuffer(GL_RENDERBUFFER, big_rb);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8_OES, BIG, BIG);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                                  GL_RENDERBUFFER, big_rb);
        if (glCheckFramebufferStatus(GL_FRAMEBUFFER) !=
            GL_FRAMEBUFFER_COMPLETE) {
            report("multi-tile surface", 0, "framebuffer incomplete");
            free(big);
            goto done;
        }

        glViewport(0, 0, BIG, BIG);
        glClearColor(0.0f, 0.0f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        glDrawArrays(GL_TRIANGLES, 0, 3);
        glFinish();

        memset(big, 0x5a, BIG * BIG * 4);
        glReadPixels(0, 0, BIG, BIG, GL_RGBA, GL_UNSIGNED_BYTE, big);
        for (y2 = 0; y2 < BIG; y2++) {
            for (x2 = 0; x2 < BIG; x2++) {
                const unsigned char *p = big + (y2 * BIG + x2) * 4;

                if (p[0] == 0xff && p[1] == 0x00 && p[2] == 0x00) {
                    red++;
                }
            }
        }
        /* The top two corners are outside the triangle whichever tile they
         * fall in. */
        if (big[2] != 0xff) {
            bad_corner++;
        }
        if (big[((BIG - 1)) * 4 + 2] != 0xff) {
            bad_corner++;
        }
        {
            char detail[96];
            unsigned total = BIG * BIG;

            snprintf(detail, sizeof(detail),
                     "%ux%u = %u tiles, %u/%u covered, %u bad corners",
                     BIG, BIG, ((BIG + 63) / 64) * ((BIG + 63) / 64),
                     red, total, bad_corner);
            report("multi-tile triangle",
                   red > total / 5 && red < total * 3 / 4 && !bad_corner,
                   detail);
        }
        free(big);
        glBindFramebuffer(GL_FRAMEBUFFER, fbo);
        glViewport(0, 0, W, H);
    }

    /*
     * Stage 5: an interpolated varying.  Each vertex gets its own colour and
     * the fragment shader reads the interpolated value, so the result has to
     * change across the triangle rather than being flat.
     */
    {
        static const char *vary_vs =
            "attribute vec4 pos;\n"
            "attribute vec4 col;\n"
            "varying vec4 vcol;\n"
            "void main() { gl_Position = pos; vcol = col; }\n";
        static const char *vary_fs =
            "precision mediump float;\n"
            "varying vec4 vcol;\n"
            "void main() { gl_FragColor = vcol; }\n";
        static const GLfloat colours[] = {
            1.0f, 0.0f, 0.0f, 1.0f,
            0.0f, 1.0f, 0.0f, 1.0f,
            0.0f, 0.0f, 1.0f, 1.0f,
        };
        GLuint vs2 = compile(GL_VERTEX_SHADER, vary_vs);
        GLuint fs2 = compile(GL_FRAGMENT_SHADER, vary_fs);
        GLuint prog2, cbo;
        GLint linked2 = 0;

        if (!vs2 || !fs2) {
            report("varying shaders compile", 0, "see log above");
            goto done;
        }
        prog2 = glCreateProgram();
        glAttachShader(prog2, vs2);
        glAttachShader(prog2, fs2);
        glBindAttribLocation(prog2, 0, "pos");
        glBindAttribLocation(prog2, 1, "col");
        glLinkProgram(prog2);
        glGetProgramiv(prog2, GL_LINK_STATUS, &linked2);
        if (!linked2) {
            char log[512] = { 0 };

            glGetProgramInfoLog(prog2, sizeof(log) - 1, NULL, log);
            report("varying shaders link", 0, log);
            goto done;
        }
        glUseProgram(prog2);

        glGenBuffers(1, &cbo);
        glBindBuffer(GL_ARRAY_BUFFER, cbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof(colours), colours,
                     GL_STATIC_DRAW);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 0, 0);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0);

        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        glDrawArrays(GL_TRIANGLES, 0, 3);
        glFinish();

        memset(px, 0x5a, sizeof(px));
        glReadPixels(0, 0, W, H, GL_RGBA, GL_UNSIGNED_BYTE, px);
        {
            /*
             * The triangle's wide edge runs along row 3 (its apex is at the
             * opposite side), so both of these are inside it: one near the
             * red vertex, one near the green one.  They must differ.
             */
            const unsigned char *l = px + (3 * W + 3) * 4;
            const unsigned char *r2 = px + (3 * W + (W - 4)) * 4;
            char detail[128];
            /*
             * "They differ" is too weak -- it passes for almost any wrong
             * answer.  These two points are mirror images about the centre,
             * so the left must lean red exactly as much as the right leans
             * green, and both must carry the same little of the blue vertex.
             */
            int ok = l[0] > l[1] + 0x40 &&          /* left leans red   */
                     r2[1] > r2[0] + 0x40 &&        /* right leans green */
                     abs((int)l[0] - (int)r2[1]) <= 4 &&
                     abs((int)l[1] - (int)r2[0]) <= 4 &&
                     abs((int)l[2] - (int)r2[2]) <= 4 &&
                     l[2] > 0x10 && l[2] < 0x60;    /* a little blue     */

            snprintf(detail, sizeof(detail),
                     "left %02x%02x%02x, right %02x%02x%02x (mirrored)",
                     l[0], l[1], l[2], r2[0], r2[1], r2[2]);
            report("varying interpolated", ok, detail);
        }
    }

    /*
     * Stage 6: face culling.  The triangle is front-facing, so culling front
     * faces must remove it entirely and leave the clear colour.
     */
    {
        const unsigned char *c;
        char detail[96];

        glUseProgram(prog);
        glDisableVertexAttribArray(1);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0);

        unsigned char culled_front, kept_back;

        (void)c;
        glEnable(GL_CULL_FACE);

        /* The triangle is front facing, so culling fronts must remove it. */
        glCullFace(GL_FRONT);
        glClearColor(0.0f, 0.0f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        glDrawArrays(GL_TRIANGLES, 0, 3);
        glFinish();
        memset(px, 0x5a, sizeof(px));
        glReadPixels(0, 0, W, H, GL_RGBA, GL_UNSIGNED_BYTE, px);
        culled_front = px[((H / 2) * W + W / 2) * 4];      /* red channel */

        /* ... and culling backs must leave it alone.  Checking both ways
         * means a model that has the winding sense backwards cannot pass. */
        glCullFace(GL_BACK);
        glClear(GL_COLOR_BUFFER_BIT);
        glDrawArrays(GL_TRIANGLES, 0, 3);
        glFinish();
        memset(px, 0x5a, sizeof(px));
        glReadPixels(0, 0, W, H, GL_RGBA, GL_UNSIGNED_BYTE, px);
        kept_back = px[((H / 2) * W + W / 2) * 4];
        glDisable(GL_CULL_FACE);

        snprintf(detail, sizeof(detail),
                 "cull front -> red %02x (want 00), cull back -> red %02x (want ff)",
                 culled_front, kept_back);
        report("face culling", culled_front == 0x00 && kept_back == 0xff,
               detail);
    }

    /*
     * Stage 7: blending.  Half-transparent red over a blue clear should land
     * halfway between the two.
     */
    {
        static const char *blend_fs_src =
            "precision mediump float;\n"
            "void main() { gl_FragColor = vec4(1.0, 0.0, 0.0, 0.5); }\n";
        GLuint bfs = compile(GL_FRAGMENT_SHADER, blend_fs_src);
        GLuint bprog;
        GLint linked3 = 0;
        const unsigned char *c;
        char detail[96];

        if (!bfs) {
            report("blend shader compiles", 0, "see log above");
            goto done;
        }
        bprog = glCreateProgram();
        glAttachShader(bprog, compile(GL_VERTEX_SHADER, vs_src));
        glAttachShader(bprog, bfs);
        glBindAttribLocation(bprog, 0, "pos");
        glLinkProgram(bprog);
        glGetProgramiv(bprog, GL_LINK_STATUS, &linked3);
        if (!linked3) {
            report("blend shader links", 0, "link failed");
            goto done;
        }
        glUseProgram(bprog);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0);

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glClearColor(0.0f, 0.0f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        glDrawArrays(GL_TRIANGLES, 0, 3);
        glFinish();
        glDisable(GL_BLEND);

        memset(px, 0x5a, sizeof(px));
        glReadPixels(0, 0, W, H, GL_RGBA, GL_UNSIGNED_BYTE, px);
        c = px + ((H / 2) * W + W / 2) * 4;
        snprintf(detail, sizeof(detail),
                 "centre %02x%02x%02x (want about 800080)",
                 c[0], c[1], c[2]);
        report("alpha blending",
               c[0] > 0x60 && c[0] < 0xa0 && c[2] > 0x60 && c[2] < 0xa0,
               detail);
    }

    /*
     * Stage 8: depth testing.  A near triangle then a far one covering the
     * same pixels: the far one must lose.
     */
    {
        GLuint depth_rb;
        const unsigned char *c;
        char detail[96];
        static const GLfloat far_tri[] = {
            -0.9f, -0.9f, 0.5f, 1.0f,
             0.9f, -0.9f, 0.5f, 1.0f,
             0.0f,  0.9f, 0.5f, 1.0f,
        };
        GLuint far_vbo;

        glGenRenderbuffers(1, &depth_rb);
        glBindRenderbuffer(GL_RENDERBUFFER, depth_rb);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT16, W, H);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                                  GL_RENDERBUFFER, depth_rb);
        if (glCheckFramebufferStatus(GL_FRAMEBUFFER) !=
            GL_FRAMEBUFFER_COMPLETE) {
            report("depth attachment", 0, "framebuffer incomplete");
            goto done;
        }

        static const char *green_fs_src =
            "precision mediump float;\n"
            "void main() { gl_FragColor = vec4(0.0, 1.0, 0.0, 1.0); }\n";
        GLuint gprog = glCreateProgram();
        GLint glinked = 0;

        glAttachShader(gprog, compile(GL_VERTEX_SHADER, vs_src));
        glAttachShader(gprog, compile(GL_FRAGMENT_SHADER, green_fs_src));
        glBindAttribLocation(gprog, 0, "pos");
        glLinkProgram(gprog);
        glGetProgramiv(gprog, GL_LINK_STATUS, &glinked);
        if (!glinked) {
            report("depth: green program links", 0, "link failed");
            goto done;
        }

        glUseProgram(prog);            /* opaque red, near */
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);
        glClearColor(0.0f, 0.0f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        /* Near first, at z = 0. */
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0);
        glDrawArrays(GL_TRIANGLES, 0, 3);

        /* Then the same triangle further away, in green: it must be rejected.
         * A different colour is the whole point -- with both draws red the
         * check would pass whether or not the depth test did anything. */
        glUseProgram(gprog);
        glGenBuffers(1, &far_vbo);
        glBindBuffer(GL_ARRAY_BUFFER, far_vbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof(far_tri), far_tri, GL_STATIC_DRAW);
        glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0);
        glDrawArrays(GL_TRIANGLES, 0, 3);
        glFinish();
        glDisable(GL_DEPTH_TEST);

        memset(px, 0x5a, sizeof(px));
        glReadPixels(0, 0, W, H, GL_RGBA, GL_UNSIGNED_BYTE, px);
        c = px + ((H / 2) * W + W / 2) * 4;
        snprintf(detail, sizeof(detail),
                 "centre %02x%02x%02x (near red must survive the far green)",
                 c[0], c[1], c[2]);
        report("depth test", c[0] == 0xff && c[1] == 0x00, detail);
    }

    /*
     * Stage 9: texturing.  A 2x2 texture of four distinct colours stretched
     * over the whole viewport with nearest filtering, so each quadrant must
     * come back as its own texel.  Getting the layout or the coordinates
     * wrong shuffles the quadrants rather than blanking them, which is why
     * all four are checked.
     */
    {
        static const unsigned char texels[] = {
            0xff, 0x00, 0x00, 0xff,     /* red    at (0,0) */
            0x00, 0xff, 0x00, 0xff,     /* green  at (1,0) */
            0x00, 0x00, 0xff, 0xff,     /* blue   at (0,1) */
            0xff, 0xff, 0x00, 0xff,     /* yellow at (1,1) */
        };
        static const char *tex_vs_src =
            "attribute vec4 pos;\n"
            "attribute vec2 uv;\n"
            "varying vec2 vuv;\n"
            "void main() { gl_Position = pos; vuv = uv; }\n";
        static const char *tex_fs_src =
            "precision mediump float;\n"
            "uniform sampler2D tex;\n"
            "varying vec2 vuv;\n"
            "void main() { gl_FragColor = texture2D(tex, vuv); }\n";
        /* Two triangles covering the viewport, with matching texcoords. */
        static const GLfloat quad[] = {
            -1.0f, -1.0f, 0.0f, 1.0f,   1.0f, -1.0f, 0.0f, 1.0f,
            -1.0f,  1.0f, 0.0f, 1.0f,   1.0f, -1.0f, 0.0f, 1.0f,
             1.0f,  1.0f, 0.0f, 1.0f,  -1.0f,  1.0f, 0.0f, 1.0f,
        };
        static const GLfloat uvs[] = {
            0.0f, 0.0f,  1.0f, 0.0f,  0.0f, 1.0f,
            1.0f, 0.0f,  1.0f, 1.0f,  0.0f, 1.0f,
        };
        GLuint tex, tprog, qvbo, uvbo;
        GLint tlinked = 0;
        const unsigned char *q0, *q1, *q2, *q3;
        char detail[160];
        int ok;

        glGenTextures(1, &tex);
        glBindTexture(GL_TEXTURE_2D, tex);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 2, 2, 0, GL_RGBA,
                     GL_UNSIGNED_BYTE, texels);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

        tprog = glCreateProgram();
        glAttachShader(tprog, compile(GL_VERTEX_SHADER, tex_vs_src));
        glAttachShader(tprog, compile(GL_FRAGMENT_SHADER, tex_fs_src));
        glBindAttribLocation(tprog, 0, "pos");
        glBindAttribLocation(tprog, 1, "uv");
        glLinkProgram(tprog);
        glGetProgramiv(tprog, GL_LINK_STATUS, &tlinked);
        if (!tlinked) {
            char log[512] = { 0 };

            glGetProgramInfoLog(tprog, sizeof(log) - 1, NULL, log);
            report("texture program links", 0, log);
            goto done;
        }
        glUseProgram(tprog);
        glUniform1i(glGetUniformLocation(tprog, "tex"), 0);

        glGenBuffers(1, &qvbo);
        glBindBuffer(GL_ARRAY_BUFFER, qvbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof(quad), quad, GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0);

        glGenBuffers(1, &uvbo);
        glBindBuffer(GL_ARRAY_BUFFER, uvbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof(uvs), uvs, GL_STATIC_DRAW);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, 0);

        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        glDrawArrays(GL_TRIANGLES, 0, 6);
        glFinish();

        memset(px, 0x5a, sizeof(px));
        glReadPixels(0, 0, W, H, GL_RGBA, GL_UNSIGNED_BYTE, px);

        /* One sample well inside each quadrant. */
        q0 = px + ((H / 4) * W + W / 4) * 4;
        q1 = px + ((H / 4) * W + 3 * W / 4) * 4;
        q2 = px + ((3 * H / 4) * W + W / 4) * 4;
        q3 = px + ((3 * H / 4) * W + 3 * W / 4) * 4;

        /* Each quadrant must be one of the four texels, and all four must be
         * different from each other. */
        ok = (q0[0] | q0[1] | q0[2]) && (q1[0] | q1[1] | q1[2]) &&
             (q2[0] | q2[1] | q2[2]) && (q3[0] | q3[1] | q3[2]) &&
             memcmp(q0, q1, 3) && memcmp(q0, q2, 3) && memcmp(q0, q3, 3) &&
             memcmp(q1, q2, 3) && memcmp(q1, q3, 3) && memcmp(q2, q3, 3);

        snprintf(detail, sizeof(detail),
                 "quadrants %02x%02x%02x %02x%02x%02x %02x%02x%02x %02x%02x%02x"
                 " (four distinct texels)",
                 q0[0], q0[1], q0[2], q1[0], q1[1], q1[2],
                 q2[0], q2[1], q2[2], q3[0], q3[1], q3[2]);
        report("texture sampled", ok, detail);

        /*
         * The same texture with linear filtering.  Every channel of every one
         * of the four texels is either 0x00 or 0xff, so a genuine blend at the
         * centre of the quad -- where all four contribute -- must land strictly
         * inside that range on all three channels.  Nearest filtering returns
         * one texel exactly and so fails.  Checking the interval rather than a
         * specific average keeps the test independent of exactly where the
         * sample point falls, which the pixel centre offset shifts slightly.
         */
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glClear(GL_COLOR_BUFFER_BIT);
        glDrawArrays(GL_TRIANGLES, 0, 6);
        glFinish();

        memset(px, 0x5a, sizeof(px));
        glReadPixels(0, 0, W, H, GL_RGBA, GL_UNSIGNED_BYTE, px);
        {
            const unsigned char *c = px + ((H / 2) * W + W / 2) * 4;
            int blended = c[0] > 0x08 && c[0] < 0xf7 &&
                          c[1] > 0x08 && c[1] < 0xf7 &&
                          c[2] > 0x08 && c[2] < 0xf7;

            snprintf(detail, sizeof(detail),
                     "centre %02x%02x%02x (every channel must be a blend, "
                     "not 00 or ff)", c[0], c[1], c[2]);
            report("texture filtered", blended, detail);
        }
        glDisableVertexAttribArray(1);
    }

    /*
     * Stages 10-13 sweep paths the model has code for but has never actually
     * been asked to run by a real driver: indexed drawing, the strip and fan
     * primitive modes, more vertices than fit in one 16-lane batch, and the
     * scissor rectangle.
     */
    {
        /* A quad as four vertices, drawn three different ways. */
        static const GLfloat quad4[] = {
            -0.8f, -0.8f, 0.0f, 1.0f,
             0.8f, -0.8f, 0.0f, 1.0f,
            -0.8f,  0.8f, 0.0f, 1.0f,
             0.8f,  0.8f, 0.0f, 1.0f,
        };
        static const GLushort strip_idx[] = { 0, 1, 2, 3 };
        static const GLushort fan_idx[] = { 0, 1, 3, 2 };
        GLuint qbo, ibo;
        unsigned centre = ((H / 2) * W + W / 2) * 4;
        char detail[128];

        glUseProgram(prog);                     /* opaque red */
        glDisableVertexAttribArray(1);
        glGenBuffers(1, &qbo);
        glBindBuffer(GL_ARRAY_BUFFER, qbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof(quad4), quad4, GL_STATIC_DRAW);
        glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0);

        glGenBuffers(1, &ibo);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);

        /* Indexed triangle strip. */
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(strip_idx), strip_idx,
                     GL_STATIC_DRAW);
        glClearColor(0.0f, 0.0f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        glDrawElements(GL_TRIANGLE_STRIP, 4, GL_UNSIGNED_SHORT, 0);
        glFinish();
        memset(px, 0x5a, sizeof(px));
        glReadPixels(0, 0, W, H, GL_RGBA, GL_UNSIGNED_BYTE, px);
        snprintf(detail, sizeof(detail), "centre %02x%02x%02x (want ff0000)",
                 px[centre], px[centre + 1], px[centre + 2]);
        report("indexed triangle strip",
               px[centre] == 0xff && px[centre + 2] == 0x00, detail);

        /* Indexed triangle fan over the same four vertices. */
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(fan_idx), fan_idx,
                     GL_STATIC_DRAW);
        glClear(GL_COLOR_BUFFER_BIT);
        glDrawElements(GL_TRIANGLE_FAN, 4, GL_UNSIGNED_SHORT, 0);
        glFinish();
        memset(px, 0x5a, sizeof(px));
        glReadPixels(0, 0, W, H, GL_RGBA, GL_UNSIGNED_BYTE, px);
        snprintf(detail, sizeof(detail), "centre %02x%02x%02x (want ff0000)",
                 px[centre], px[centre + 1], px[centre + 2]);
        report("indexed triangle fan",
               px[centre] == 0xff && px[centre + 2] == 0x00, detail);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    }

    /*
     * More vertices than one 16-lane batch, so the model has to shade several
     * batches and stitch the results together.
     */
    {
        enum { STRIP_N = 42 };
        GLfloat strip[STRIP_N * 4];
        GLuint sbo;
        unsigned i, red = 0;
        char detail[96];

        /* A horizontal band built from a long triangle strip. */
        for (i = 0; i < STRIP_N; i++) {
            float t = (float)(i / 2) / (float)(STRIP_N / 2 - 1);

            strip[i * 4 + 0] = -1.0f + 2.0f * t;
            strip[i * 4 + 1] = (i & 1) ? 0.6f : -0.6f;
            strip[i * 4 + 2] = 0.0f;
            strip[i * 4 + 3] = 1.0f;
        }

        glGenBuffers(1, &sbo);
        glBindBuffer(GL_ARRAY_BUFFER, sbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof(strip), strip, GL_STATIC_DRAW);
        glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0);

        glClearColor(0.0f, 0.0f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, STRIP_N);
        glFinish();

        memset(px, 0x5a, sizeof(px));
        glReadPixels(0, 0, W, H, GL_RGBA, GL_UNSIGNED_BYTE, px);
        for (i = 0; i < W * H; i++) {
            if (px[i * 4] == 0xff && px[i * 4 + 2] == 0x00) {
                red++;
            }
        }
        /* The band is 60% of the height across the full width. */
        snprintf(detail, sizeof(detail), "%u vertices, %u/%u pixels covered",
                 STRIP_N, red, W * H);
        report("multi-batch strip",
               red > (W * H) / 3 && red < (W * H) * 4 / 5, detail);
    }

    /* The scissor rectangle must keep a draw out of the rest of the surface. */
    {
        const unsigned char *inside, *outside;
        char detail[96];

        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0);

        glClearColor(0.0f, 0.0f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        glEnable(GL_SCISSOR_TEST);
        glScissor(0, 0, W / 2, H);          /* left half only */
        glDrawArrays(GL_TRIANGLES, 0, 3);
        glFinish();
        glDisable(GL_SCISSOR_TEST);

        memset(px, 0x5a, sizeof(px));
        glReadPixels(0, 0, W, H, GL_RGBA, GL_UNSIGNED_BYTE, px);
        /* Two points the triangle covers, one each side of the scissor edge. */
        inside = px + ((H / 4) * W + W / 2 - 2) * 4;
        outside = px + ((H / 4) * W + W / 2 + 2) * 4;
        snprintf(detail, sizeof(detail),
                 "left half: in %02x%02x%02x, out %02x%02x%02x"
                 " (out must stay blue)",
                 inside[0], inside[1], inside[2],
                 outside[0], outside[1], outside[2]);
        report("scissor in x",
               inside[0] == 0xff && outside[2] == 0xff && outside[0] == 0x00,
               detail);

        /*
         * And again across the other axis, so the model cannot pass with the
         * scissor's y origin upside down.  glReadPixels and glScissor share an
         * origin, so scissoring the bottom half must keep the low rows.
         */
        glClear(GL_COLOR_BUFFER_BIT);
        glEnable(GL_SCISSOR_TEST);
        glScissor(0, 0, W, H / 2);
        glDrawArrays(GL_TRIANGLES, 0, 3);
        glFinish();
        glDisable(GL_SCISSOR_TEST);

        memset(px, 0x5a, sizeof(px));
        glReadPixels(0, 0, W, H, GL_RGBA, GL_UNSIGNED_BYTE, px);
        /* The triangle is wide at the bottom, so a low row is covered. */
        inside = px + (2 * W + W / 2) * 4;
        outside = px + ((H - 3) * W + W / 2) * 4;
        snprintf(detail, sizeof(detail),
                 "bottom half: in %02x%02x%02x, out %02x%02x%02x"
                 " (out must stay blue)",
                 inside[0], inside[1], inside[2],
                 outside[0], outside[1], outside[2]);
        report("scissor in y",
               inside[0] == 0xff && outside[2] == 0xff && outside[0] == 0x00,
               detail);
    }

done:
    printf("GLES: %d stage(s) failed\n", failures);
    return failures ? 1 : 0;
}
