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

done:
    printf("GLES: %d stage(s) failed\n", failures);
    return failures ? 1 : 0;
}
