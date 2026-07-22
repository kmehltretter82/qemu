/*
 * Connect the guest display and input devices to the browser page.
 *
 * The emscripten build has no display backend: QEMU runs with
 * -display none, so nothing ever drives qemu_console_hw_update() and the
 * VIDC20 model draws into a surface nobody reads.
 *
 * Rather than port a real UI backend, drive the refresh from a timer
 * here and hand the finished surface to the page.  The direction of the
 * call matters: QEMU's main() runs on a worker under -sPROXY_TO_PTHREAD
 * while the canvas lives on the browser main thread, and emscripten can
 * proxy a *called* JS function but cannot let the main thread reach in
 * and run QEMU code safely.  So C calls out, exactly as build/
 * stdin-proxy.js does for the keyboard, and the JS side is marked
 * __proxy: 'sync'.  Memory is shared (pthreads implies a
 * SharedArrayBuffer), so the pointer stays valid on the main thread. Input
 * takes the reverse trip through a small queue which is likewise consumed by
 * a proxied call, then enters QEMU through its ordinary input API.
 *
 * Copyright (c) 2026 Karl Mehltretter
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/timer.h"
#include "ui/console.h"
#include "ui/input.h"

/* Implemented in build/display-canvas.js */
extern void rpc_fb_blit(void *data, int width, int height, int stride);
extern uint32_t rpc_input_pop(void);

#define RPC_FB_FRAME_NS (NANOSECONDS_PER_SECOND / 30)
#define RPC_INPUT_TYPE_SHIFT 24
#define RPC_INPUT_VALUE_MASK 0xffff
#define RPC_INPUT_DRAIN_LIMIT 256

enum RpcInputType {
    RPC_INPUT_KEY_DOWN = 1,
    RPC_INPUT_KEY_UP,
    RPC_INPUT_REL_X,
    RPC_INPUT_REL_Y,
    RPC_INPUT_BUTTON_DOWN,
    RPC_INPUT_BUTTON_UP,
};

static QEMUTimer *rpc_fb_timer;

static void rpc_input_drain(QemuConsole *con)
{
    static const InputButton buttons[] = {
        INPUT_BUTTON_LEFT,
        INPUT_BUTTON_MIDDLE,
        INPUT_BUTTON_RIGHT,
    };
    bool pointer_events = false;
    unsigned int i;

    for (i = 0; i < RPC_INPUT_DRAIN_LIMIT; i++) {
        uint32_t event = rpc_input_pop();
        unsigned int type;
        int value;

        if (!event) {
            break;
        }
        type = event >> RPC_INPUT_TYPE_SHIFT;
        value = (int16_t)(event & RPC_INPUT_VALUE_MASK);

        switch (type) {
        case RPC_INPUT_KEY_DOWN:
        case RPC_INPUT_KEY_UP:
            if (value > 0) {
                qemu_input_event_send_key_linux(con, value,
                                                type == RPC_INPUT_KEY_DOWN);
            }
            break;
        case RPC_INPUT_REL_X:
        case RPC_INPUT_REL_Y:
            if (value) {
                qemu_input_queue_rel(con,
                                     type == RPC_INPUT_REL_X ?
                                     INPUT_AXIS_X : INPUT_AXIS_Y,
                                     value);
                pointer_events = true;
            }
            break;
        case RPC_INPUT_BUTTON_DOWN:
        case RPC_INPUT_BUTTON_UP:
            if (value >= 0 && value < (int)ARRAY_SIZE(buttons)) {
                qemu_input_queue_btn(con, buttons[value],
                                     type == RPC_INPUT_BUTTON_DOWN);
                pointer_events = true;
            }
            break;
        default:
            break;
        }
    }

    if (pointer_events) {
        qemu_input_event_sync();
    }
}

static void rpc_fb_tick(void *opaque)
{
    QemuConsole *con = qemu_console_lookup_by_index(0);
    DisplaySurface *surface;

    if (con) {
        rpc_input_drain(con);

        /*
         * With no DisplayChangeListener attached nothing else calls
         * this, so the device's gfx_update never runs.
         */
        qemu_console_hw_update(con);

        surface = qemu_console_surface(con);
        if (surface) {
            rpc_fb_blit(surface_data(surface), surface_width(surface),
                        surface_height(surface), surface_stride(surface));
        }
    }

    timer_mod(rpc_fb_timer,
              qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + RPC_FB_FRAME_NS);
}

void rpc_fb_start(void);
void rpc_fb_start(void)
{
    if (rpc_fb_timer) {
        return;
    }
    rpc_fb_timer = timer_new_ns(QEMU_CLOCK_REALTIME, rpc_fb_tick, NULL);
    timer_mod(rpc_fb_timer,
              qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + RPC_FB_FRAME_NS);
}
