/*
 * SDIO function model for the Nintendo 3DS WiFi module.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_SD_N3DS_SDIO_H
#define HW_SD_N3DS_SDIO_H

#include "hw/core/qdev.h"
#include "qom/object.h"

#define TYPE_N3DS_SDIO "n3ds-sdio"
OBJECT_DECLARE_SIMPLE_TYPE(N3DSSdioState, N3DS_SDIO)

enum {
    SDIO_ST_IDLE,
    SDIO_ST_READY,
    SDIO_ST_STANDBY,
    SDIO_ST_SELECTED,
};

struct N3DSSdioState {
    DeviceState parent_obj;

    uint16_t vendor_id;
    uint16_t device_id;

    int state;
    uint16_t rca;
    uint32_t ocr;
    unsigned ocr_polls;
    uint8_t io_enable;
    uint8_t int_enable;
    uint8_t bus_iface;
    uint16_t fn0_blksz;
    uint16_t fn1_blksz;

    /* CMD53 transfer in progress */
    bool xfer_write;
    bool xfer_incr;
    unsigned xfer_fn;
    uint32_t xfer_addr;
    uint32_t xfer_left;

    /*
     * Backing store for function 1. The real part exposes the AR6K
     * mailboxes and register window here; this is plain memory, enough for
     * a driver's accesses to succeed rather than error out.
     */
    uint8_t fn1_mem[0x20000];
};

int n3ds_sdio_command(N3DSSdioState *s, uint32_t cmd, uint32_t arg,
                      uint8_t *resp, size_t respsz);
void n3ds_sdio_read_data(N3DSSdioState *s, uint8_t *buf, size_t len);
void n3ds_sdio_write_data(N3DSSdioState *s, const uint8_t *buf, size_t len);

#endif
