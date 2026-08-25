/* Clean-room boot ROM stubs for the n3ds machine; sources in n3ds-bootstub/. */
#ifndef HW_ARM_N3DS_BOOTSTUB_H
#define HW_ARM_N3DS_BOOTSTUB_H
#include <stdint.h>
extern const uint32_t n3ds_boot11_stub[0x1B00 / 4];
extern const uint32_t n3ds_boot9_stub[0x0C80 / 4];
#endif
