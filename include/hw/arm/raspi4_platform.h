/*
 * Raspberry Pi 4 / BCM2711 platform definitions
 *
 * These definitions are derived from those in Raspbian Linux at
 * arch/arm/mach-{bcm2708,bcm2709}/include/mach/platform.h
 * where they carry the following notice:
 *
 * Copyright (C) 2010 Broadcom
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Various undocumented addresses and names come from Herman Hermitage's VC4
 * documentation:
 * https://github.com/hermanhermitage/videocoreiv/wiki/MMIO-Register-map
 */

#ifndef HW_ARM_RASPI4_PLATFORM_H
#define HW_ARM_RASPI4_PLATFORM_H

#include "hw/core/boards.h"
#include "hw/arm/boot.h"
#include "hw/arm/bcm2838.h"

#define TYPE_RASPI4_BASE_MACHINE MACHINE_TYPE_NAME("raspi4-base")
OBJECT_DECLARE_TYPE(Raspi4BaseMachineState, Raspi4BaseMachineClass,
                    RASPI4_BASE_MACHINE)

struct Raspi4BaseMachineState {
    /*< private >*/
    MachineState parent_obj;
    /*< public >*/
    struct arm_boot_info binfo;
    BCM2838State soc;
    bool exact_poison;
    uint8_t poison_byte;
    uint64_t poison_seed;
};

struct Raspi4BaseMachineClass {
    /*< private >*/
    MachineClass parent_obj;
    /*< public >*/
    uint32_t board_rev;
};

void raspi4_common_machine_init(MachineState *machine, BCM2838State *soc);

void raspi4_common_machine_class_init(MachineClass *mc,
                                      uint32_t board_rev,
                                      const char *model);
uint64_t raspi4_board_ram_size(uint32_t board_rev);

#define ST_OFFSET               0x3000   /* System Timer */
#define TXP_OFFSET              0x4000   /* Transposer */
#define DMA_OFFSET              0x7000   /* DMA controller, channels 0-14 */
#define BRDG_OFFSET             0xa000   /* RPiVid ASB for BCM2838 (BCM2711) */
#define ARM_OFFSET              0xB000   /* ARM control block */
#define ARMCTRL_OFFSET          (ARM_OFFSET + 0x000)
#define ARMCTRL_IC_OFFSET       (ARM_OFFSET + 0x200) /* Interrupt controller */
#define ARMCTRL_TIMER0_1_OFFSET (ARM_OFFSET + 0x400) /* Timer 0 and 1 (SP804) */
/* User 0 (ARM) semaphores, doorbells, and mailboxes */
#define ARMCTRL_0_SBM_OFFSET    (ARM_OFFSET + 0x800)
#define PM_OFFSET               0x100000 /* Power Management */
#define CPRMAN_OFFSET           0x101000 /* Clock Management */
#define RNG200_OFFSET           0x104000
#define GPIO_OFFSET             0x200000
#define PIXELVALVE2_OFFSET      0x20a000
#define UART0_OFFSET            0x201000 /* PL011 */
#define MMCI0_OFFSET            0x202000 /* Legacy MMC */
#define I2S_OFFSET              0x203000 /* PCM */
#define SPI0_OFFSET             0x204000 /* SPI master */
#define BSC0_OFFSET             0x205000 /* BSC0 I2C/TWI */
#define PWM0_OFFSET             0x20c000
#define PWM1_OFFSET             0x20c800
#define OTP_OFFSET              0x20f000
#define BSC_SL_OFFSET           0x214000 /* SPI slave (bootrom) */
#define AUX_OFFSET              0x215000 /* AUX: UART1/SPI1/SPI2 */
#define EMMC1_OFFSET            0x300000
#define EMMC2_OFFSET            0x340000
#define PCIE_RC_OFFSET          0x1500000
#define GENET_OFFSET            0x1580000
#define GENET_SIZE              0x10000
#define AVS_OFFSET              0x15d2000
#define SMI_OFFSET              0x600000
#define HVS_OFFSET              0x400000
#define BSC1_OFFSET             0x804000 /* BSC1 I2C/TWI */
#define BSC2_OFFSET             0x805000 /* BSC2 I2C/TWI */
#define DBUS_OFFSET             0x900000
#define AVE0_OFFSET             0x910000
#define USB_OTG_OFFSET          0x980000 /* DTC_OTG USB controller */
#define V3D_OFFSET              0xc00000
#define SDRAMC_OFFSET           0xe00000
#define DMA15_OFFSET            0xE05000 /* DMA controller, channel 15 */
#define DVP_OFFSET              0xF00000 /* HDMI DVP clock/reset controller */
#define AON_INTR_OFFSET         0xF00100 /* Always-on HDMI L2 interrupts */
#define HDMI0_CSC_OFFSET        0xF00200
#define HDMI0_DVP_OFFSET        0xF00300
#define HDMI0_CORE_OFFSET       0xF00700
#define HDMI0_AUTO_I2C_OFFSET   0xF00B00
#define HDMI0_PHY_OFFSET        0xF00F00
#define HDMI0_RM_OFFSET         0xF00F80
#define HDMI0_PACKET_OFFSET     0xF01B00
#define HDMI0_METADATA_OFFSET   0xF01F00
#define HDMI0_CEC_OFFSET        0xF04300
#define HDMI0_DDC_OFFSET        0xF04500
#define HDMI1_AUTO_I2C_OFFSET   0xF05B00
#define HDMI1_DDC_OFFSET        0xF09500
#define HDMI_SHARED_HD_OFFSET   0xF20000

/* GPU interrupts */
#define INTERRUPT_TIMER0               0
#define INTERRUPT_TIMER1               1
#define INTERRUPT_TIMER2               2
#define INTERRUPT_TIMER3               3
#define INTERRUPT_USB                  9
#define INTERRUPT_AUX                  29
#define INTERRUPT_HOSTPORT             32
#define INTERRUPT_I2C                  53
#define INTERRUPT_SPI                  54
#define INTERRUPT_PCM                  55
#define INTERRUPT_SDIO                 56
#define INTERRUPT_UART0                57
#define INTERRUPT_ARASANSDIO           62

/* ARM CPU IRQs use a private number space */
#define INTERRUPT_ARM_MAILBOX          1

#endif /* HW_ARM_RASPI4_PLATFORM_H */
