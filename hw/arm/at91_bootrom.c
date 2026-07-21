/*
 * Atmel AT91 ROM-code boot helpers.
 *
 * The SAM9G45 internal ROM looks for BOOT.BIN in the root directory of a
 * FAT12/16/32 SD card, validates its ARM exception vectors, and copies it to
 * internal SRAM.  The board model performs that high-level ROM operation
 * here rather than depending on a dump of Atmel's proprietary ROM.
 *
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "hw/arm/at91_bootrom.h"
#include "hw/core/loader.h"
#include "system/block-backend.h"

#define AT91_SD_SECTOR_SIZE       512
#define AT91_BOOT_VECTOR_SIZE     28
#define AT91_BOOT_MAX_IMAGE_SIZE  (60 * KiB)

typedef enum At91FatType {
    AT91_FAT12,
    AT91_FAT16,
    AT91_FAT32,
} At91FatType;

typedef struct At91Fat {
    BlockBackend *blk;
    int64_t card_size;
    uint64_t volume_offset;
    uint32_t bytes_per_sector;
    uint32_t sectors_per_cluster;
    uint32_t reserved_sectors;
    uint32_t fat_count;
    uint32_t fat_sectors;
    uint32_t root_entries;
    uint32_t root_cluster;
    uint32_t total_sectors;
    uint32_t first_fat_sector;
    uint32_t first_root_sector;
    uint32_t first_data_sector;
    uint32_t cluster_count;
    At91FatType type;
} At91Fat;

typedef struct At91FatDirEntry {
    uint32_t first_cluster;
    uint32_t size;
} At91FatDirEntry;

static bool at91_fat_read(const At91Fat *fat, uint64_t offset, size_t bytes,
                          void *buf, Error **errp)
{
    if (offset > INT64_MAX || bytes > (uint64_t)fat->card_size ||
        offset > (uint64_t)fat->card_size - bytes) {
        error_setg(errp, "AT91 boot ROM: FAT read exceeds the SD card");
        return false;
    }

    if (blk_pread(fat->blk, offset, bytes, buf, 0) < 0) {
        error_setg(errp, "AT91 boot ROM: cannot read the SD card");
        return false;
    }
    return true;
}

static bool at91_fat_read_volume(const At91Fat *fat, uint64_t offset,
                                 size_t bytes, void *buf, Error **errp)
{
    if (offset > UINT64_MAX - fat->volume_offset) {
        error_setg(errp, "AT91 boot ROM: invalid FAT offset");
        return false;
    }
    return at91_fat_read(fat, fat->volume_offset + offset, bytes, buf, errp);
}

static bool at91_fat_parse_bpb(At91Fat *fat, uint64_t volume_offset,
                               uint64_t max_volume_size, const uint8_t *bpb)
{
    uint32_t root_dir_sectors, data_sectors;
    uint64_t first_root_sector, first_data_sector, volume_size;

    fat->bytes_per_sector = lduw_le_p(bpb + 11);
    fat->sectors_per_cluster = bpb[13];
    fat->reserved_sectors = lduw_le_p(bpb + 14);
    fat->fat_count = bpb[16];
    fat->root_entries = lduw_le_p(bpb + 17);
    fat->total_sectors = lduw_le_p(bpb + 19);
    fat->fat_sectors = lduw_le_p(bpb + 22);
    if (!fat->total_sectors) {
        fat->total_sectors = ldl_le_p(bpb + 32);
    }
    if (!fat->fat_sectors) {
        fat->fat_sectors = ldl_le_p(bpb + 36);
    }

    if (bpb[510] != 0x55 || bpb[511] != 0xaa ||
        fat->bytes_per_sector < AT91_SD_SECTOR_SIZE ||
        fat->bytes_per_sector > 4096 ||
        !is_power_of_2(fat->bytes_per_sector) ||
        !fat->sectors_per_cluster ||
        !is_power_of_2(fat->sectors_per_cluster) ||
        fat->sectors_per_cluster > 128 || !fat->reserved_sectors ||
        !fat->fat_count || !fat->fat_sectors || !fat->total_sectors) {
        return false;
    }

    root_dir_sectors = DIV_ROUND_UP(fat->root_entries * 32,
                                    fat->bytes_per_sector);
    first_root_sector = fat->reserved_sectors +
                        (uint64_t)fat->fat_count * fat->fat_sectors;
    first_data_sector = first_root_sector + root_dir_sectors;
    if (first_root_sector > UINT32_MAX || first_data_sector > UINT32_MAX ||
        first_data_sector >= fat->total_sectors) {
        return false;
    }
    fat->first_fat_sector = fat->reserved_sectors;
    fat->first_root_sector = first_root_sector;
    fat->first_data_sector = first_data_sector;

    data_sectors = fat->total_sectors - fat->first_data_sector;
    fat->cluster_count = data_sectors / fat->sectors_per_cluster;
    if (fat->cluster_count < 1) {
        return false;
    } else if (fat->cluster_count < 4085) {
        fat->type = AT91_FAT12;
    } else if (fat->cluster_count < 65525) {
        fat->type = AT91_FAT16;
    } else {
        fat->type = AT91_FAT32;
    }
    if (fat->type == AT91_FAT32 && fat->cluster_count > 0x0ffffff5) {
        return false;
    }

    if (fat->type == AT91_FAT32) {
        if (fat->root_entries) {
            return false;
        }
        fat->root_cluster = ldl_le_p(bpb + 44) & 0x0fffffff;
        if (fat->root_cluster < 2) {
            return false;
        }
    } else if (!fat->root_entries) {
        return false;
    }

    volume_size = (uint64_t)fat->total_sectors * fat->bytes_per_sector;
    if (volume_offset > fat->card_size ||
        volume_size > (uint64_t)fat->card_size - volume_offset ||
        volume_size > max_volume_size) {
        return false;
    }
    fat->volume_offset = volume_offset;
    return true;
}

static bool at91_fat_open(At91Fat *fat, BlockBackend *blk, Error **errp)
{
    uint8_t sector[AT91_SD_SECTOR_SIZE];
    int i;

    memset(fat, 0, sizeof(*fat));
    fat->blk = blk;
    fat->card_size = blk_getlength(blk);
    if (fat->card_size < AT91_SD_SECTOR_SIZE) {
        error_setg(errp, "AT91 boot ROM: SD card is too small");
        return false;
    }
    if (!at91_fat_read(fat, 0, sizeof(sector), sector, errp)) {
        return false;
    }

    /* Accept an unpartitioned (superfloppy) FAT volume. */
    if (at91_fat_parse_bpb(fat, 0, fat->card_size, sector)) {
        return true;
    }

    /* Otherwise locate a FAT volume in the DOS partition table. */
    if (sector[510] != 0x55 || sector[511] != 0xaa) {
        error_setg(errp, "AT91 boot ROM: SD card has no FAT filesystem");
        return false;
    }

    for (i = 0; i < 4; i++) {
        const uint8_t *part = sector + 0x1be + i * 16;
        uint32_t start_lba = ldl_le_p(part + 8);
        uint32_t sectors = ldl_le_p(part + 12);
        uint64_t offset = (uint64_t)start_lba * AT91_SD_SECTOR_SIZE;
        uint8_t bpb[AT91_SD_SECTOR_SIZE];

        if (!part[4] || !start_lba || !sectors ||
            offset > fat->card_size - AT91_SD_SECTOR_SIZE) {
            continue;
        }
        if (blk_pread(blk, offset, sizeof(bpb), bpb, 0) < 0) {
            continue;
        }
        if (at91_fat_parse_bpb(fat, offset,
                               (uint64_t)sectors * AT91_SD_SECTOR_SIZE,
                               bpb)) {
            return true;
        }
    }

    error_setg(errp, "AT91 boot ROM: SD card has no FAT12/16/32 volume");
    return false;
}

static bool at91_fat_cluster_valid(const At91Fat *fat, uint32_t cluster)
{
    return cluster >= 2 && cluster < fat->cluster_count + 2;
}

static bool at91_fat_cluster_offset(const At91Fat *fat, uint32_t cluster,
                                    uint64_t *offset, Error **errp)
{
    uint64_t sector;

    if (!at91_fat_cluster_valid(fat, cluster)) {
        error_setg(errp, "AT91 boot ROM: invalid FAT cluster %u", cluster);
        return false;
    }
    sector = fat->first_data_sector +
             (uint64_t)(cluster - 2) * fat->sectors_per_cluster;
    *offset = sector * fat->bytes_per_sector;
    return true;
}

static bool at91_fat_next_cluster(const At91Fat *fat, uint32_t cluster,
                                  uint32_t *next, Error **errp)
{
    uint64_t fat_offset = (uint64_t)fat->first_fat_sector *
                          fat->bytes_per_sector;
    uint64_t fat_size = (uint64_t)fat->fat_sectors * fat->bytes_per_sector;
    uint64_t entry_offset;
    uint8_t entry[4];
    size_t entry_size;

    switch (fat->type) {
    case AT91_FAT12:
        entry_offset = cluster + cluster / 2;
        entry_size = 2;
        break;
    case AT91_FAT16:
        entry_offset = (uint64_t)cluster * 2;
        entry_size = 2;
        break;
    case AT91_FAT32:
        entry_offset = (uint64_t)cluster * 4;
        entry_size = 4;
        break;
    default:
        g_assert_not_reached();
    }

    if (entry_offset > fat_size || entry_size > fat_size - entry_offset) {
        error_setg(errp, "AT91 boot ROM: FAT cluster entry is out of range");
        return false;
    }
    fat_offset += entry_offset;

    if (!at91_fat_read_volume(fat, fat_offset, entry_size, entry, errp)) {
        return false;
    }
    if (fat->type == AT91_FAT12) {
        *next = lduw_le_p(entry);
        *next = cluster & 1 ? *next >> 4 : *next & 0x0fff;
    } else if (fat->type == AT91_FAT16) {
        *next = lduw_le_p(entry);
    } else {
        *next = ldl_le_p(entry) & 0x0fffffff;
    }
    return true;
}

static bool at91_fat_is_eoc(const At91Fat *fat, uint32_t cluster)
{
    switch (fat->type) {
    case AT91_FAT12:
        return cluster >= 0x0ff8;
    case AT91_FAT16:
        return cluster >= 0xfff8;
    case AT91_FAT32:
        return cluster >= 0x0ffffff8;
    default:
        g_assert_not_reached();
    }
}

/* Return 1 for BOOT.BIN, 0 to continue, and -1 at the end marker. */
static int at91_fat_scan_dir(const At91Fat *fat, const uint8_t *buf,
                             size_t bytes,
                             At91FatDirEntry *file)
{
    static const uint8_t boot_name[11] = "BOOT    BIN";
    size_t offset;

    for (offset = 0; offset + 32 <= bytes; offset += 32) {
        const uint8_t *entry = buf + offset;
        uint8_t attr = entry[11];
        size_t i;

        if (entry[0] == 0x00) {
            return -1;
        }
        if (entry[0] == 0xe5 || attr == 0x0f || (attr & 0x18)) {
            continue;
        }
        for (i = 0; i < sizeof(boot_name); i++) {
            if (g_ascii_toupper(entry[i]) != boot_name[i]) {
                break;
            }
        }
        if (i == sizeof(boot_name)) {
            file->first_cluster = lduw_le_p(entry + 26);
            if (fat->type == AT91_FAT32) {
                file->first_cluster |= (uint32_t)lduw_le_p(entry + 20) << 16;
            }
            file->first_cluster &= 0x0fffffff;
            file->size = ldl_le_p(entry + 28);
            return 1;
        }
    }
    return 0;
}

static bool at91_fat_find_boot_bin(const At91Fat *fat,
                                   At91FatDirEntry *file, Error **errp)
{
    g_autofree uint8_t *sector = g_malloc(fat->bytes_per_sector);
    uint32_t cluster, visits = 0;

    if (fat->type != AT91_FAT32) {
        uint32_t root_sectors = DIV_ROUND_UP(fat->root_entries * 32,
                                             fat->bytes_per_sector);
        uint32_t i;

        for (i = 0; i < root_sectors; i++) {
            uint64_t offset = (uint64_t)(fat->first_root_sector + i) *
                              fat->bytes_per_sector;
            int result;

            if (!at91_fat_read_volume(fat, offset, fat->bytes_per_sector,
                                      sector, errp)) {
                return false;
            }
            result = at91_fat_scan_dir(fat, sector, fat->bytes_per_sector,
                                       file);
            if (result > 0) {
                return true;
            }
            if (result < 0) {
                break;
            }
        }
        error_setg(errp, "AT91 boot ROM: BOOT.BIN not found in FAT root");
        return false;
    }

    cluster = fat->root_cluster;
    while (at91_fat_cluster_valid(fat, cluster) &&
           visits++ < fat->cluster_count) {
        uint64_t cluster_offset;
        uint32_t i, next;

        if (!at91_fat_cluster_offset(fat, cluster, &cluster_offset, errp)) {
            return false;
        }
        for (i = 0; i < fat->sectors_per_cluster; i++) {
            int result;

            if (!at91_fat_read_volume(fat,
                        cluster_offset + (uint64_t)i * fat->bytes_per_sector,
                        fat->bytes_per_sector, sector, errp)) {
                return false;
            }
            result = at91_fat_scan_dir(fat, sector, fat->bytes_per_sector,
                                       file);
            if (result > 0) {
                return true;
            }
            if (result < 0) {
                error_setg(errp,
                           "AT91 boot ROM: BOOT.BIN not found in FAT root");
                return false;
            }
        }
        if (!at91_fat_next_cluster(fat, cluster, &next, errp)) {
            return false;
        }
        if (at91_fat_is_eoc(fat, next)) {
            break;
        }
        cluster = next;
    }

    error_setg(errp, "AT91 boot ROM: BOOT.BIN not found in FAT root");
    return false;
}

static bool at91_fat_read_file(const At91Fat *fat,
                               const At91FatDirEntry *file, size_t bytes,
                               uint8_t *buf, Error **errp)
{
    uint32_t cluster = file->first_cluster, visits = 0;
    size_t done = 0;
    uint64_t cluster_size = (uint64_t)fat->sectors_per_cluster *
                            fat->bytes_per_sector;

    while (done < bytes && at91_fat_cluster_valid(fat, cluster) &&
           visits++ < fat->cluster_count) {
        uint64_t offset;
        size_t chunk = MIN(cluster_size, bytes - done);
        uint32_t next;

        if (!at91_fat_cluster_offset(fat, cluster, &offset, errp) ||
            !at91_fat_read_volume(fat, offset, chunk, buf + done, errp)) {
            return false;
        }
        done += chunk;
        if (done == bytes) {
            return true;
        }
        if (!at91_fat_next_cluster(fat, cluster, &next, errp)) {
            return false;
        }
        if (at91_fat_is_eoc(fat, next)) {
            break;
        }
        cluster = next;
    }

    error_setg(errp, "AT91 boot ROM: truncated BOOT.BIN cluster chain");
    return false;
}

static bool at91_boot_vector_valid(uint32_t insn)
{
    /* Unconditional ARM B, or LDR pc using pc-relative addressing. */
    return (insn & 0xff000000) == 0xea000000 ||
           (insn & 0xff1ff000) == 0xe51ff000;
}

static bool at91_boot_image_valid(const uint8_t *image, size_t file_size,
                                  size_t sram_size, uint32_t *code_size,
                                  Error **errp)
{
    static const unsigned int vector_offsets[] = { 0, 4, 8, 12, 16, 24 };
    int i;

    if (file_size < AT91_BOOT_VECTOR_SIZE) {
        error_setg(errp, "AT91 boot ROM: BOOT.BIN is too small");
        return false;
    }
    for (i = 0; i < ARRAY_SIZE(vector_offsets); i++) {
        if (!at91_boot_vector_valid(ldl_le_p(image + vector_offsets[i]))) {
            error_setg(errp,
                       "AT91 boot ROM: BOOT.BIN has an invalid ARM vector");
            return false;
        }
    }

    *code_size = ldl_le_p(image + 0x14);
    if (*code_size < AT91_BOOT_VECTOR_SIZE ||
        *code_size >= AT91_BOOT_MAX_IMAGE_SIZE || *code_size > sram_size ||
        *code_size > file_size) {
        error_setg(errp, "AT91 boot ROM: invalid BOOT.BIN code size %u",
                   *code_size);
        return false;
    }
    return true;
}

bool at91_bootrom_load_sd(BlockBackend *blk, hwaddr sram_addr,
                          size_t sram_size, Error **errp)
{
    At91Fat fat;
    At91FatDirEntry file;
    g_autofree uint8_t *image = NULL;
    size_t read_size;
    uint32_t code_size;

    if (!at91_fat_open(&fat, blk, errp) ||
        !at91_fat_find_boot_bin(&fat, &file, errp)) {
        return false;
    }

    read_size = MIN((size_t)file.size,
                    MIN(sram_size, (size_t)AT91_BOOT_MAX_IMAGE_SIZE - 1));
    if (read_size < AT91_BOOT_VECTOR_SIZE) {
        error_setg(errp, "AT91 boot ROM: BOOT.BIN is too small");
        return false;
    }
    image = g_malloc(read_size);
    if (!at91_fat_read_file(&fat, &file, read_size, image, errp) ||
        !at91_boot_image_valid(image, file.size, sram_size, &code_size, errp)) {
        return false;
    }

    rom_add_blob_fixed("at91.boot.bin", image, code_size, sram_addr);
    return true;
}
