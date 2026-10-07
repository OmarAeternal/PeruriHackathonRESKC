/*
 * NeuroFlex-CORDIC HPS driver (Linux on the DE10-Nano ARM Cortex-A9).
 * Maps the accelerator behind the Lightweight HPS-to-FPGA bridge via /dev/mem,
 * loads the INT8 model and an IMU window, starts inference and prints the
 * class, score and cycle count.
 *
 * Usage: hps_driver <model_load.txt> <window.csv> [mode 0|1|2] [slot 0|1]
 *   mode: 0 = CORDIC-8, 1 = CORDIC-12, 2 = CORDIC-16
 */
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <unistd.h>

#define LW_BRIDGE_BASE  0xFF200000u
#ifndef NF_OFFSET
#define NF_OFFSET       0x00040000u   /* base address of s0 in Platform Designer */
#endif
#define NF_SPAN         0x2000u

#define NF_CTRL    0x00
#define NF_STATUS  0x04
#define NF_RESULT  0x08
#define NF_CYCLES  0x0C
#define NF_INFER   0x10
#define NF_ID      0x14
#define NF_CONFIG  0x18
#define NF_WINBUF  0x1000

#define NF_ID_VALUE 0x4E460100u
#define N_IN        6
#define SEQ_LEN     64
#define WIN_BYTES   (SEQ_LEN * N_IN)
#define CLK_MHZ     50.0

static volatile uint32_t *nf;

static void nf_wr(uint32_t off, uint32_t v) { nf[off / 4] = v; }
static uint32_t nf_rd(uint32_t off) { return nf[off / 4]; }

static int load_model(const char *path)
{
    FILE *f = fopen(path, "r");
    unsigned addr, data;
    int n = 0;

    if (!f) {
        perror(path);
        return -1;
    }
    while (fscanf(f, "%x %x", &addr, &data) == 2) {
        if (addr < 0x0800 || addr >= NF_SPAN || (addr & 3)) {
            fprintf(stderr, "alamat tidak valid: 0x%04X\n", addr);
            fclose(f);
            return -1;
        }
        nf_wr(addr, data);
        n++;
    }
    fclose(f);
    printf("Model: %d word dimuat dari %s\n", n, path);
    return 0;
}

static int load_window(const char *path, int8_t win[SEQ_LEN][N_IN])
{
    FILE *f = fopen(path, "r");
    int v;

    if (!f) {
        perror(path);
        return -1;
    }
    for (int t = 0; t < SEQ_LEN; t++) {
        for (int i = 0; i < N_IN; i++) {
            if (fscanf(f, " %d", &v) != 1 || v < -128 || v > 127) {
                fprintf(stderr, "%s: data window tidak valid (t=%d, i=%d)\n", path, t, i);
                fclose(f);
                return -1;
            }
            win[t][i] = (int8_t)v;
            fscanf(f, " ,");
        }
    }
    fclose(f);
    return 0;
}

/* The slave has no byteenable: always write packed 32-bit words. */
static void write_window(const int8_t win[SEQ_LEN][N_IN], int slot)
{
    const uint8_t *b = (const uint8_t *)win;
    uint32_t base = NF_WINBUF + (uint32_t)slot * WIN_BYTES;

    for (int i = 0; i < WIN_BYTES; i += 4) {
        uint32_t w = (uint32_t)b[i] | ((uint32_t)b[i + 1] << 8) |
                     ((uint32_t)b[i + 2] << 16) | ((uint32_t)b[i + 3] << 24);
        nf_wr(base + (uint32_t)i, w);
    }
}

static void neuroflex_infer(int slot, int mode)
{
    nf_wr(NF_CTRL, ((uint32_t)slot << 3) | ((uint32_t)mode << 1) | 1u);
    while (!(nf_rd(NF_STATUS) & 0x2))
        ;
    uint32_t res    = nf_rd(NF_RESULT);
    uint32_t cycles = nf_rd(NF_CYCLES);
    printf("class=%s score=%.3f cycles=%u (%.2f us @%.0fMHz) total_infer=%u\n",
           (res & 0xFF) ? "ANOMALI" : "NORMAL", (res >> 16) / 1024.0,
           cycles, cycles / CLK_MHZ, CLK_MHZ, nf_rd(NF_INFER));
}

int main(int argc, char **argv)
{
    int8_t win[SEQ_LEN][N_IN];
    int mode = argc > 3 ? atoi(argv[3]) : 2;
    int slot = argc > 4 ? atoi(argv[4]) : 0;

    if (argc < 3 || mode < 0 || mode > 2 || slot < 0 || slot > 1) {
        fprintf(stderr, "Usage: %s <model_load.txt> <window.csv> [mode 0|1|2] [slot 0|1]\n", argv[0]);
        return 1;
    }

    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        perror("/dev/mem (jalankan sebagai root)");
        return 1;
    }
    void *map = mmap(NULL, NF_SPAN, PROT_READ | PROT_WRITE, MAP_SHARED, fd,
                     LW_BRIDGE_BASE + NF_OFFSET);
    if (map == MAP_FAILED) {
        perror("mmap");
        close(fd);
        return 1;
    }
    nf = (volatile uint32_t *)map;

    uint32_t id = nf_rd(NF_ID), cfg = nf_rd(NF_CONFIG);
    if (id != NF_ID_VALUE) {
        fprintf(stderr, "NeuroFlex tidak ditemukan (ID=0x%08X), cek NF_OFFSET dan bitstream\n", id);
        munmap(map, NF_SPAN);
        close(fd);
        return 1;
    }
    printf("NeuroFlex ID=0x%08X  N_IN=%u N_HID=%u N_CLS=%u SEQ_LEN=%u\n", id,
           cfg & 0xFF, (cfg >> 8) & 0xFF, (cfg >> 16) & 0xFF, cfg >> 24);

    int rc = 1;
    if (load_model(argv[1]) == 0 && load_window(argv[2], win) == 0) {
        write_window(win, slot);
        neuroflex_infer(slot, mode);
        rc = 0;
    }

    munmap(map, NF_SPAN);
    close(fd);
    return rc;
}
