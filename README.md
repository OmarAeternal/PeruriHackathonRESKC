# NeuroFlex-CORDIC — Implementasi VHDL untuk DE10-Nano

Akselerator Edge AI berbasis **TinyGRU** dan **Shared CORDIC** untuk pemantauan kondisi
real-time dari data IMU 6-sumbu, diimplementasikan dalam VHDL untuk board
**Terasic DE10-Nano (Cyclone V SoC 5CSEBA6U23I7)**.

Tim **Tba Sistem Cerdas Dan Res** — Universitas Brawijaya
(Syarief Choirul Anwar, Keirio Omar Attallah, Muhamad Al Fatih, Grace Wahyuni;
pembimbing: Wijaya Kurniawan, S.T., M.T., Ph.D.)

---

## 1. Ringkasan

| Item | Nilai |
|---|---|
| Model | GRU 1 layer, input 6, hidden 16, dense 16 → 2 (NORMAL / ANOMALI), 1.186 parameter |
| Window | 64 timestep × 6 sumbu (akselerometer + giroskop), INT8 |
| Operasi linear | INT8 MAC Engine, 4 lane, akumulator 24-bit |
| Operasi nonlinear | 1 Shared CORDIC hiperbolik untuk sigmoid dan tanh, mode 8/12/16 iterasi |
| Format internal | Aktivasi Q5.10 (16-bit, 1.0 = 1024), CORDIC Q7.16 (24-bit) |
| Antarmuka | Avalon-MM slave di Lightweight HPS-to-FPGA Bridge + conduit status |
| Clock target | 50 MHz |
| Latensi (simulasi) | 122.452 siklus (CORDIC-8) s.d. 150.100 siklus (CORDIC-16) ≈ 2,45–3,00 ms / window |

Seluruh RTL telah diverifikasi **bit-exact** terhadap golden model Python (lihat bagian 8).

---

## 2. Struktur Folder

```
neuroflex_de10nano/
├── rtl/                      RTL akselerator (VHDL)
│   ├── neuroflex_pkg.vhd     konstanta model, tipe fixed-point, fungsi saturasi
│   ├── hps_bridge_if.vhd     slave Avalon-MM, register kontrol/status, dekode memori
│   ├── input_window_buf.vhd  buffer window IMU INT8 (2 slot, M10K)
│   ├── weight_mem.vhd        bobot INT8 (baris 32-bit) + bias Q5.10 (M10K)
│   ├── int8_mac_engine.vhd   MAC INT8 4 lane, akumulator 24-bit
│   ├── cordic_nfu.vhd        Shared CORDIC NFU (tanh / sigmoid)
│   ├── seq_divider.vhd       pembagi restoring sekuensial (sinh / cosh)
│   ├── hidden_state_buf.vhd  register file h(t-1)/h(t) ping-pong + update GRU
│   ├── gru_controller.vhd    FSM penjadwal gate, neuron, dan timestep
│   ├── dense_argmax.vhd      dense 16 → 2 (reuse MAC) + argmax
│   ├── perf_counter.vhd      penghitung siklus dan jumlah inferensi
│   ├── neuroflex_top.vhd     top-level akselerator (komponen Platform Designer)
│   └── de10nano_top.vhd      top-level demo board mandiri (KEY/SW/LED)
├── tb/                       testbench
│   ├── tb_int8_mac.vhd       unit test MAC (corner case + acak)
│   ├── tb_cordic_nfu.vhd     unit test CORDIC (bit-exact + error vs float)
│   └── tb_neuroflex_top.vhd  integration test end-to-end via Avalon
├── sim/                      script simulasi + test vector
│   ├── run_ghdl.sh           GHDL
│   ├── run_modelsim.do       ModelSim / Questa
│   ├── cordic_ref.txt        vektor CORDIC (mode, fungsi, z, golden, ideal)
│   ├── model_load.txt        bobot + bias dalam format "ALAMAT DATA" (hex)
│   ├── windows_load.txt      2 window contoh (NORMAL, ANOMALI)
│   └── expected.txt          hasil golden model (slot, mode, kelas, score)
├── quartus/                  proyek Quartus Prime
│   ├── neuroflex_de10nano.qpf / .qsf / .sdc
│   ├── neuroflex_hw.tcl      komponen Platform Designer
│   ├── tinygru_wrow.mif      inisialisasi baris bobot
│   ├── tinygru_bias.mif      inisialisasi bias
│   └── imu_window.mif        inisialisasi 2 window contoh
├── python/
│   ├── train_quantize.py     training TinyGRU, kuantisasi INT8, ekspor MIF/vektor
│   ├── golden_model.py       golden model fixed-point bit-exact + generator vektor
│   └── out/                  model hasil training dan test set terkuantisasi
└── sw/
    ├── hps_driver.c          program Linux HPS (mmap /dev/mem)
    ├── Makefile
    ├── win_normal.csv        contoh window NORMAL (64 × 6 INT8)
    └── win_anomaly.csv       contoh window ANOMALI
```

---

## 3. Arsitektur

```
             HPS (ARM Cortex-A9, Linux)
                       │  Lightweight HPS-to-FPGA Bridge (Avalon-MM)
┌──────────────────────▼──────────────────────────────────────────────┐
│ hps_bridge_if ── CTRL/STATUS/RESULT/CYCLES ── perf_counter          │
│      │ tulis memori                                                 │
│      ├──► input_window_buf ──x(t)──┐                                │
│      └──► weight_mem ──W,U──┐      │                                │
│                             ▼      ▼                                │
│                      int8_mac_engine ◄── op vector [x(t) | int8(h)] │
│                             │ acc 24-bit                            │
│                             ▼                                       │
│  gru_controller ──► preact Q5.10 ──► cordic_nfu (sigmoid / tanh)    │
│        │                                   │                        │
│        └──────────► hidden_state_buf ◄─────┘   h(t) ping-pong       │
│                             │ h(T)                                  │
│                             ▼                                       │
│                      dense_argmax ──► class, score                  │
└─────────────────────────────────────────────────────────────────────┘
```

### Persamaan GRU (konvensi PyTorch `nn.GRU`)

Untuk setiap timestep `t` dan neuron `j` (dijadwalkan berurutan oleh `gru_controller`):

```
z_j = sigmoid( Wz[j]·x + Uz[j]·h + bz[j] )
r_j = sigmoid( Wr[j]·x + Ur[j]·h + br[j] )
n_j = tanh   ( Wn[j]·x + b_in[j] + r_j * (Un[j]·h + b_hn[j]) )
h'_j = n_j + z_j * (h_j - n_j)          = (1 - z_j)·n_j + z_j·h_j
```

Konvensi ini sama dengan `torch.nn.GRU`, sehingga bobot hasil training dapat langsung diekspor
tanpa modifikasi. `h` dibaca dari bank aktif `hidden_state_buf`, sedangkan `h'` ditulis ke bank
lain. Bank ditukar di akhir timestep.

### Format bilangan

| Besaran | Format | Skala |
|---|---|---|
| Input IMU `x` | INT8 | 2⁻⁷ (input dinormalisasi ke [-1, 1): akselerometer / 2 g, giroskop / full-scale) |
| Bobot `W`, `U`, dense | INT8 | 2⁻⁶ (rentang ±2) |
| Hidden state ke MAC | INT8 = sat(h >> 3) | 2⁻⁷ |
| Akumulator | INT24 | 2⁻¹³ → `acc >> ACC_SHIFT(3)` = Q5.10 |
| Bias, pre-activation, gate, h | Q5.10 (16-bit) | 2⁻¹⁰ |
| CORDIC internal | Q7.16 (24-bit) | 2⁻¹⁶ |

Skala bobot dibuat pangkat dua agar konversi akumulator ke Q5.10 cukup dilakukan dengan
*arithmetic shift*, tanpa multiplier tambahan.

### Shared CORDIC NFU

1. `tanh(z)`: argumen direduksi menjadi `z/4` (konvergensi CORDIC hiperbolik ±1,118). Rotasi
   CORDIC menghasilkan `cosh`/`sinh`, lalu dua langkah *double-angle*
   (`c' = c² + s²`, `s' = 2sc`) menghasilkan `cosh z` dan `sinh z`.
   `tanh = sinh / cosh` dihitung oleh `seq_divider` (11 siklus).
2. `sigmoid(z) = 0,5 × (1 + tanh(z/2))`, dengan satu jalur hardware yang sama.
3. Untuk `|z| ≥ 4` keluaran disaturasi ke ±1023/1024 (selisih < 1 LSB dari nilai sebenarnya).
4. Mode iterasi: `00` = 8, `01` = 12, `1x` = 16. Iterasi i = 4 dan 13 diulang.
5. Latensi: 26–36 siklus per fungsi (tergantung mode).

---

## 4. Modul RTL

| Modul | Fungsi | Implementasi |
|---|---|---|
| `hps_bridge_if` | Register kontrol/status, dekode tulis memori | Avalon-MM, read latency 1 |
| `input_window_buf` | 2 slot window IMU 64×6 INT8 (double buffering) | M10K 192×32 |
| `weight_mem` | Baris bobot Z/R/H + dense, bias Q5.10 | M10K 300×32, 66×16, init `.mif` |
| `int8_mac_engine` | Dot product 4 lane per siklus | 4 multiplier 8×8, akumulator INT24 |
| `cordic_nfu` + `seq_divider` | sigmoid / tanh | CORDIC hiperbolik 8/12/16 iterasi |
| `hidden_state_buf` | h(t-1)/h(t), update GRU | Register ping-pong 2×16×16 bit |
| `gru_controller` | FSM gate → neuron → timestep → dense | FSM |
| `dense_argmax` | Dense 16 → 2 + argmax + margin score | Reuse MAC + comparator |
| `perf_counter` | Siklus per inferensi, total inferensi | Counter 32-bit |
| `neuroflex_top` | Integrasi semua modul | Komponen Platform Designer |
| `de10nano_top` | Demo mandiri tanpa HPS | Debounce KEY, LED status |

---

## 5. Peta Register (Avalon-MM, span 8 KB)

Offset berupa **alamat byte** relatif terhadap base address komponen. Semua akses harus
berupa **word 32-bit** (slave tidak memiliki `byteenable`).

| Offset | Nama | Akses | Keterangan |
|---|---|---|---|
| `0x0000` | CTRL | R/W | bit0 START (pulsa, diabaikan saat busy), bit[2:1] MODE CORDIC, bit3 SLOT window |
| `0x0004` | STATUS | R | bit0 BUSY, bit1 DONE (di-clear saat START) |
| `0x0008` | RESULT | R | bit[7:0] kelas (0 = NORMAL, 1 = ANOMALI), bit[31:16] score (Q5.10, selisih logit) |
| `0x000C` | CYCLES | R | jumlah siklus inferensi terakhir |
| `0x0010` | INFER_CNT | R | total inferensi sejak reset |
| `0x0014` | ID | R | `0x4E460100` ("NF" v1.0) |
| `0x0018` | CONFIG | R | [7:0] N_IN, [15:8] N_HID, [23:16] N_CLS, [31:24] SEQ_LEN |
| `0x0800`–`0x0FFF` | WROW | W | baris bobot, 1 word = 4 bobot INT8 (little-endian) |
| `0x1000`–`0x17FF` | WINBUF | W | window IMU; slot 0 di `0x1000`, slot 1 di `0x1000 + 384` |
| `0x1800`–`0x1FFF` | BIAS | W | bias Q5.10 di bit[15:0] |

### Layout memori bobot

Setiap baris berisi `ROW_WORDS = 6` word (24 elemen INT8):

```
elemen 0..5   : W[j][0..5]   (bagian x)
elemen 6..7   : 0            (padding)
elemen 8..23  : U[j][0..15]  (bagian h)
```

Urutan baris: Z (16 baris), R (16), H (16), lalu dense (2 baris, bagian x = 0).
Alamat word baris `r` dan word `k` adalah `r × 6 + k`.
Urutan bias: `bz[16]`, `br[16]`, `b_in[16]`, `b_hn[16]`, `bd[2]`.

Window: sampel `x[t][i]` berada pada byte `slot × 384 + t × 6 + i`.

---

## 6. Alur Kerja

### 6.1 Training, kuantisasi, dan ekspor (Python)

Kebutuhan: Python 3.10+, `numpy`, `torch`.

```bash
cd python
python train_quantize.py            # dataset sintetis; atau --data dataset.npz
python golden_model.py              # vektor uji, window contoh, tabel error CORDIC
```

`train_quantize.py` menghasilkan `quartus/tinygru_wrow.mif`, `quartus/tinygru_bias.mif`,
`sim/model_load.txt`, dan `python/out/*.npz`. `golden_model.py` menghasilkan
`sim/cordic_ref.txt`, `sim/windows_load.txt`, `sim/expected.txt`, `quartus/imu_window.mif`,
dan `sw/win_*.csv`.

Format dataset sendiri (`--data`): file `.npz` dengan `x` float `[N, 64, 6]` yang sudah
dinormalisasi ke [-1, 1) dan `y` int `[N]` (0 = normal, 1 = anomali).

> Setelah training ulang, jalankan kembali kedua skrip agar MIF, vektor simulasi, dan
> `expected.txt` tetap konsisten.

### 6.2 Simulasi RTL

**GHDL** (≥ 3.0):

```bash
cd sim
./run_ghdl.sh                       # semua testbench
./run_ghdl.sh tb_cordic_nfu         # satu testbench
WAVE=1 ./run_ghdl.sh tb_int8_mac    # dengan waveform .ghw (GTKWave)
```

**ModelSim / Questa Intel FPGA Edition**:

```
cd sim
vsim -c -do run_modelsim.do
```

Testbench integrasi mensimulasikan ±19 ms waktu (7 inferensi). Di GHDL durasinya sekitar 1 menit.

### 6.3 Demo mandiri di board (tanpa HPS)

1. Buka `quartus/neuroflex_de10nano.qpf` di Quartus Prime (Lite/Standard 18.1+).
2. *Processing → Start Compilation*.
3. Program `output_files/neuroflex_de10nano.sof` via USB-Blaster II
   (*Tools → Programmer*, pilih device `5CSEBA6U23` pada chain).

Bobot dan dua window contoh dimuat dari file `.mif`.

| Input / Output | Fungsi |
|---|---|
| KEY0 | Reset |
| KEY1 | Mulai inferensi |
| SW1..SW0 | Mode CORDIC (`00` = 8, `01` = 12, `1x` = 16 iterasi) |
| SW2 | Slot window (0 = contoh NORMAL, 1 = contoh ANOMALI) |
| LED0 | Kelas (menyala = ANOMALI) |
| LED1 | Busy |
| LED2 | Done |
| LED3 | Heartbeat |
| LED7..LED4 | Bagian bulat score (margin logit) |

### 6.4 Integrasi dengan HPS (Platform Designer)

1. Gunakan GHRD DE10-Nano dari Terasic (`DE10_NANO_SoC_GHRD`) sebagai basis proyek.
2. Salin `rtl/`, `quartus/neuroflex_hw.tcl`, dan ketiga file `.mif` ke proyek GHRD dengan
   struktur relatif yang sama (`neuroflex_hw.tcl` merujuk `../rtl/*.vhd`), atau sesuaikan path
   di `neuroflex_hw.tcl`.
3. Di Platform Designer: *Tools → Options → IP Search Path*, tambahkan folder yang berisi
   `neuroflex_hw.tcl`. Komponen muncul di grup **NeuroFlex**.
4. Tambahkan komponen, lalu hubungkan:
   - `clock` ← `clk_0.clk` (50 MHz), `reset` ← `clk_0.clk_reset`
   - `s0` ← `hps_0.h2f_lw_axi_master`, base address **`0x0004_0000`**
     (tidak bertabrakan dengan IP bawaan GHRD; alamat fisik = `0xFF24_0000`)
   - `status`: *export*, lalu di top-level GHRD hubungkan `ext_start`, `ext_mode`, dan
     `ext_win_sel` ke `'0'` (atau ke KEY/SW), dan `st_*` ke LED jika diinginkan.
5. *Generate HDL*, compile proyek, lalu buat `.rbf` untuk boot Linux sesuai prosedur GHRD.
6. Build driver:

```bash
cd sw
make                                   # cross-compile (arm-linux-gnueabihf-gcc)
make CROSS_COMPILE=                    # atau langsung di board
make NF_OFFSET=0x00040000              # sesuaikan dengan base address s0
```

7. Jalankan di board (sebagai root):

```bash
./hps_driver model_load.txt win_anomaly.csv 2 0
# NeuroFlex ID=0x4E460100  N_IN=6 N_HID=16 N_CLS=2 SEQ_LEN=64
# Model: 366 word dimuat dari model_load.txt
# class=ANOMALI score=10.045 cycles=150100 (3002.00 us @50MHz) total_infer=1
```

Argumen: `<model_load.txt> <window.csv> [mode 0|1|2] [slot 0|1]`. Dengan dua slot, HPS dapat
mengisi window berikutnya selagi akselerator memproses slot lain.

---

## 7. Mengubah Parameter Model

Parameter ada di `rtl/neuroflex_pkg.vhd` dan **harus sama** dengan konstanta di
`python/golden_model.py`:

| Konstanta | Default | Batas peta memori |
|---|---|---|
| `N_IN` | 6 | — |
| `N_HID` | 16 | `(3·N_HID + N_CLS) · ROW_WORDS ≤ 512` |
| `N_CLS` | 2 | `4·N_HID + N_CLS ≤ 512` |
| `SEQ_LEN` | 64 | `2 · SEQ_LEN · N_IN ≤ 2048` byte |
| `ACC_SHIFT` | 3 | skala bobot = 2^-(ACC_SHIFT+3) |

`neuroflex_top` berisi assertion yang menghentikan elaborasi bila model tidak muat di peta
memori. Untuk model multikelas, `class_out` sudah 8-bit dan argmax bersifat generik.

---

## 8. Hasil Verifikasi

### Simulasi (GHDL 6.0)

| Testbench | Hasil |
|---|---|
| `tb_int8_mac` | PASS: −128 × −128 × 32 siklus, prioritas clear, 4 × 250 vektor acak |
| `tb_cordic_nfu` | PASS: 2.046 vektor, **bit-exact** dengan golden model |
| `tb_neuroflex_top` | PASS: 2 window × 3 mode, kelas dan score **bit-exact** dengan golden model; jalur conduit `ext_start` OK |

### Error CORDIC terhadap floating-point (z ∈ [-4, 4), satuan nilai riil)

| Mode | Iterasi | tanh MAE | tanh max | sigmoid MAE | sigmoid max |
|---|---|---|---|---|---|
| 00 | 8 | 0,00217 | 0,01568 | 0,00193 | 0,00833 |
| 01 | 12 | 0,00051 | 0,00147 | 0,00039 | 0,00125 |
| 1x | 16 | 0,00047 | 0,00105 | 0,00035 | 0,00099 |

Untuk mode 12 dan 16, error maksimum ≤ 1 LSB Q5.10, sehingga sudah dibatasi oleh resolusi
format keluaran.

### Latensi per window (clock 50 MHz)

| Mode | Siklus | Waktu | Throughput |
|---|---|---|---|
| CORDIC-8 | 122.452 | 2,45 ms | ≈ 408 window/s |
| CORDIC-12 | 134.740 | 2,69 ms | ≈ 371 window/s |
| CORDIC-16 | 150.100 | 3,00 ms | ≈ 333 window/s |

### Akurasi model (dataset sintetis)

| Model | Akurasi test |
|---|---|
| FP32 (PyTorch) | 99,3 % (300 window) |
| INT8 + CORDIC-16 (golden model = RTL) | 99,0 % (200 window) |

Resource (ALM, M10K, DSP), Fmax, dan daya belum tercantum karena harus diambil dari laporan
Quartus Fitter / Timing Analyzer / Power Analyzer setelah kompilasi.

---

## 9. Catatan

- Semua tulisan dari HPS harus berupa word 32-bit. Window dikemas 4 sampel per word
  (little-endian), seperti yang dilakukan `hps_driver.c`.
- Jangan menulis ke slot window atau bobot yang sedang dipakai selama `BUSY = 1`. Gunakan slot
  lain untuk double buffering.
- Penulisan START diabaikan selama akselerator sedang busy.
- Dataset bawaan bersifat sintetis (getaran rotor 40–80 Hz, anomali = imbalance propeller).
  Untuk hasil nyata, latih ulang dengan data IMU dari bench-test rig atau UAV.
- Perbedaan dengan snippet pada lampiran proposal: persamaan GRU mengikuti `nn.GRU`
  (agar bobot PyTorch dapat langsung dipakai), skala bobot pangkat dua, CORDIC memakai reduksi
  argumen `z/4` + double-angle agar akurat di luar rentang konvergensi ±1,118, dan window
  ditulis per word 32-bit.
