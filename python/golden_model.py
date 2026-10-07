"""Bit-exact fixed-point golden model of the NeuroFlex-CORDIC datapath.

Mirrors the RTL: INT8 MAC with 24-bit accumulator, Q5.10 activations,
hyperbolic CORDIC (Q7.16 internal) with x/4 argument reduction and two
double-angle steps, restoring divider, PyTorch-style GRU equations.

Run after train_quantize.py to generate the simulation vectors, the
window MIF for the standalone demo and the CORDIC error table.
"""
import math
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
OUT = Path(__file__).resolve().parent / "out"

N_IN, N_HID, N_CLS, SEQ_LEN = 6, 16, 2, 64
WIN_SLOTS, LANES = 2, 4
X_WORDS = (N_IN + LANES - 1) // LANES
H_WORDS = (N_HID + LANES - 1) // LANES
ROW_WORDS = X_WORDS + H_WORDS
N_ROWS = 3 * N_HID + N_CLS
N_BIAS = 4 * N_HID + N_CLS
WIN_LEN = SEQ_LEN * N_IN

FRAC, ONE = 10, 1 << 10
ACC_SHIFT = 3
W_SHIFT = ACC_SHIFT + 3      # weight scale 2^-6
X_SHIFT = 7                  # input / hidden INT8 scale 2^-7

F_C = 16
ATANH = [round(math.atanh(2.0 ** -i) * (1 << F_C)) for i in range(1, 17)]
X0_KH = 79135                # 1/Kh in Q.16
Z_SAT = 4 << F_C
TANH_SAT = ONE - 1
N_ITER = {0: 8, 1: 12, 2: 16, 3: 16}

BASE_WROW = 0x0800
BASE_WIN = 0x1000
BASE_BIAS = 0x1800


def sat(v, bits):
    lo, hi = -(1 << (bits - 1)), (1 << (bits - 1)) - 1
    return lo if v < lo else hi if v > hi else v


def sat16(v):
    return sat(v, 16)


def to_int8(h):
    return sat(h >> (FRAC - X_SHIFT), 8)


def _tanh_core(zi, n_iter):
    if zi >= Z_SAT:
        return TANH_SAT
    if zi <= -Z_SAT:
        return -TANH_SAT
    x, y, z = X0_KH, 0, zi >> 2
    i, rep = 1, False
    while True:
        if z >= 0:
            x, y, z = x + (y >> i), y + (x >> i), z - ATANH[i - 1]
        else:
            x, y, z = x - (y >> i), y - (x >> i), z + ATANH[i - 1]
        if i in (4, 13) and not rep:
            rep = True
        else:
            rep = False
            if i == n_iter:
                break
            i += 1
    for _ in range(2):
        x, y = (x * x + y * y) >> F_C, (x * y) >> (F_C - 1)
    num, den = abs(y), x
    mag = ONE if num >= den else (num << FRAC) // den
    return -mag if y < 0 else mag


def cordic_tanh(z, n_iter=16):
    return _tanh_core(z << (F_C - FRAC), n_iter)


def cordic_sigmoid(z, n_iter=16):
    return (ONE + _tanh_core((z << (F_C - FRAC)) >> 1, n_iter)) >> 1


def dot(w, v):
    return sum(int(a) * int(b) for a, b in zip(w, v))


def infer(win, m, mode=2):
    """win: SEQ_LEN x N_IN int8, m: dict of quantized tensors."""
    n_iter = N_ITER[mode]
    h = [0] * N_HID
    for t in range(SEQ_LEN):
        x = [int(v) for v in win[t]]
        h8 = [to_int8(v) for v in h]
        h_new = []
        for j in range(N_HID):
            z = cordic_sigmoid(sat16((dot(m["Wz"][j], x) + dot(m["Uz"][j], h8) >> ACC_SHIFT) + m["bz"][j]), n_iter)
            r = cordic_sigmoid(sat16((dot(m["Wr"][j], x) + dot(m["Ur"][j], h8) >> ACC_SHIFT) + m["br"][j]), n_iter)
            ax = sat16((dot(m["Wn"][j], x) >> ACC_SHIFT) + m["b_in"][j])
            bh = sat16((dot(m["Un"][j], h8) >> ACC_SHIFT) + m["b_hn"][j])
            n = cordic_tanh(sat16(ax + ((r * bh) >> FRAC)), n_iter)
            h_new.append(sat16(n + ((z * (h[j] - n)) >> FRAC)))
        h = h_new
    h8 = [to_int8(v) for v in h]
    logits = [(dot(m["Wd"][k], h8) >> ACC_SHIFT) + int(m["bd"][k]) for k in range(N_CLS)]
    best, second, cls = logits[0], None, 0
    for k in range(1, N_CLS):
        if logits[k] > best:
            second, best, cls = best, logits[k], k
        elif second is None or logits[k] > second:
            second = logits[k]
    score = sat16(best - second) if N_CLS > 1 else 0
    return cls, score, logits


def load_model(path=OUT / "model_int8.npz"):
    d = np.load(path)
    return {k: d[k].astype(np.int64) for k in d.files}


def wrow_words(m):
    rows = [(m["Wz"][j], m["Uz"][j]) for j in range(N_HID)]
    rows += [(m["Wr"][j], m["Ur"][j]) for j in range(N_HID)]
    rows += [(m["Wn"][j], m["Un"][j]) for j in range(N_HID)]
    rows += [(np.zeros(N_IN, dtype=np.int64), m["Wd"][k]) for k in range(N_CLS)]
    words = []
    for wx, uh in rows:
        elems = np.zeros(ROW_WORDS * LANES, dtype=np.int64)
        elems[:N_IN] = wx
        elems[X_WORDS * LANES:X_WORDS * LANES + N_HID] = uh
        for w in range(ROW_WORDS):
            words.append(pack_word(elems[w * LANES:(w + 1) * LANES]))
    return words


def bias_values(m):
    return [int(v) for v in np.concatenate([m["bz"], m["br"], m["b_in"], m["b_hn"], m["bd"]])]


def pack_word(b4):
    return sum((int(v) & 0xFF) << (8 * i) for i, v in enumerate(b4))


def window_words(wins):
    flat = np.zeros(WIN_SLOTS * WIN_LEN, dtype=np.int64)
    for s, w in enumerate(wins):
        flat[s * WIN_LEN:(s + 1) * WIN_LEN] = np.asarray(w).reshape(-1)
    return [pack_word(flat[i:i + 4]) for i in range(0, len(flat), 4)]


def write_mif(path, values, width):
    mask = (1 << width) - 1
    digits = (width + 3) // 4
    with open(path, "w") as f:
        f.write(f"DEPTH = {len(values)};\nWIDTH = {width};\n")
        f.write("ADDRESS_RADIX = DEC;\nDATA_RADIX = HEX;\nCONTENT BEGIN\n")
        for i, v in enumerate(values):
            f.write(f"  {i} : {int(v) & mask:0{digits}X};\n")
        f.write("END;\n")


def export_model(m):
    write_mif(ROOT / "quartus" / "tinygru_wrow.mif", wrow_words(m), 32)
    write_mif(ROOT / "quartus" / "tinygru_bias.mif", bias_values(m), 16)
    with open(ROOT / "sim" / "model_load.txt", "w") as f:
        for i, w in enumerate(wrow_words(m)):
            f.write(f"{BASE_WROW + 4 * i:04X} {w:08X}\n")
        for i, b in enumerate(bias_values(m)):
            f.write(f"{BASE_BIAS + 4 * i:04X} {b & 0xFFFFFFFF:08X}\n")


def cordic_error_table():
    xs = range(-4 * ONE, 4 * ONE, 4)
    print("mode  iter  tanh MAE   tanh MAX   sigm MAE   sigm MAX   (satuan nilai riil)")
    for mode in (0, 1, 2):
        n = N_ITER[mode]
        et = [abs(cordic_tanh(v, n) / ONE - math.tanh(v / ONE)) for v in xs]
        es = [abs(cordic_sigmoid(v, n) / ONE - 1 / (1 + math.exp(-v / ONE))) for v in xs]
        print(f"  {mode:2d}   {n:3d}  {np.mean(et):.6f}  {max(et):.6f}  {np.mean(es):.6f}  {max(es):.6f}")


def write_cordic_vectors():
    zs = list(range(-6 * ONE, 6 * ONE, 37)) + [-32768, -20000, -4097, -4096, 4095, 4096, 20000, 32767]
    with open(ROOT / "sim" / "cordic_ref.txt", "w") as f:
        for mode in (0, 1, 2):
            n = N_ITER[mode]
            for sig in (0, 1):
                for z in zs:
                    if sig:
                        g = cordic_sigmoid(z, n)
                        ideal = round(ONE / (1 + math.exp(-z / ONE)))
                    else:
                        g = cordic_tanh(z, n)
                        ideal = round(math.tanh(z / ONE) * ONE)
                    f.write(f"{mode} {sig} {z} {g} {ideal}\n")


def main():
    m = load_model()
    data = np.load(OUT / "dataset_test.npz")
    xq, y = data["x"], data["y"]

    cordic_error_table()
    write_cordic_vectors()

    n_eval = min(len(y), 200)
    correct = sum(infer(xq[i], m)[0] == y[i] for i in range(n_eval))
    print(f"Akurasi golden model INT8 + CORDIC-16 : {correct}/{n_eval} = {correct / n_eval:.3f}")

    i_norm = next(i for i in range(len(y)) if y[i] == 0 and infer(xq[i], m)[0] == 0)
    i_anom = next(i for i in range(len(y)) if y[i] == 1 and infer(xq[i], m)[0] == 1)
    wins = [xq[i_norm], xq[i_anom]]

    write_mif(ROOT / "quartus" / "imu_window.mif", window_words(wins), 32)
    with open(ROOT / "sim" / "windows_load.txt", "w") as f:
        for i, w in enumerate(window_words(wins)):
            f.write(f"{BASE_WIN + 4 * i:04X} {w:08X}\n")
    for name, w in (("win_normal.csv", wins[0]), ("win_anomaly.csv", wins[1])):
        np.savetxt(ROOT / "sw" / name, w, fmt="%d", delimiter=",")

    with open(ROOT / "sim" / "expected.txt", "w") as f:
        for slot, w in enumerate(wins):
            for mode in (0, 1, 2):
                cls, score, logits = infer(w, m, mode)
                f.write(f"{slot} {mode} {cls} {score}\n")
                print(f"slot {slot} mode {mode}: class={cls} score={score} logits={logits}")


if __name__ == "__main__":
    main()
