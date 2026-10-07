"""Train TinyGRU (6 -> 16 -> 2) on IMU windows, quantize to INT8 and export
the memory images used by the RTL (MIF for Quartus, Avalon load file for
simulation and the HPS driver).

Without --data a synthetic UAV-like vibration dataset is generated:
NORMAL = low vibration amplitude, ANOMALI = propeller imbalance (large
1x rotor component + 2x harmonic).

--data expects an .npz with x: float [N, SEQ_LEN, 6] already normalised to
[-1, 1) (accel / 2 g, gyro / full-scale) and y: int [N] (0 normal, 1 anomali).
"""
import argparse

import numpy as np
import torch
import torch.nn as nn

from golden_model import (ACC_SHIFT, FRAC, N_CLS, N_HID, N_IN, OUT, SEQ_LEN,
                          W_SHIFT, X_SHIFT, export_model)

W_LIM = (127 / (1 << W_SHIFT), -128 / (1 << W_SHIFT))


class TinyGRU(nn.Module):
    def __init__(self, n_in=N_IN, n_hid=N_HID, n_cls=N_CLS):
        super().__init__()
        self.gru = nn.GRU(n_in, n_hid, batch_first=True)
        self.dense = nn.Linear(n_hid, n_cls)

    def forward(self, x):
        _, h = self.gru(x)
        return self.dense(h[-1])


def synth_imu(n, anomaly, rng, fs=400.0):
    t = np.arange(SEQ_LEN) / fs
    x = np.zeros((n, SEQ_LEN, 6))
    for k in range(n):
        f_rot = rng.uniform(40, 80)
        ph = rng.uniform(0, 2 * np.pi, 8)
        acc = np.zeros((SEQ_LEN, 3))
        acc[:, 2] = 1.0
        gyr = np.zeros((SEQ_LEN, 3))
        slow = rng.uniform(-0.15, 0.15, 3)[None, :] * np.sin(2 * np.pi * rng.uniform(0.5, 3) * t + ph[6])[:, None]
        acc += slow
        gyr += 0.5 * slow
        amp = rng.uniform(0.18, 0.5) if anomaly else rng.uniform(0.02, 0.10)
        for a in range(3):
            acc[:, a] += amp * np.sin(2 * np.pi * f_rot * t + ph[a])
            gyr[:, a] += 0.5 * amp * np.sin(2 * np.pi * f_rot * t + ph[a + 3])
            if anomaly:
                acc[:, a] += 0.3 * amp * np.sin(4 * np.pi * f_rot * t + ph[7])
        acc += 0.04 * rng.standard_normal(acc.shape)
        gyr += 0.02 * rng.standard_normal(gyr.shape)
        x[k, :, :3] = acc / 2.0
        x[k, :, 3:] = gyr
    return np.clip(x, -1.0, 127 / 128)


def make_dataset(n_per_class, seed):
    rng = np.random.default_rng(seed)
    x = np.concatenate([synth_imu(n_per_class, False, rng), synth_imu(n_per_class, True, rng)])
    y = np.concatenate([np.zeros(n_per_class, dtype=np.int64), np.ones(n_per_class, dtype=np.int64)])
    p = rng.permutation(len(y))
    return x[p], y[p]


def quantize_input(x):
    return np.clip(np.round(x * (1 << X_SHIFT)), -128, 127).astype(np.int8)


def quantize_w(w):
    return torch.clamp(torch.round(w * (1 << W_SHIFT)), -128, 127).to(torch.int64).numpy()


def quantize_b(b):
    return torch.clamp(torch.round(b * (1 << FRAC)), -32768, 32767).to(torch.int64).numpy()


def export_int8(model):
    g = model.gru
    w_ih, w_hh = g.weight_ih_l0.detach(), g.weight_hh_l0.detach()
    b_ih, b_hh = g.bias_ih_l0.detach(), g.bias_hh_l0.detach()
    Wr, Wz, Wn = w_ih.chunk(3)
    Ur, Uz, Un = w_hh.chunk(3)
    bir, biz, bin_ = b_ih.chunk(3)
    bhr, bhz, bhn = b_hh.chunk(3)
    return {
        "Wz": quantize_w(Wz), "Uz": quantize_w(Uz), "bz": quantize_b(biz + bhz),
        "Wr": quantize_w(Wr), "Ur": quantize_w(Ur), "br": quantize_b(bir + bhr),
        "Wn": quantize_w(Wn), "Un": quantize_w(Un),
        "b_in": quantize_b(bin_), "b_hn": quantize_b(bhn),
        "Wd": quantize_w(model.dense.weight.detach()), "bd": quantize_b(model.dense.bias.detach()),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", help="npz dataset (x, y); default: synthetic")
    ap.add_argument("--epochs", type=int, default=25)
    ap.add_argument("--seed", type=int, default=7)
    args = ap.parse_args()

    torch.manual_seed(args.seed)
    if args.data:
        d = np.load(args.data)
        x, y = d["x"], d["y"]
        n_tr = int(0.8 * len(y))
        x_tr, y_tr, x_te, y_te = x[:n_tr], y[:n_tr], x[n_tr:], y[n_tr:]
    else:
        x_tr, y_tr = make_dataset(1000, args.seed)
        x_te, y_te = make_dataset(150, args.seed + 1)

    xq_tr, xq_te = quantize_input(x_tr), quantize_input(x_te)
    xt = torch.tensor(xq_tr / (1 << X_SHIFT), dtype=torch.float32)
    yt = torch.tensor(y_tr)
    xv = torch.tensor(xq_te / (1 << X_SHIFT), dtype=torch.float32)
    yv = torch.tensor(y_te)

    model = TinyGRU()
    opt = torch.optim.Adam(model.parameters(), lr=1e-2)
    loss_fn = nn.CrossEntropyLoss()
    for ep in range(args.epochs):
        model.train()
        perm = torch.randperm(len(yt))
        for i in range(0, len(yt), 64):
            idx = perm[i:i + 64]
            opt.zero_grad()
            loss = loss_fn(model(xt[idx]), yt[idx])
            loss.backward()
            opt.step()
            with torch.no_grad():
                for name, p in model.named_parameters():
                    if "weight" in name:
                        p.clamp_(W_LIM[1], W_LIM[0])
        model.eval()
        with torch.no_grad():
            acc = (model(xv).argmax(1) == yv).float().mean().item()
        print(f"epoch {ep + 1:2d}  loss {loss.item():.4f}  akurasi FP32 test {acc:.3f}")

    OUT.mkdir(exist_ok=True)
    q = export_int8(model)
    np.savez(OUT / "model_int8.npz", **q)
    np.savez(OUT / "dataset_test.npz", x=xq_te, y=y_te)
    torch.save(model.state_dict(), OUT / "tinygru_fp32.pt")
    export_model(q)
    n_par = sum(p.numel() for p in model.parameters())
    print(f"Parameter: {n_par}, ACC_SHIFT={ACC_SHIFT}, skala bobot 2^-{W_SHIFT}, skala input 2^-{X_SHIFT}")
    print("Selesai: quartus/tinygru_wrow.mif, quartus/tinygru_bias.mif, sim/model_load.txt")


if __name__ == "__main__":
    main()
