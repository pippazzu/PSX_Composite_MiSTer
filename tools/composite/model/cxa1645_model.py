"""CXA1645 encoder constants for the PSX composite core.

Writes the RGB-to-YUV contribution ROM used by rtl/composite/cxa1645.sv and
prints the derived filter coefficients so they can be checked against the RTL.
Every number comes from the Sony CXA1645P/M data sheet typical values, the
NTSC standard, or a stated assumption (see docs/composite/PARAMETERS.md).

Run:  python tools/composite/model/cxa1645_model.py
"""
import cmath
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]   # repository root
ROM = ROOT / 'rtl' / 'composite' / 'cxa1645_rgb.hex'

FS = 53.693175e6          # CLK_VIDEO, 15 x fsc
FSC = FS / 15
T = 1 / FS
Q = 65536                 # Q2.16 volts

WHITE = 0.71              # loaded Y/CV white, data sheet typ (0.60-0.82)
SYNC = 0.29               # loaded sync, typ (0.26-0.33)
BURST_PP = 0.25           # NTSC burst, typ (0.20-0.30)
RGB_FS = 1.0              # PSX DAC full scale at RIN/GIN/BIN: assumption
# Chroma keeps the standard NTSC chroma-to-burst ratios, which the sheet's
# typical R/G/B ratios (3.16/2.95/2.24) and phases (104/241/347 deg) match.
CHROMA = (BURST_PP / 2) / (0.2 * 0.714) * 0.714   # volts per unit (B-Y)/(R-Y) weight


def q(v, bits=16):
    n = round(v * Q)
    lim = 1 << (bits - 1)
    assert -lim <= n < lim, (v, n)
    return n & ((1 << bits) - 1)


def write_rom():
    w = {'r': 0.299, 'g': 0.587, 'b': 0.114}
    lines = []
    for ch in 'rgb':
        for code in range(256):
            v = code / 255 * RGB_FS
            y = WHITE * w[ch] * v
            by = ((1.0 if ch == 'b' else 0.0) - w[ch]) * v
            ry = ((1.0 if ch == 'r' else 0.0) - w[ch]) * v
            u = CHROMA * 0.493 * by
            vv = CHROMA * 0.877 * ry
            lines.append(f'{q(y):04x}{q(u):04x}{q(vv):04x}')
    ROM.write_text('\n'.join(lines) + '\n')
    print(f'wrote {ROM.relative_to(ROOT)}: 3 x 256 words {{Y,U,V}} Q2.16')


def chroma_check():
    # Ratios and phases the RTL reproduces, for comparison with the sheet.
    for name, (r, g, b) in {'R': (1, 0, 0), 'G': (0, 1, 0), 'B': (0, 0, 1)}.items():
        y = 0.299 * r + 0.587 * g + 0.114 * b
        u = CHROMA * 0.493 * (b - y)
        v = CHROMA * 0.877 * (r - y)
        z = complex(u, v)
        print(f'  {name}: chroma/burst {2 * abs(z) / BURST_PP:.2f}, phase '
              f'{math.degrees(cmath.phase(z)) % 360:.1f} deg, Y {WHITE * y:.3f} V')


def one_pole(fc):
    return 1 - math.exp(-2 * math.pi * fc * T)


def main():
    write_rom()
    print(f'chroma scale {CHROMA:.4f} V; burst peak {BURST_PP / 2 * Q:.0f} Q2.16; '
          f'sync {SYNC * Q:.0f} Q2.16')
    chroma_check()

    # Internal chroma BPF (fo set by FO = 20k). Its bandwidth is not given;
    # assumption: -3 dB at fsc +/- 1.3 MHz, Q = 1.38, which acts on the colour
    # differences as a one-pole low-pass at 1.3 MHz.
    a = one_pole(1.3e6)
    approx = 1 / 8 + 1 / 64 + 1 / 1024
    fc = -math.log(1 - approx) * FS / (2 * math.pi)
    print(f'chroma BPF baseband alpha {a:.5f}; RTL 1/8+1/64+1/1024 = {approx:.5f} '
          f'({fc / 1e6:.3f} MHz); group delay {1 / (2 * math.pi * fc) * 1e9:.0f} ns '
          f'= {FS / (2 * math.pi * fc):.2f} samples')

    # YTRAP, pin 17: internal 1.5 kohm source into the PU-8's series trap,
    # L507 33 uH + an unmarked capacitor, taken as the value tuning it to fsc.
    r = 1500.0
    l = 33e-6
    c = 1 / ((2 * math.pi * FSC) ** 2 * l)
    w0 = 1 / math.sqrt(l * c)
    rl = r / l
    k = w0 / math.tan(w0 * T / 2)   # bilinear, prewarped at the notch
    d0 = k * k + rl * k + w0 * w0
    b0 = (k * k + w0 * w0) / d0
    a1 = 2 * (w0 * w0 - k * k) / d0
    a2 = (k * k - rl * k + w0 * w0) / d0
    s12 = 1 << 12
    q = [round(x * s12) for x in (b0, a1, a2)]
    print(f'PU-8 YTRAP {l * 1e6:.0f} uH + {c * 1e12:.1f} pF: notch {w0 / 2 / math.pi / 1e6:.3f} MHz, '
          f'Q {math.sqrt(l / c) / r:.2f}; Q12 b0 a1 a2 = {q} (RTL TB0, TA1, TA2)')
    for f in (1e6, 2e6, 3e6, FSC, 4.2e6, 6e6):
        z = cmath.exp(-1j * 2 * math.pi * f * T)
        bq, a1q, a2q = [x / s12 for x in q]
        h = (bq * (1 + z * z) + a1q * z) / (1 + a1q * z + a2q * z * z)
        w = 2 * math.pi * f
        ha = (1 - w * w * l * c) / (1 - w * w * l * c + 1j * w * r * c)
        print(f'  {f / 1e6:5.3f} MHz: analog {20 * math.log10(max(abs(ha), 1e-9)):7.2f} dB, '
              f'RTL {20 * math.log10(max(abs(h), 1e-9)):7.2f} dB')

if __name__ == '__main__':
    main()
