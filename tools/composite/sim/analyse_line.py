"""Latency and level checks from a tb_equiv line.txt (line 60: colour bars).
Columns: hc cvbs r_ref g_ref b_ref r_psx g_psx b_psx."""
import sys
rows = [list(map(int, l.split())) for l in open(sys.argv[1])]
hc = [r[0] for r in rows]
cv = [r[1] / 2**21 for r in rows]
# Source: black -> white step lands between hc 608 and 609 (bar 0 starts at 608).
src = 608.5
r1 = [r[5] for r in rows]
for i in range(1, len(rows)):
    if hc[i] > 600 and r1[i - 1] < 128 <= r1[i]:
        x = hc[i - 1] + (128 - r1[i - 1]) / (r1[i] - r1[i - 1])
        print(f'decoded 50% crossing at {x:.1f}: {x - src:.1f} samples after the encoder input')
        break
blank = sum(cv[hc.index(400):hc.index(440)]) / 40
tip = sum(cv[hc.index(100):hc.index(180)]) / 80
print(f'sync tip {tip - blank:+.3f} V below blanking')
bars = ['white', 'yellow', 'cyan', 'green', 'magenta', 'red', 'blue', 'black']
for k, name in enumerate(bars):
    a = 608 + k * 320 + 120
    seg = [rows[hc.index(a + 54 + j)] for j in range(60)]
    y = sum(cv[hc.index(a + j)] for j in range(60)) / 60 - blank
    pp = max(cv[hc.index(a + j)] for j in range(60)) - min(cv[hc.index(a + j)] for j in range(60))
    m = [sum(s[c] for s in seg) / len(seg) for c in range(2, 8)]
    print(f'{name:8s} cvbs mean {y:+.3f} V p-p {pp:.3f} V | ref RGB {m[0]:5.1f} {m[1]:5.1f} {m[2]:5.1f}'
          f' | psx RGB {m[3]:5.1f} {m[4]:5.1f} {m[5]:5.1f}')
# Burst: 0.6 us after sync end (sync ends at hc 220 for this timing).
seg = cv[hc.index(220 + 40):hc.index(220 + 140)]
print(f'burst p-p {max(seg) - min(seg):.3f} V')
