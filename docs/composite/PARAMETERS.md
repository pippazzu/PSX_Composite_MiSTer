# CXA1645 composite model: parameters

The model is based on the PlayStation PU-8 motherboard (NTSC) and its
Sony CXA1645 RGB encoder in NTSC mode (NPIN high, FO = 20 kohm). Source: Sony CXA1645P/M data sheet
(E93411A41-ST), typical values at its loaded measurement point (75 ohm
driver, 220 uF, 75 ohm source, 75 ohm termination). Constants are derived by
`tools/composite/model/cxa1645_model.py`; the RTL is `rtl/composite/`.

Kinds: **DS** data sheet typical, **STD** NTSC standard, **CIR** derived from
circuit values, **BRD** read from board photographs, **CORE** follows the
existing MiSTer core, **ASM** assumption.

## Source

| Parameter | Value | Kind | Notes |
| --- | --- | --- | --- |
| Sample clock | 53.693175 MHz, 15 per subcarrier cycle | CORE | `pll2` CLK_VIDEO; the GPU dot clock |
| Subcarrier | CLK_VIDEO / 15, free-running | ASM | Not traced on the board; makes the line 227.53 cycles (3413 clocks), so dot crawl follows the GPU's real line length |
| RGB full scale at RIN/GIN/BIN | code 255 = 1.0 V, linear | ASM | Sheet allows 1.0 Vp-p max. DAC transfer and settling not modelled |
| SYNC IN | H sync, inverted during V sync (`hs ^ vs`) | CORE | Same as the core's GunCon C-SYNC. Equalising pulses unverified |
| H sync width | 252 clocks, 4.69 us | CORE | `gpu_videoout_async.vhd` |

## Encoder

| Parameter | Value | Kind | Notes |
| --- | --- | --- | --- |
| White (Y/CV) | 0.71 V | DS | 0.60-0.82 |
| Sync | 0.29 V | DS | 0.26-0.33 |
| Y weights | 0.299 / 0.587 / 0.114 | STD | Sheet Y levels R 0.21, G 0.42, B 0.08 V agree |
| Burst | 0.25 Vp-p, 180 deg | DS | 0.20-0.30 |
| Chroma | standard ratios to burst | STD | R 3.16 / 103.5 deg, G 2.95 / 240.7, B 2.24 / 347.1; sheet typ 3.16 / 104, 2.95 / 241, 2.24 / 347 |
| Burst flag | 0.60 us after SYNC IN rise, 2.76 us | DS | tD 0.4-0.75, tW 2.5-3.2 |
| Half-H killer | ignore rising edges within 0.75 H | ASM | Threshold not published |
| Input clamp | level during burst flag is black (ICLP) | DS | Identity for PSX: the GPU outputs black through H blanking, which holds the burst flag. Not implemented |
| Modulators | ideal multipliers of the fundamental | ASM | Block diagram shows a sine-to-pulse stage; harmonics would be removed by the BPF |
| Chroma BPF | -3 dB at fsc +/- 1.3 MHz (Q 1.38) | ASM | Not published. Applied as a 1.305 MHz one-pole on U/V (narrow-band equivalent of a resonator) |
| Y delay | 7 samples (130 ns) | ASM | Sheet: delay line matches C. BPF group delay is 6.55 samples |
| Y / RGB bandwidth | flat | DS | Sheet: no worse than -3 dB at 5 MHz |
| YTRAP source | 1.5 kohm | DS | Pin 17 input resistance |
| YTRAP network | L507 33 uH + unmarked capacitor in series to ground | BRD | Capacitor taken as 59.9 pF, tuning the trap to fsc (notch Q 0.49); bilinear transform, Q12 |
| Board output stage | 1 kohm load on the chip output, emitter follower (Q504), 75 ohm series | BRD | Restores the sheet's loaded levels at a 75 ohm TV; follower response not modelled |
| Output coupling | 220 uF into 150 ohm, 4.8 Hz | CIR | Not modelled: the receiver clamps every line |

## Receiver

Kitrinx-derived decoder (MIT), PC Engine version, made DSP-lean for this core.
Contrast 2873 = 65536 * 255 / (0.71 * 8192). Luma boxcar 8, peaking span 9
(2.98 MHz), burst gate 1.3 us and clamp gate 4.0 us after sync. Brightness and
sharpness choices follow the SNES composite core.
