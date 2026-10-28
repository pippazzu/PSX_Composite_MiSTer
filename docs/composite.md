# Composite video: technical notes

## PlayStation composite output (CXA1645)

Sources: the Sony CXA1645P/M data sheet (E93411A41-ST), photographs of two PU-8 motherboards,
an NTSC one (1-658-467-23, 53.69 MHz video crystal) and a PAL SCPH-1002 (1-658-467-22, 53.20 MHz),
both with encoder IC501 CXA1645M, the NTSC standard, and the existing MiSTer core for the GPU's video
timing. No service schematic was used, and no console has been measured with an oscilloscope, so
nothing below is checked against a real console's output. Every constant, its source and the
assumptions are listed in [composite/PARAMETERS.md](composite/PARAMETERS.md).

1. **The GPU** drives analog R, G and B and a sync. The core's GPU line is 3413 cycles of the
   53.693175 MHz video clock with a 4.69 µs horizontal sync (`gpu_videoout_async.vhd`). Taking the
   subcarrier as that clock / 15, a line is 227.53 subcarrier cycles, close to NTSC's 227.5, so the
   colour phase moves from line to line and fine detail crawls. The DAC's full scale at the encoder
   is an estimate: code 255 = 1.0 V, linear (the data sheet allows 1.0 Vp-p at its inputs; the
   DAC's real level and settling are not documented). The encoder's SYNC IN is taken as horizontal
   sync inverted during vertical sync; whether the console adds equalising pulses is not known.
2. **The CXA1645 encoder** turns RGB into luma and chroma (data sheet typical values at its loaded
   measurement point): Y = 0.299 R + 0.587 G + 0.114 B, white 0.71 V, sync 0.29 V, no setup
   pedestal, a 0.25 Vp-p burst at 180°, starting 0.6 µs after the rising edge of SYNC IN for
   2.76 µs, and a half-H killer. The chroma ratios to burst follow the NTSC standard; they match
   the sheet's typical R 3.16 / 104°, G 2.95 / 241°, B 2.24 / 347°. The chip's chroma band-pass
   filter has no published response: it is an estimate, −3 dB at fsc ± 1.3 MHz, with the internal
   luma delay line matched to its group delay. The half-H killer's threshold is not published
   either; it is taken as 0.75 H.
3. **The board** (read from the photographs, the same on both): YTRAP (pin 17, 1.5 kΩ input resistance per the data
   sheet) goes to ground through L507, 33 µH on both boards, in series with an unmarked capacitor. The
   capacitor's value is an estimate, 59.9 pF, which tunes the trap to the NTSC subcarrier (notch
   Q 0.49); the inductor is the only marked part. It notches the luma of the composite output only. The output
   stage is a 1 kΩ load on the chip's output, an emitter follower (Q504) and 75 Ω in series, which
   gives the data sheet's loaded levels at a 75 Ω TV. The output coupling is 220 µF into 150 Ω
   (4.8 Hz), which a TV's clamp removes.

Not known: the GPU DAC's real levels and settling, the encoder's chroma filter response, the trap
capacitor's value, and the transistor buffer's frequency response.

## How it is implemented

All of it runs on the core's video clock, 53.693175 MHz, 15 samples per subcarrier cycle.

1. **GPU input**: the GPU's 8-bit RGB, forced to black through horizontal and vertical blanking,
   and the sync as above, registered once.
2. **Encoder**: one ROM per input channel gives its {Y, U, V} contribution in Q2.16 volts (U and V
   already weighted 0.493 / 0.877 and scaled to the burst), summed per sample. The chroma filter is
   applied as a one-pole low-pass at 1.305 MHz on U and V (the narrow-band equivalent of the
   band-pass), luma is delayed 7 samples to match its 6.55-sample group delay, and the burst flag,
   half-H killer and sync level follow the data sheet values above. The input clamp is not
   implemented: the GPU outputs black during the burst flag, so it would change nothing.
3. **Board**: the Y trap is a bilinear-transform filter with Q12 coefficients. The output is a
   signed Q2.21 voltage at the terminated jack. `tools/composite/model/cxa1645_model.py` derives
   every constant and writes the ROM.
4. **Composite filters**: Kitrinx's composite decoder, run at 15 samples per subcarrier cycle, with
   the changes listed below. The core sets it to a luma boxcar of 8 samples, a peaking span of 9
   (2.98 MHz), a burst gate 1.3 µs and a black clamp 4.0 µs after the end of sync, and a contrast
   of 2873 (65536 × 255 / (0.71 × 8192)), what a set's sync AGC gives for the encoder's 0.71 V
   white. Black is taken at blanking. Comp. Brightness feeds the decoder's brightness input: USA
   lowers black by 0.054 V (7.5 IRE), the setup a US set expects; More, Much More and Too Much
   raise it by 0.036, 0.078 and 0.114 V. Comp. Sharpness sets the peaking to 0, 4, 8 or 12 of 15.
5. **Picture timing**: the core's own blanking, sync and field signals are delayed 48 samples to
   sit on the decoded picture, which is presented at the video clock / 4 (13.4 MHz) so detail at
   the subcarrier survives the scaler. Encoder to decoded RGB is 49 samples, 0.91 µs. The
   decoder's own sync recovery serves only its burst and clamp gates.
6. **Selection**: composite replaces the RGB picture only for NTSC video at the normal video clock.
   PAL video, the debug framebuffer and fast forward (which raises the video clock) show the RGB
   picture. The composite chain is never reset: it runs from power-up, so its delay lines stay in
   block RAM, and every register whose reset value is not zero has that value at power-up. In
   `PSX.sdc` its paths are held to the 53.69 MHz period (18.5 ns), since timing analysis otherwise
   uses the fast-forward clock.

Not implemented: PAL, interlaced sync (equalising pulses), the GPU DAC's settling, the output
buffer's response.

In simulation (Icarus Verilog, `tools/composite/sim`), the encoder and decoder run on PSX timing
(3413-clock lines, 263-line frames) with a colour-bar and fine-detail pattern, and the decoder's
output is compared sample by sample with the reference decoder `composite_decoder_ref.sv`, the
same decoder without the PlayStation changes. With the notch filter, 99.6% of
active-picture samples are within 2 of 255 of the reference, and the largest differences fall
outside the picture. The same run started from a zeroed line period (`UPSET=1`, what power-up
without reset gives) locks and gives the same result.

## Changes to Kitrinx's composite decoder

The decoder is Kitrinx's from the NES core (MIT licence; the changes are MIT too), in
`rtl/composite/composite_decoder.sv`, licence in `rtl/composite/LICENSE.decoder`. It comes from the
same line as the Mega Drive core's decoder, with further changes for the PlayStation.

* **Burst gate and black clamp** are timed from the trailing edge of sync, as a set's burst gate
  pulse is, instead of from the leading edge. The gate positions are parameters, set in tenths of a
  microsecond. With the PlayStation's 4.69 µs sync the burst gate still falls 6.0 µs after the
  leading edge.
* **Chroma is blanked** until the black clamp position, as a set blanks it through horizontal
  blanking, so the burst is never shown.
* **Premium TV comb** uses a set's fixed 227.5-cycle delay instead of one line of the source. A
  PlayStation line is 227.53 cycles, so a delay of one source line would meet chroma 12° away from
  inverted and leave colour in the luma.
* **Sync separator** slices the signal averaged over one subcarrier cycle (15 samples), as a set's
  does through its low-pass, instead of the raw signal, so saturated colours whose chroma dips
  toward the sync tip are not taken as line starts. The divide by 15 is a shift-add (1/16 + 1/256,
  0.4% low), because logic is scarce. The average's delay is not taken back out: the burst and
  clamp gates move with it, and the picture's position comes from the core's own timing, so
  nothing depends on it.
* **Line lock**: the line period is measured from sync edge to sync edge and accepted within one
  subcarrier cycle of 227.5 cycles; between edges, and through vertical sync, a flywheel keeps the
  line going. An out-of-range period (power-up, an upset) is replaced by the next normal one
  instead of locking acquisition out, because this core never resets the decoder.
* **Vertical timing**: vertical sync is recognised when sync is still low 8.0 µs after its leading
  edge, and the presented rows are fixed by inputs (frame length, first and last row) instead of
  being derived from the signal, so the extra line of a 263-line frame is blanking only.
* **Power-up values**: every register whose reset value is not zero is declared with it, because
  the core never asserts reset.
* **DSP use**: the core has almost no DSP blocks to spare. Constant and narrow products are built
  from logic (`mul_logic.sv`). The per-line burst arithmetic (hue rotation, burst magnitude, the
  1/|burst|² divide, saturation, contrast and the colour matrix) runs on one serial multiplier; it
  is measured on one line and applied from the next, so a steady burst gives the coefficients the
  original computes on the same line. Contrast is a parameter folded into the colour coefficients
  (it rounds once where the original rounds three times), and G − Y is a fixed mix of R − Y and
  B − Y. Full-rate DSP use is the demodulator and the R − Y and B − Y sums.
* **Chroma trail removed**: the one-pole chroma tail and its input are gone; it was off.
* **Samples per cycle**: the core runs the decoder at 15 samples per subcarrier cycle (odd). The
  notch's half-cycle delay is then the mean of the two samples around it, and the comb's
  half-sample delay likewise. The other boxcar divides by 15 are shift-adds as above (0.003% for the
  demodulated chroma).
* **Timing registers**: extra register stages in the notch/comb path, the comb blend, the peaking
  difference and the G mix, for timing at this core's fill. Each keeps the original's alignment of
  samples; the G mix stage adds one sample of latency.
* Not changing the picture: the sharpness filter's tap spacing is a parameter, two products are
  folded into one each (same results), and the comb's line store is a RAM inside the decoder rather
  than the core's dpram.

With a 4.7 µs sync and standard NTSC timing, the decoder gives the same picture as the original.

## Build

Quartus 17.0.2 Build 602 Lite, 5CSEBA6U23I7, seed 6. Logic 41,059 / 41,910 ALMs (98%), DSP blocks
110/112, RAM blocks 392/553 (71%). Worst setup slack is +0.029 ns and hold slack +0.127 ns, so
timing passes.
