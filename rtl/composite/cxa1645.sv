// SPDX-License-Identifier: GPL-2.0-or-later
//
// Sony CXA1645 RGB encoder as on the PU-8 board, NTSC mode (NPIN high, FO = 20 kohm), one sample
// per CLK_VIDEO (53.693175 MHz, 15 per subcarrier cycle).
//
// Levels are the data sheet's typical values at its loaded measurement point
// (75 ohm driver, 220 uF, 75 ohm source, 75 ohm termination), so the output is
// the voltage a TV sees:
//
//   R/G/B in   1.0 V = 100% (PSX DAC full scale: assumption); black in the
//              burst flag, so the input clamp changes nothing
//   White      0.71 V    Y = 0.299 R + 0.587 G + 0.114 B
//   Sync       0.29 V    added after the Y delay line while SYNC IN is low
//   Burst      0.25 Vp-p 180 deg, 0.6 us after the SYNC IN rising edge, 2.75 us
//   Chroma     standard NTSC ratios to burst: R 3.16 / 104 deg,
//              G 2.95 / 241 deg, B 2.24 / 347 deg, as the sheet's typicals
//
// The modulators are taken as ideal multipliers of the fundamental. The
// internal chroma BPF has no published response; it is modelled as -3 dB at
// fsc +/- 1.3 MHz (an assumption), which on the colour differences is a
// one-pole low-pass, and the internal Y delay line matches its group delay.
// YTRAP (pin 17) acts on the CVOUT luma only, through the pin's 1.5 kohm
// input resistance; the target PU-8 board fits a 33 uH +
// capacitor series trap there. model/cxa1645_model.py derives every
// constant.

module cxa1645 #(
	parameter ROM_FILE = "rtl/composite/cxa1645_rgb.hex"   // project-relative
)(
	input                    clk,
	input                    reset,

	input              [7:0] r,         // DAC codes at RIN / GIN / BIN
	input              [7:0] g,
	input              [7:0] b,
	input                    csync_n,   // SYNC IN, low in sync

	output reg signed [23:0] cvbs       // Q2.21 volts at the terminated jack
);

// Q2.16 volts.
localparam signed [19:0] SYNC_LVL   = 20'sd19005;   // 0.29 V
localparam signed [19:0] BURST_PEAK = 20'sd8192;    // 0.125 V

// Burst flag in samples from the SYNC IN rising edge.
localparam [7:0]  BF_DELAY = 8'd32;      // 0.60 us
localparam [7:0]  BF_WIDTH = 8'd148;     // 2.76 us
// Half-H killer: a rising edge within 0.75 H of the last is not a line.
localparam [11:0] HALF_H_KILL = 12'd2560;
// Y delay matching the chroma low-pass group delay (6.55 samples).
localparam integer Y_DELAY = 7;

// ------------------------------------------------------------ RGB matrix

// One ROM per input, {Y, U, V} contributions in Q2.16. U and V are the
// colour differences already weighted 0.493 / 0.877 and scaled to burst.
(* ramstyle = "M10K" *) reg [47:0] rom [0:767];
initial $readmemh(ROM_FILE, rom);

reg [47:0] cr, cg, cb;
always @(posedge clk) begin
	cr <= rom[{2'd0, r}];
	cg <= rom[10'd256 + {2'd0, g}];
	cb <= rom[10'd512 + {2'd0, b}];
end

wire signed [19:0] y0 = $signed(cr[47:32]) + $signed(cg[47:32]) + $signed(cb[47:32]);
wire signed [19:0] u0 = $signed(cr[31:16]) + $signed(cg[31:16]) + $signed(cb[31:16]);
wire signed [19:0] v0 = $signed(cr[15:0])  + $signed(cg[15:0])  + $signed(cb[15:0]);

// ------------------------------------------------------------ burst flag

// SYNC IN is registered twice, the second time beside the ROM output.
reg  sync_a, sync_b;
reg  [11:0] since_line = 12'hfff;   // power-up value: reset is never asserted
reg  [7:0]  bf_cnt;
reg         bf_run;

wire line_edge = sync_b & ~sync_a & (since_line >= HALF_H_KILL);
wire bf = bf_run && (bf_cnt >= BF_DELAY);

always @(posedge clk) begin
	if (reset) begin
		sync_a <= 1'b0;
		sync_b <= 1'b0;
		since_line <= 12'hfff;
		bf_cnt <= 8'd0;
		bf_run <= 1'b0;
	end else begin
		sync_a <= ~csync_n;
		sync_b <= sync_a;
		if (line_edge) since_line <= 12'd0;
		else if (~&since_line) since_line <= since_line + 12'd1;

		if (line_edge) begin
			bf_cnt <= 8'd0;
			bf_run <= 1'b1;
		end else if (bf_run) begin
			bf_cnt <= bf_cnt + 8'd1;
			if (bf_cnt == BF_DELAY + BF_WIDTH - 8'd1) bf_run <= 1'b0;
		end
	end
end

// ------------------------------------------------------------ input clamp

// The inputs are AC coupled and clamped while the burst flag is up (ICLP).
// The PSX GPU drives black through horizontal blanking, which contains the
// burst flag, so the clamp level is black and the clamp is the identity here.

reg signed [19:0] y1, u1, v1;
reg               sync1;

always @(posedge clk) begin
	y1    <= y0;
	u1    <= u0 - (bf ? BURST_PEAK : 20'sd0);
	v1    <= v0;
	sync1 <= sync_b;
end

// ------------------------------------------------------------ chroma BPF

// One-pole low-pass on U and V, alpha = 1/8 + 1/64 + 1/1024 (1.305 MHz),
// carried in Q2.20 so the shifts lose nothing that matters.
reg  signed [23:0] lu, lv;
wire signed [23:0] du = {u1, 4'd0} - lu;
wire signed [23:0] dv = {v1, 4'd0} - lv;

always @(posedge clk) begin
	if (reset) begin
		lu <= 24'sd0;
		lv <= 24'sd0;
	end else begin
		lu <= lu + (du >>> 3) + (du >>> 6) + (du >>> 10);
		lv <= lv + (dv >>> 3) + (dv >>> 6) + (dv >>> 10);
	end
end

// ------------------------------------------------------------ modulators

// 4096 sin and cos of 2 pi n / 15; the subcarrier is CLK_VIDEO / 15.
function automatic signed [13:0] sin15(input [3:0] n);
	case (n)
		4'd0 : sin15 =  14'sd0;     4'd1 : sin15 =  14'sd1666;  4'd2 : sin15 =  14'sd3044;
		4'd3 : sin15 =  14'sd3896;  4'd4 : sin15 =  14'sd4074;  4'd5 : sin15 =  14'sd3547;
		4'd6 : sin15 =  14'sd2408;  4'd7 : sin15 =  14'sd852;   4'd8 : sin15 = -14'sd852;
		4'd9 : sin15 = -14'sd2408;  4'd10: sin15 = -14'sd3547;  4'd11: sin15 = -14'sd4074;
		4'd12: sin15 = -14'sd3896;  4'd13: sin15 = -14'sd3044;  default: sin15 = -14'sd1666;
	endcase
endfunction

function automatic signed [13:0] cos15(input [3:0] n);
	case (n)
		4'd0 : cos15 =  14'sd4096;  4'd1 : cos15 =  14'sd3742;  4'd2 : cos15 =  14'sd2741;
		4'd3 : cos15 =  14'sd1266;  4'd4 : cos15 = -14'sd428;   4'd5 : cos15 = -14'sd2048;
		4'd6 : cos15 = -14'sd3314;  4'd7 : cos15 = -14'sd4006;  4'd8 : cos15 = -14'sd4006;
		4'd9 : cos15 = -14'sd3314;  4'd10: cos15 = -14'sd2048;  4'd11: cos15 = -14'sd428;
		4'd12: cos15 =  14'sd1266;  4'd13: cos15 =  14'sd2741;  default: cos15 = 14'sd3742;
	endcase
endfunction

reg  [3:0] ph;
reg signed [13:0] s2, c2;
reg signed [17:0] mu, mv;

always @(posedge clk) begin
	if (reset) ph <= 4'd0;
	else       ph <= (ph == 4'd14) ? 4'd0 : ph + 4'd1;
	s2 <= sin15(ph);
	c2 <= cos15(ph);
	mu <= 18'(lu >>> 4);
	mv <= 18'(lv >>> 4);
end

// C = U sin + V cos: one sum of two products.
/* verilator lint_off UNUSEDSIGNAL */
wire signed [32:0] c_m = mu * s2 + mv * c2;
/* verilator lint_on UNUSEDSIGNAL */
reg signed [19:0] c3;
always @(posedge clk) c3 <= 20'(c_m >>> 12);

// ------------------------------------------------------------ Y delay, sync

// Y waits Y_DELAY samples beyond the chroma pipeline (three registers);
// sync is added after the delay line, so it waits for the pipeline only.
reg signed [19:0] yd [0:Y_DELAY+2];
reg         [2:0] sd;
integer k;
always @(posedge clk) begin
	yd[0] <= y1;
	for (k = 1; k <= Y_DELAY + 2; k = k + 1) yd[k] <= yd[k-1];
	sd <= {sd[1:0], sync1};
end

reg signed [19:0] y3;      // YOUT luma with sync, beside c3
always @(posedge clk) y3 <= yd[Y_DELAY+1] - (sd[1] ? SYNC_LVL : 20'sd0);

// ------------------------------------------------------------ YTRAP

// CVOUT luma. PU-8: L507 33 uH in series with an unmarked capacitor taken
// as 59.9 pF, which tunes the trap to fsc (Q 0.49); bilinear transform
// prewarped at the notch, Q12, in Q2.18: y = b0 (x + x2) + a1 (x1 - y1) - a2 y2.
localparam signed [12:0] TB0 =  13'sd2903;
localparam signed [13:0] TA1 = -14'sd5304;
localparam signed [12:0] TA2 =  13'sd1710;

// The feed-forward terms and the y2 term are summed and registered outside
// the loop, so only y1's product closes it in one sample; the trap output arrives two
// samples after y3, and chroma waits to match.
wire signed [21:0] xl = {y3, 2'd0};
reg  signed [21:0] x1, x2, t1;
reg  signed [37:0] ff;   // feed-forward terms minus the y2 term
/* verilator lint_off UNUSEDSIGNAL */
wire signed [22:0] xs = xl + x2;
wire signed [35:0] lc_b0, lc_a1x, lc_a1t;
wire signed [34:0] lc_a2;
mul_logic #(23, 13) lc_mb0(xs, TB0, lc_b0);
mul_logic #(22, 14) lc_ma1x(x1, TA1, lc_a1x);
mul_logic #(22, 14) lc_ma1t(t1, TA1, lc_a1t);
mul_logic #(22, 13) lc_ma2(t1, TA2, lc_a2);
wire signed [37:0] lc_m = ff - lc_a1t;
/* verilator lint_on UNUSEDSIGNAL */
wire signed [21:0] t_lc = 22'(lc_m >>> 12);

always @(posedge clk) begin
	if (reset) begin
		{x1, x2, t1} <= '0;
		ff <= '0;
	end else begin
		x1 <= xl;
		x2 <= x1;
		ff <= lc_b0 + lc_a1x - lc_a2;
		t1 <= t_lc;
	end
end

// ------------------------------------------------------------ Y/C mix

wire signed [23:0] ycv = {t1, 2'd0};   // Q2.20

reg signed [19:0] c4, c5;
always @(posedge clk) {c5, c4} <= {c4, c3};

wire signed [24:0] mix = {ycv[23], ycv} + {c5[19], c5, 4'd0};   // Q2.20
always @(posedge clk) begin
	if      (mix >  25'sd4194303) cvbs <= 24'sd8388607;
	else if (mix < -25'sd4194304) cvbs <= -24'sd8388608;
	else                          cvbs <= {mix[22:0], 1'b0};
end

endmodule
