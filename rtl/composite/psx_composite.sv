// SPDX-License-Identifier: GPL-2.0-or-later
//
// PlayStation NTSC composite, aimed at the NTSC PU-8 board: GPU RGB
// and sync -> CXA1645 encoder with the PU-8 Y trap -> TV
// decoder, all on CLK_VIDEO (53.693175 MHz, 15 samples per subcarrier).
//
// SYNC IN is horizontal sync, inverted during vertical sync, the composite
// sync the core already derives for the GunCon. Whether the console's own
// C-SYNC carries equalising pulses is unverified.
//
// Picture geometry stays the core's: its blanking, crop and field timing
// are delayed to sit on the decoded picture, which is presented at
// CLK_VIDEO / 4 (13.4 MHz) so detail at the subcarrier survives. The
// decoder's own sync recovery serves only its burst and clamp gates.

module psx_composite (
	input              clk,
	input              reset,

	input        [7:0] r,           // GPU DAC codes, black in native blanking
	input        [7:0] g,
	input        [7:0] b,
	input              hs,          // GPU sync, high in sync
	input              vs,

	input              premium,     // adaptive comb instead of a notch
	input signed [15:0] brightness,
	input        [3:0] sharpness,

	input              t_hs,        // presentation timing, as the RGB path
	input              t_vs,
	input              t_hb,
	input              t_vb,
	input              t_field,

	output             pix,
	output             hs_out,
	output             vs_out,
	output             hb_out,
	output             vb_out,
	output             field_out,
	output       [7:0] r_out,
	output       [7:0] g_out,
	output       [7:0] b_out
);

reg [7:0] ri, gi, bi;
reg       csync_n;
always @(posedge clk) begin
	{ri, gi, bi} <= {r, g, b};
	csync_n <= ~(hs ^ vs);
end

wire signed [23:0] cvbs;
cxa1645 encoder (
	.clk(clk), .reset(reset),
	.r(ri), .g(gi), .b(bi), .csync_n(csync_n),
	.cvbs(cvbs)
);

// Receiver: a consumer set's notch, or an adaptive comb. Contrast is the
// encoder's 0.71 V white; black is taken at blanking (no NTSC-M setup).
composite_decoder #(
	.SPC(15), .HCNT_W(12), .LUMA_LP(8), .PK_SPAN(9), .PIX_DIV(4),
	.BURST_GATE_TENTHS(13), .CLAMP_GATE_TENTHS(40), .CONTRAST(2873)
) decoder (
	.clk(clk), .reset(reset), .ce(1'b1),
	.frame_lines(9'd263), .active_start(9'd0), .active_end(9'd263),
	.comp(cvbs), .sat(8'd128), .hue(8'd0),
	.sharpness(sharpness), .black_stretch(2'd0), .brightness(brightness),
	.comb_mode(premium ? 2'd2 : 2'd0),
	.ce_out(), .pix_out(), .hs_out(), .vs_out(), .hb_out(), .vb_out(),
	.r_out(r_out), .g_out(g_out), .b_out(b_out)
);

// Source RGB to decoded RGB, at the centre of a luma step, is LATENCY
// samples (48.2 measured with tools/composite/sim from the encoder input, plus the
// input register); the timing inputs trail the RGB by one sample.
localparam integer LATENCY = 49;
localparam integer PRESENT_DELAY = LATENCY - 1;

reg [4:0] timing [0:PRESENT_DELAY-1];
integer i;
always @(posedge clk) begin
	timing[0] <= {t_hs, t_vs, t_hb, t_vb, t_field};
	for (i = 1; i < PRESENT_DELAY; i = i + 1) timing[i] <= timing[i-1];
end
assign {hs_out, vs_out, hb_out, vb_out, field_out} = timing[PRESENT_DELAY-1];

// One pixel in four, locked to the line.
reg [1:0] pcnt;
reg       hs_q;
always @(posedge clk) begin
	hs_q <= hs_out;
	pcnt <= (hs_out & ~hs_q) ? 2'd0 : pcnt + 2'd1;
end
assign pix = (pcnt == 2'd3);

endmodule
