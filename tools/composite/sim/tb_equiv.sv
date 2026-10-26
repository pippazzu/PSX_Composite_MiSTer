// Encoder + both decoders on PSX-like NTSC timing.
//  - compares the PSX decoder against the reference (PCE) decoder sample by sample
//  - writes the PSX decoder's frame as a PPM for a visual check
//  - logs one composite line for level checks
`timescale 1ns/1ps
module tb;
parameter integer SHARP = 0;
parameter integer COMB  = 0;
parameter integer YTRAP = 0;
parameter integer FRAMES = 4;
parameter integer PATTERN = 0;     // 0 bars + detail, 1 flat grey ramp
parameter integer UPSET = 0;      // 1: decoder line period starts at 0, as on power-up
parameter integer INTERLACE = 0;   // 1: fields of 263 / 262 lines, half-line V sync on one

localparam integer HTOT = 3413, HS_W = 252;
wire [9:0] LINES = (INTERLACE != 0 && frame[0]) ? 10'd262 : 10'd263;
localparam integer X0 = 608, DIV = 8, XW = 320;

reg clk = 0, reset = 1;
always #9.312 clk = ~clk;

integer hc = 0, vc = 0, frame = 0;
reg [7:0] r, g, b;
reg hs, vs;
integer seed = 1;

// hsync starts 32 clocks before the line wraps, as gpu_videoout_async
always @(posedge clk) begin
	hc <= (hc == HTOT - 1) ? 0 : hc + 1;
	if (hc == HTOT - 1) begin
		vc <= (vc == LINES - 1) ? 0 : vc + 1;
		if (vc == LINES - 1) frame <= frame + 1;
	end
end

function [23:0] bar(input integer x);
	case ((x * 8) / XW)
		0: bar = 24'hFFFFFF; 1: bar = 24'hFFFF00; 2: bar = 24'h00FFFF; 3: bar = 24'h00FF00;
		4: bar = 24'hFF00FF; 5: bar = 24'hFF0000; 6: bar = 24'h0000FF; default: bar = 24'h000000;
	endcase
endfunction

integer px, ln;
reg [23:0] pix;
always @(posedge clk) begin
	hs <= (hc >= HTOT - 32) || (hc < HS_W - 32);
	if (INTERLACE && frame[0])
		vs <= (vc > 243 || (vc == 243 && hc >= 1675)) && (vc < 246 || (vc == 246 && hc < 1675));
	else
		vs <= (vc >= 242 && vc < 245);
	px = (hc - X0) / DIV;
	ln = vc - 16;
	if (hc >= X0 && hc < X0 + XW * DIV && vc >= 16 && vc < 256) begin
		if (PATTERN == 1) pix = {3{8'(px * 255 / (XW - 1))}};
		else if (ln < 120) pix = bar(px);
		else if (ln < 160) pix = {3{8'(px * 255 / (XW - 1))}};
		else if (ln < 200) pix = (px & 1) ? 24'hFFFFFF : 24'h000000;
		else pix = {8'(px * 7 + ln), 8'(px * 3 + ln * 5), 8'(ln * 9 - px)};
	end else pix = 24'h000000;
	{r, g, b} <= pix;
end

wire csync_n = ~(hs ^ vs);

wire signed [23:0] cvbs;
cxa1645 #(.ROM_FILE("../../../../rtl/composite/cxa1645_rgb.hex")) enc (
	.clk(clk), .reset(reset), .r(r), .g(g), .b(b), .csync_n(csync_n),
	.cvbs(cvbs));

wire [7:0] r0w, g0w, b0w, r1, g1, b1;
reg  [7:0] r0, g0, b0, r0d, g0d, b0d;   // reference, delayed by the PSX decoder's two extra stages
always @(posedge clk) begin {r0d, g0d, b0d} <= {r0w, g0w, b0w}; {r0, g0, b0} <= {r0d, g0d, b0d}; end
composite_decoder_ref #(
	.SPC(15), .HCNT_W(12), .LUMA_LP(8), .PK_SPAN(9), .PIX_DIV(4),
	.BURST_GATE_TENTHS(13), .CLAMP_GATE_TENTHS(40)
) ref_dec (
	.clk(clk), .reset(reset), .ce(1'b1),
	.frame_lines(9'd263), .active_start(9'd0), .active_end(9'd263),
	.comp(cvbs), .sat(8'd128), .hue(8'd0), .chroma_trail(4'd0),
	.sharpness(4'(SHARP)), .black_stretch(2'd0), .brightness(16'sd0),
	.contrast(16'd2873), .comb_mode(2'(COMB)),
	.ce_out(), .pix_out(), .hs_out(), .vs_out(), .hb_out(), .vb_out(),
	.r_out(r0w), .g_out(g0w), .b_out(b0w));

composite_decoder #(
	.SPC(15), .HCNT_W(12), .LUMA_LP(8), .PK_SPAN(9), .PIX_DIV(4),
	.BURST_GATE_TENTHS(13), .CLAMP_GATE_TENTHS(40), .CONTRAST(2873)
) psx_dec (
	.clk(clk), .reset(reset), .ce(1'b1),
	.frame_lines(9'd263), .active_start(9'd0), .active_end(9'd263),
	.comp(cvbs), .sat(8'd128), .hue(8'd0),
	.sharpness(4'(SHARP)), .black_stretch(2'd0), .brightness(16'sd0),
	.comb_mode(2'(COMB)),
	.ce_out(), .pix_out(), .hs_out(), .vs_out(), .hb_out(), .vb_out(),
	.r_out(r1), .g_out(g1), .b_out(b1));

integer hist [0:3];
integer maxd = 0, n = 0, d, i, f, cl;
integer worst_hc = 0, worst_vc = 0;
integer ahist [0:3];
integer lmax = 0, lm, cl2, sat0 = 0, sat1 = 0, sl;
function integer absd(input [7:0] a, input [7:0] c);
	absd = (a > c) ? a - c : c - a;
endfunction

initial begin
	for (i = 0; i < 4; i = i + 1) begin hist[i] = 0; ahist[i] = 0; end
	lm = $fopen("linemax.txt", "w");
	repeat (20) @(posedge clk);
	reset = 0;
	if (UPSET) begin               // power-up state of the never-reset core
		force psx_dec.line_period = 12'd0;
		repeat (2) @(posedge clk);
		release psx_dec.line_period;
	end
	f = $fopen("frame.ppm", "w");
	$fwrite(f, "P3\n%0d %0d\n255\n", HTOT / 4, LINES);
	cl = $fopen("line.txt", "w");
	cl2 = $fopen("line190.txt", "w");
	sl = $fopen("linesat.txt", "w");
	wait (frame == FRAMES);
	$fclose(f);
	$fclose(cl);
	$display("SHARP=%0d COMB=%0d YTRAP=%0d samples=%0d max|diff|=%0d at line %0d sample %0d",
	         SHARP, COMB, YTRAP, n, maxd, worst_vc, worst_hc);
	$display("  diff 0:%0d 1:%0d 2:%0d >2:%0d", hist[0], hist[1], hist[2], hist[3]);
	$display("  active lines 20-239: 0:%0d 1:%0d 2:%0d >2:%0d", ahist[0], ahist[1], ahist[2], ahist[3]);
	$fclose(lm);
	$finish;
end

always @(posedge clk) if (!reset && frame >= 2) begin
	d = absd(r0, r1);
	if (absd(g0, g1) > d) d = absd(g0, g1);
	if (absd(b0, b1) > d) d = absd(b0, b1);
	hist[(d > 3) ? 3 : d] = hist[(d > 3) ? 3 : d] + 1;
	n = n + 1;
	if (d > maxd) begin maxd = d; worst_hc = hc; worst_vc = vc; end
	if (frame == FRAMES - 1 && hc % 4 == 0 && hc < 3412) begin
		$fwrite(f, "%0d %0d %0d\n", r1, g1, b1);
	end
	if (hc >= 700 && hc < 3300 && d > lmax) lmax = d;
	if (hc >= 700 && hc < 3300 && vc >= 20 && vc < 240) begin
		ahist[(d > 3) ? 3 : d] = ahist[(d > 3) ? 3 : d] + 1;
	end
	if (hc >= 900 && hc < 3000) begin
		if (absd(r0, g0) + absd(g0, b0) > sat0) sat0 = absd(r0, g0) + absd(g0, b0);
		if (absd(r1, g1) + absd(g1, b1) > sat1) sat1 = absd(r1, g1) + absd(g1, b1);
	end
	if (hc == HTOT - 1) begin $fwrite(lm, "%0d %0d\n", vc, lmax); lmax = 0; $fwrite(sl, "%0d %0d %0d %0d\n", frame, vc, sat0, sat1); sat0 = 0; sat1 = 0; end
	if (frame == FRAMES - 1 && vc == 190) $fwrite(cl2, "%0d %0d\n", hc, cvbs);
	if (frame == FRAMES - 1 && vc == 60) $fwrite(cl, "%0d %0d %0d %0d %0d %0d %0d %0d\n", hc, cvbs, r0, g0, b0, r1, g1, b1);
end
endmodule
