// SPDX-License-Identifier: MIT
// Signed product built from logic, not a DSP block. Quartus honours the
// multstyle attribute on a module; constant operands reduce to shift-adds.
(* multstyle = "logic" *)
module mul_logic #(
	parameter WA = 18,
	parameter WB = 18
)(
	input  signed [WA-1:0]    a,
	input  signed [WB-1:0]    b,
	output signed [WA+WB-1:0] p
);
assign p = a * b;
endmodule
