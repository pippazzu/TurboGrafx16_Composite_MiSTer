//============================================================================
// Decoded PC Engine composite pixels to the existing CLK_VIDEO domain.
// Copyright (C) 2026
//
// This program is free software; you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation; either version 2 of the License, or (at your option)
// any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details. You should have received a copy of the GNU General Public
// License along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//============================================================================

// Related PLL clocks: clk_dec = 42.954545 MHz, clk_vid = 85.909090 MHz.
// Each decoder pixel is held for two decoder clocks (four video clocks).
// Synchronize its event toggle, then capture the entire held bundle together.
// This is a bounded held-data crossing, not an arbitrary-rate async FIFO:
// pix_in must be a one-cycle strobe, at least two clk_dec cycles apart: the
// bundle is captured three video clocks after the toggle, one before the next.
module pce_cvbs_out (
	input            clk_dec,
	input            reset,
	input            pix_in,
	input            hs_in,
	input            vs_in,
	input            hb_in,
	input            vb_in,
	input      [7:0] r_in,
	input      [7:0] g_in,
	input      [7:0] b_in,
	input            clk_vid,
	output reg       ce_pix = 1'b0,
	output reg       hs = 1'b0,
	output reg       vs = 1'b0,
	output reg       hb = 1'b1,
	output reg       vb = 1'b1,
	output reg [7:0] r = 8'd0,
	output reg [7:0] g = 8'd0,
	output reg [7:0] b = 8'd0
);

reg        tog = 1'b0;
reg [27:0] hold = {4'b0011, 24'd0};
always @(posedge clk_dec) begin
	if (reset) begin
		tog <= 1'b0;
		hold <= {4'b0011, 24'd0};
	end else if (pix_in) begin
		hold <= {hs_in, vs_in, hb_in, vb_in, r_in, g_in, b_in};
		tog <= ~tog;
	end
end

reg tog_m = 1'b0, tog_m2 = 1'b0, tog_seen = 1'b0;
always @(posedge clk_vid) begin
	if (reset) begin
		tog_m <= 1'b0;
		tog_m2 <= 1'b0;
		tog_seen <= 1'b0;
		ce_pix <= 1'b0;
		{hs, vs, hb, vb, r, g, b} <= {4'b0011, 24'd0};
	end else begin
		tog_m <= tog;
		tog_m2 <= tog_m;
		tog_seen <= tog_m2;
		ce_pix <= tog_m2 != tog_seen;
		if (tog_m2 != tog_seen)
			{hs, vs, hb, vb, r, g, b} <= hold;
	end
end

endmodule
