// SPDX-License-Identifier: GPL-2.0-or-later
// PC Engine source -> PWD-623A board approximation -> TV receiver.
// All three stages use clk_sys = 42.954545 MHz (12 samples/carrier).
// Phase and gate profiles are provisional; this is not a full netlist model.
module pce_composite (
    input              clk,
    input              reset,
    input        [8:0] color,
    input              bw,
    input       [11:0] hcount,
    input        [9:0] vcount,
    input        [8:0] frame_lines,
    input              hs,
    input              vs,
    input              hbl,
    input              vbl,
    input              premium,
    input              overscan,
    input       [15:0] brightness,
    input        [1:0] sharpness,
    output             pix,
    output             hs_out,
    output             vs_out,
    output             hb_out,
    output             vb_out,
    output       [7:0] r_out,
    output       [7:0] g_out,
    output       [7:0] b_out
);

    wire signed [23:0] y_pin, ry_pin, by_pin, burst_pin;
    wire signed [23:0] cvbs;

    huc6260_composite source (
        .clk(clk), .reset(reset),
        .color(color), .bw(bw),
        .hcount(hcount), .vcount(vcount), .frame_lines(frame_lines),
        .hs(hs), .vs(vs), .hbl(hbl), .vbl(vbl),
        .y_pin(y_pin), .ry_pin(ry_pin), .by_pin(by_pin),
        .burst_pin(burst_pin)
    );

    pce_av_board board (
        .clk(clk), .reset(reset),
        .y_pin(y_pin), .ry_pin(ry_pin), .by_pin(by_pin),
        .burst_pin(burst_pin), .cvbs(cvbs)
    );

    // Contract candidate receiver profile: gates are tenths of a microsecond
    // after the recovered sync trailing edge; geometry is samples / lines.
    // These receiver choices and source phase/gates require hardware comparison.
    composite_decoder #(
        .SPC(12), .HCNT_W(12), .PIX_DIV(4),
        .LUMA_LP(6), .PK_SPAN(7),
        .BURST_GATE_TENTHS(13), .CLAMP_GATE_TENTHS(38),
        .HS_WIDTH(192), .HB_END(390), .FRONT_PORCH(73)
    ) receiver (
        .clk(clk), .reset(reset), .ce(1'b1),
        // Current 262/263-line metadata is held stable for the frame by VCE.
        // It needs no sample-by-sample delay through the source/board pipeline.
        .frame_lines(frame_lines), .comp(cvbs),
        // Match the VCE active rows in both frame lengths; end is exclusive.
        .active_start(overscan ? 9'd19 : 9'd23),
        .active_end(overscan ? 9'd261 : 9'd254),
        .sat(8'd128), .contrast(16'd2857), .brightness(brightness),
        .hue(8'd0), .chroma_trail(4'd0), .sharpness({sharpness, 2'b00}),
        .black_stretch(2'd0), .comb_mode(premium ? 2'd2 : 2'd0),
        .ce_out(), .pix_out(pix),
        .hs_out(hs_out), .vs_out(vs_out),
        .hb_out(hb_out), .vb_out(vb_out),
        .r_out(r_out), .g_out(g_out), .b_out(b_out)
    );

    // RGB and timing are the receiver's matched output tuple. The existing
    // top-level bridge captures it on pix and transfers it to 85.909 MHz.
    // No wrapper clock generation, output RAM, or additional output pipeline.
endmodule
