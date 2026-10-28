// SPDX-License-Identifier: GPL-2.0-or-later
// TurboBooster HES-AVB-01/02 PSG audio stage (HD Retrovision tracing,
// reference/board/hdretrovision-turbobooster.png). Per channel:
//   SNDL/SNDR -> C8/C10 1 uF -> R9/R12 2.7 kohm into the c358 virtual ground
//     high-pass: 1 / (2 pi * 1 uF * 2.7 kohm)  = 58.9 Hz
//   feedback R15/R16 12 kohm || C7/C12 1.5 nF
//     low-pass:  1 / (2 pi * 12 kohm * 1.5 nF) = 8.84 kHz
// The LC2/LC4 EMI filters (220 pF) and the 47 uF output coupling into the
// amplifier input are far outside the audio band and are not modelled; the
// MiSTer DC blocker follows. The stage's inverting gain of 12k/2.7k is
// normalised to the core's existing PSG level (49/64 ~ the DC gain of the
// replaced psg_filter IIR), so the mix balance with CD audio is unchanged.
// The PWD-623A base board has no filter between HuC6280 p17/p18 and the
// expansion port. Its uPC358 RF mixer is a 75 us FM pre-emphasis network
// that the TV de-emphasises, so it is not modelled either.
//
// ce must be clk / 32 (42.954545 MHz / 32 = 1.342 MHz); the shift-add
// coefficients below are derived for that rate:
//   HPF k = 2^-12 + 2^-15 + 2^-20         -> 58.9 Hz
//   LPF k = 2^-5 + 2^-7 + 2^-10 + 2^-11   -> 8.84 kHz
module pce_booster_audio (
    input  wire               clk,
    input  wire               reset,
    input  wire               ce,
    input  wire signed [15:0] in_l,
    input  wire signed [15:0] in_r,
    output wire signed [15:0] out_l,
    output wire signed [15:0] out_r
);
    pce_booster_audio_ch left  (.clk(clk), .reset(reset), .ce(ce), .din(in_l), .dout(out_l));
    pce_booster_audio_ch right (.clk(clk), .reset(reset), .ce(ce), .din(in_r), .dout(out_r));
endmodule

module pce_booster_audio_ch (
    input  wire               clk,
    input  wire               reset,
    input  wire               ce,
    input  wire signed [15:0] din,
    output reg  signed [15:0] dout
);
    // All state is Q20 in 40 bits: 16-bit input, 2x high-pass step overshoot
    // and guard bits leave ample headroom.
    wire signed [39:0] x49 = ($signed({{24{din[15]}}, din}) <<< 5)
                           + ($signed({{24{din[15]}}, din}) <<< 4)
                           +  $signed({{24{din[15]}}, din});

    reg  signed [39:0] xq, dc, hp, lp;

    // High-pass as input minus a tracked DC level (C8 charge).
    wire signed [39:0] dc_err = xq - dc;
    wire signed [39:0] dc_next = dc + (dc_err >>> 12) + (dc_err >>> 15) + (dc_err >>> 20);

    // One-pole low-pass (R15 || C7).
    wire signed [39:0] lp_err = hp - lp;
    wire signed [39:0] lp_next = lp + (lp_err >>> 5) + (lp_err >>> 7)
                                    + (lp_err >>> 10) + (lp_err >>> 11);

    wire signed [19:0] lp_int = lp[39:20];

    always @(posedge clk) begin
        if (reset) begin
            xq   <= 40'sd0;
            dc   <= 40'sd0;
            hp   <= 40'sd0;
            lp   <= 40'sd0;
            dout <= 16'sd0;
        end else if (ce) begin
            xq   <= x49 <<< 14; // x * 49/64 in Q20
            dc   <= dc_next;
            hp   <= dc_err;
            lp   <= lp_next;
            dout <= (lp_int > 20'sd32767)  ? 16'sh7fff :
                    (lp_int < -20'sd32768) ? 16'sh8000 : lp_int[15:0];
        end
    end
endmodule
