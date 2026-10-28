// SPDX-License-Identifier: GPL-2.0-or-later
// Normalised PWD-623A ideal resistor summer; Y DC gain is unity.
// Effective black-referenced Q2.21 source-voltage calibration, not literal
// HuC6260 pin DC voltages. DC normalisation folds unmeasured follower gain
// into source calibration; transistor bias/parasitics and load are not modelled.
// No inferred 7 MHz filter or fixed source impedance is imposed.
module pce_av_board (
    input  wire                     clk,
    input  wire                     reset,
    input  wire signed [23:0]       y_pin,
    input  wire signed [23:0]       ry_pin,
    input  wire signed [23:0]       by_pin,
    input  wire signed [23:0]       burst_pin,
    output reg  signed [23:0]       cvbs
);
    // Numeric packet supplies signed [17:0] Q16 localparams:
    // PCE_RY_GAIN, PCE_BY_GAIN, PCE_BURST_GAIN.
    `include "rtl/composite/huc6260_parameters.vh"

    wire signed [41:0] ry_product    = ry_pin    * PCE_RY_GAIN;
    wire signed [41:0] by_product    = by_pin    * PCE_BY_GAIN;
    wire signed [41:0] burst_product = burst_pin * PCE_BURST_GAIN;
    // Extend BEFORE shifting: preserve every bit of the unity-Y Q16 term.
    wire signed [41:0] y_scaled = $signed({{18{y_pin[23]}}, y_pin}) <<< 16;

    // Two guard bits retain the full sum even for arbitrary signed 18-bit gains.
    wire signed [43:0] sum_q16 =
        $signed({{2{y_scaled[41]}}, y_scaled}) +
        $signed({{2{ry_product[41]}}, ry_product}) +
        $signed({{2{by_product[41]}}, by_product}) +
        $signed({{2{burst_product[41]}}, burst_product});

    // Round ONCE after summing, nearest with exact half ties away from zero.
    // Arithmetic shift floors negatives, so negative bias is half minus one.
    wire signed [43:0] biased_sum = sum_q16 +
        (sum_q16[43] ? 44'sd32767 : 44'sd32768);
    wire signed [43:0] rounded_sum = biased_sum >>> 16;

    // One output pipeline stage, synchronous active-high reset, no clock enable.
    always @(posedge clk) begin
        if (reset)
            cvbs <= 24'sd0;
        else if (rounded_sum > 44'sd8388607)
            cvbs <= 24'sh7fffff;
        else if (rounded_sum < -44'sd8388608)
            cvbs <= 24'sh800000;
        else
            cvbs <= rounded_sum[23:0];
    end
endmodule
