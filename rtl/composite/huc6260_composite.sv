// SPDX-License-Identifier: GPL-2.0-or-later
// HuC6260 source-pin approximation for the original PWD-623A board model.
// Hardware-informed physical signal chain, NOT a netlist-derived waveform.
// ROM provenance: Furrtek test-mode readout / Kitrinx transcription (note 01).
// Q2.21 pins are effective black-referenced quantities, not physical pin DC.
// Double-zero U/V, sinusoidal carriers, DUO-R colour fit, reset phase, BW and
// fixed burst gate are calibration/timing assumptions; see notes 14 and 15.
module huc6260_composite (
    input  wire               clk,
    input  wire               reset,
    input  wire [8:0]         color,
    input  wire               bw,
    input  wire [11:0]        hcount,
    input  wire [9:0]         vcount,
    input  wire [8:0]         frame_lines,
    input  wire               hs,
    input  wire               vs,
    input  wire               hbl,
    input  wire               vbl,
    output reg signed [23:0]  y_pin,
    output reg signed [23:0]  ry_pin,
    output reg signed [23:0]  by_pin,
    output reg signed [23:0]  burst_pin
);
    // Compile from the core directory; declarations/functions are module-local.
    `include "rtl/composite/huc6260_parameters.vh"

    // The generated HEX is the sole initialization source for both Icarus and
    // Quartus 17. No vendor primitive or alternate working-directory path.
    reg [14:0] yuv_rom [0:511];
    initial begin
        $readmemh("rtl/composite/huc6260_yuv.hex", yuv_rom);
    end

    reg [14:0] yuv_q;
    reg [3:0] phase;
    reg [3:0] phase_q;
    reg active_q, bw_q, sync_active_q, burst_q, valid_q;

    // Keep the ROM read free of reset logic for synchronous ROM inference.
    always @(posedge clk) begin
        yuv_q <= yuv_rom[color];
    end

    always @(posedge clk) begin
        if (reset) begin
            phase    <= 4'd0;
            phase_q  <= 4'd0;
            active_q <= 1'b0;
            bw_q     <= 1'b0;
            sync_active_q   <= 1'b0;
            burst_q  <= 1'b0;
            valid_q  <= 1'b0;
        end else begin
            // No line, frame, blanking or mode event restarts the carrier.
            phase    <= (phase == 4'd11) ? 4'd0 : phase + 4'd1;
            phase_q  <= phase;
            active_q <= !hbl && !vbl;
            bw_q     <= bw;
            sync_active_q   <= hs ^ vs; // active during negative CSYNC, inverted HS in VS
            burst_q  <= (hcount >= 12'd260) && (hcount < 12'd380) && !vs;
            valid_q  <= 1'b1;
        end
    end

    // Source-calibration assumption, NOT a decap-established DAC transfer:
    // unsigned codes 0..15 map to -15..0; 16..31 map to 0..15.
    function automatic signed [5:0] chroma_double_zero(input [4:0] code);
        begin
            if (code <= 5'd15)
                chroma_double_zero = $signed({1'b0, code}) - 6'sd15;
            else
                chroma_double_zero = $signed({1'b0, code}) - 6'sd16;
        end
    endfunction

    // Explicitly signed operands and full product widths. Each step/peak is
    // Q21; the three generated signed 14-bit carrier functions are Q12.
    wire signed [23:0] y_step = PCE_Y_STEP;
    wire signed [23:0] u_step = PCE_U_PIN_STEP;
    wire signed [23:0] v_step = PCE_V_PIN_STEP;
    wire signed [23:0] burst_peak = PCE_BURST_PIN_PEAK;
    wire signed [23:0] sync_level = PCE_SYNC_LEVEL;
    wire signed [5:0] y_code = $signed({1'b0, yuv_q[4:0]});
    wire signed [5:0] u_code = chroma_double_zero(yuv_q[14:10]);
    wire signed [5:0] v_code = chroma_double_zero(yuv_q[9:5]);
    wire signed [13:0] picture_sin = PCE_SIN_PICTURE(phase_q);
    wire signed [13:0] picture_cos = PCE_COS_PICTURE(phase_q);
    wire signed [13:0] burst_sin = PCE_SIN_BURST(phase_q);
    wire signed [29:0] y_level = y_code * y_step;
    wire signed [29:0] u_level = u_code * u_step;
    wire signed [29:0] v_level = v_code * v_step;
    wire signed [43:0] u_product = u_level * picture_sin;
    wire signed [43:0] v_product = v_level * picture_cos;
    wire signed [37:0] burst_product = burst_peak * burst_sin;
    wire signed [43:0] u_scaled = u_product >>> 12;
    wire signed [43:0] v_scaled = v_product >>> 12;
    // Negate at full width BEFORE rescaling: burst is -sin(phase*30deg).
    wire signed [37:0] burst_negative = -burst_product;
    wire signed [37:0] burst_scaled = burst_negative >>> 12;

    // Stage 2: all four pins register the same ROM/control/phase sample.
    // Y sync has priority; burst is independent of BW and picture blanking.
    always @(posedge clk) begin
        if (reset || !valid_q) begin
            y_pin     <= 24'sd0;
            ry_pin    <= 24'sd0;
            by_pin    <= 24'sd0;
            burst_pin <= 24'sd0;
        end else begin
            if (sync_active_q)
                y_pin <= sync_level;
            else if (active_q)
                y_pin <= y_level[23:0];
            else
                y_pin <= 24'sd0;

            if (active_q && !bw_q) begin
                ry_pin <= v_scaled[23:0];
                by_pin <= u_scaled[23:0];
            end else begin
                ry_pin <= 24'sd0;
                by_pin <= 24'sd0;
            end
            burst_pin <= burst_q ? burst_scaled[23:0] : 24'sd0;
        end
    end
    // vcount/frame_lines are API inputs for integration; neither modifies
    // this source's continuous phase or the explicitly specified gates.
endmodule
