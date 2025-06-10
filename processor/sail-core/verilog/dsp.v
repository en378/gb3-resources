/*
 * dsp.v
 *
 * This module is a wrapper for the iCE40 SB_MAC16 DSP primitive,
 * configured as a dedicated 16x16 -> 32-bit signed multiplier.
 * It is combinational from the CPU's perspective.
 */
module dsp(
    input           clk,
    input           ce,
    input  [15:0]   A,
    input  [15:0]   B,
    output [31:0]   O
);

    // Instantiate the dedicated DSP primitive
    SB_MAC16 dsp_inst (
        .CLK(clk), .CE(ce),
        .A(A), .B(B),
        .O(O),

        // --- Tie off all unused ports to their default safe values ---
        .C(16'b0), .D(16'b0), .ADDSUBTOP(1'b0), .ADDSUBBOT(1'b0),
        .AHOLD(1'b0), .BHOLD(1'b0), .CHOLD(1'b0), .DHOLD(1'b0),
        .IRSTTOP(1'b0), .IRSTBOT(1'b0), .ORSTTOP(1'b0), .ORSTBOT(1'b0),
        .OLOADTOP(1'b0), .OLOADBOT(1'b0), .OHOLDTOP(1'b0), .OHOLDBOT(1'b0),
        .CI(1'b0), .ACCUMCI(1'b0), .SIGNEXTIN(1'b0),
        .SIGNEXTOUT(), .CO(), .ACCUMCO()
    );

    // --- Statically configure the DSP as a 16x16 signed multiplier ---
    defparam dsp_inst.TOPOUTPUT_SELECT = "11"; // "11" selects multiplier output
    defparam dsp_inst.BOTOUTPUT_SELECT = "11"; // "
    defparam dsp_inst.A_SIGNED = 1'b1;         // Input A is signed
    defparam dsp_inst.B_SIGNED = 1'b1;         // Input B is signed
    
    // Disable all internal registers for a simple combinational multiplier
    defparam dsp_inst.PIPELINE_16x16_MULT_REG1 = 1'b0;
    defparam dsp_inst.PIPELINE_16x16_MULT_REG2 = 1'b0;
    defparam dsp_inst.A_REG = 1'b0;
    defparam dsp_inst.B_REG = 1'b0;
    defparam dsp_inst.C_REG = 1'b0;
    defparam dsp_inst.D_REG = 1'b0;
    defparam dsp_inst.MODE_8x8 = 1'b0; // Use full 16x16 mode

endmodule
