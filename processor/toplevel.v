// top.v — Top-level connecting CPU with external GShare predictor
// Verilog-2005, Icarus/nextpnr/Yosys-friendly

module top (
    output [7:0] led
);

    //==================================================
    // Clock generation (iCE40 HFOSC)
    //==================================================
    wire        clk;
    reg         ENCLKHF       = 1'b1;
    reg         CLKHF_POWERUP = 1'b1;

    SB_HFOSC #(
        .CLKHF_DIV("0b10")      // divide by 8 → ~12.5 MHz
    ) u_hfosc (
        .CLKHFEN(ENCLKHF),
        .CLKHFPU(CLKHF_POWERUP),
        .CLKHF  (clk)
    );

    //==================================================
    // Memory interface signals
    //==================================================
    // CPU ↔ I-cache
    wire [31:0] inst_in;            // PC from CPU
    wire [31:0] inst_out;           // instruction back to CPU
    // I-cache ↔ backing ROM
    wire [31:0] inst_mem_addr;
    wire [31:0] inst_mem_out;
    wire        icache_ready;

    // CPU ↔ Data-mem
    wire [31:0] data_out, data_addr, data_WrData;
    wire        data_memwrite, data_memread;
    wire  [3:0] data_sign_mask;
    wire        data_clk_stall;

    //==================================================
    // Branch predictor interface
    //==================================================
    wire        branch_decode_sig;
    wire [31:0] branch_pc_decode;
    wire [31:0] branch_offset;
    wire        branch_mem_sig;
    wire        actual_branch_decision;
    wire [31:0] branch_target;
    wire        predictor_pred;

    // Register the predictor’s output to align with CPU decode stage
    reg pred_r_delay;
    always @(posedge clk) begin
        if (data_clk_stall)
            pred_r_delay <= 1'b0;
        else
            pred_r_delay <= predictor_pred;
    end

    //==================================================
    // CPU instantiation (external predictor ports)
    //==================================================
    wire clk_proc = (data_clk_stall || ~icache_ready) ? 1'b1 : clk;

    cpu u_cpu (
        .clk                    (clk_proc),
        .inst_mem_in            (inst_in),
        .inst_mem_out           (inst_out),
        .data_mem_out           (data_out),
        .data_mem_addr          (data_addr),
        .data_mem_WrData        (data_WrData),
        .data_mem_memwrite      (data_memwrite),
        .data_mem_memread       (data_memread),
        .data_mem_sign_mask     (data_sign_mask),

        // Branch predictor ports (external)
        .branch_decode_sig      (branch_decode_sig),
        .branch_pc_decode       (branch_pc_decode),
        .branch_offset          (branch_offset),
        .branch_mem_sig         (branch_mem_sig),
        .actual_branch_decision (actual_branch_decision),
        .branch_predicted       (pred_r_delay),    // registered prediction
        .branch_target          (branch_target)
    );

    //==================================================
    // GShare branch predictor instance (top level)
    //==================================================
    branch_predictor u_bp (
        .clk                    (clk),
        .actual_branch_decision (actual_branch_decision),
        .branch_decode_sig      (branch_decode_sig),
        .branch_mem_sig         (branch_mem_sig),
        .in_addr                (branch_pc_decode),
        .offset                 (branch_offset),
        .branch_addr            (branch_target),
        .prediction             (predictor_pred)
    );

    //==================================================
    // Instruction cache
    //==================================================
    icache u_icache (
        .clk       (clk),
        .addr      (inst_in),       // PC from CPU
        .data_out  (inst_out),      // instruction to CPU
        .ready     (icache_ready),  // low while refilling
        .mem_addr  (inst_mem_addr), // drives real ROM
        .mem_data  (inst_mem_out)   // ROM → cache data
    );

    //==================================================
    // Instruction memory
    //==================================================
    instruction_memory u_inst_mem (
    .addr (inst_mem_addr),
    .out  (inst_mem_out)
    );

    //==================================================
    // Data memory + LED interface
    //==================================================
    data_mem u_data_mem (
        .clk       (clk),
        .addr      (data_addr),
        .write_data(data_WrData),
        .memwrite  (data_memwrite),
        .memread   (data_memread),
        .read_data (data_out),
        .sign_mask (data_sign_mask),
        .led       (led),
        .clk_stall (data_clk_stall)
    );

endmodule
