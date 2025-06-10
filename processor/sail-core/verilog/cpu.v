/*
	Authored 2018-2019, Ryan Voo.

	All rights reserved.
	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:

	*	Redistributions of source code must retain the above
		copyright notice, this list of conditions and the following
		disclaimer.

	*	Redistributions in binary form must reproduce the above
		copyright notice, this list of conditions and the following
		disclaimer in the documentation and/or other materials
		provided with the distribution.

	*	Neither the name of the author nor the names of its
		contributors may be used to endorse or promote products
		derived from this software without specific prior written
		permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*/



/*
 *	cpu top-level
 */

/*
 *	cpu top-level
 * Modified to include a dedicated DSP unit for custom instructions.
 */

/*
 *	cpu top-level
 * Modified to include a dedicated DSP unit for custom instructions.
 */

module cpu(clk, inst_mem_in, inst_mem_out, data_mem_out, data_mem_addr, data_mem_WrData, data_mem_memwrite, data_mem_memread, data_mem_sign_mask);
	input clk;
	output [31:0] inst_mem_in;
	input [31:0] inst_mem_out;
	input [31:0] data_mem_out;
	output [31:0] data_mem_addr;
	output [31:0] data_mem_WrData;
	output data_mem_memwrite;
	output data_mem_memread;
	output [3:0] data_mem_sign_mask;

	wire [31:0] pc_mux0, pc_in, pc_out, pc_adder_out;
	wire pcsrc;
	wire [31:0] inst_mux_out, fence_mux_out;
	wire [63:0] if_id_out;
	wire [178:0] id_ex_out;
	wire [155:0] ex_mem_out;
	wire [117:0] mem_wb_out;
	wire MemtoReg1, RegWrite1, MemWrite1, MemRead1, Branch1, Jump1, Jalr1, ALUSrc1, Lui1, Auipc1, Fence_signal, CSRR_signal, CSRRI_signal;
	wire [31:0] cont_mux_out;
	wire [31:0] regA_out, regB_out, imm_out, RegA_mux_out, RegB_mux_out, rdValOut_CSR;
	wire [31:0] RegA_AddrFwdFlush_mux_out, RegB_AddrFwdFlush_mux_out;
	wire [3:0] dataMem_sign_mask;
	wire [31:0] ex_cont_mux_out;
	wire [31:0] addr_adder_mux_out, alu_mux_out, addr_adder_sum;
	wire [6:0] alu_ctl;
	wire alu_branch_enable;
	wire [31:0] alu_result, lui_result;
	wire [31:0] auipc_mux_out, mem_csrr_mux_out;
	wire [31:0] wb_mux_out, reg_dat_mux_out;
	wire [31:0] dataMemOut_fwd_mux_out, mem_fwd1_mux_out, mem_fwd2_mux_out, wb_fwd1_mux_out, wb_fwd2_mux_out;
	wire mfwd1, mfwd2, wfwd1, wfwd2;
	wire predict;
	wire [31:0] branch_predictor_mux_out;
	wire actual_branch_decision, mistake_trigger, decode_ctrl_mux_sel, inst_mux_sel;
	wire DSP_Enable_ID;
	wire [31:0] dsp_result_EX;
	wire [31:0] execute_stage_result;
	wire [31:0] mem_regwb_mux_out;

	// IF Stage
	mux2to1 pc_mux(.input0(pc_mux0), .input1(ex_mem_out[74:43]), .select(pcsrc), .out(pc_in));
	adder pc_adder(.input1(32'b100), .input2(pc_out), .out(pc_adder_out));
	program_counter PC(.inAddr(pc_in), .outAddr(pc_out), .clk(clk));
	mux2to1 inst_mux(.input0(inst_mem_out), .input1(32'b0), .select(inst_mux_sel), .out(inst_mux_out));
	mux2to1 fence_mux(.input0(pc_adder_out), .input1(pc_out), .select(Fence_signal), .out(fence_mux_out));
	if_id if_id_reg(.clk(clk), .data_in({inst_mux_out, pc_out}), .data_out(if_id_out));

	// ID Stage
	control control_unit(.opcode({if_id_out[38:32]}), .MemtoReg(MemtoReg1), .RegWrite(RegWrite1), .MemWrite(MemWrite1), .MemRead(MemRead1), .Branch(Branch1), .ALUSrc(ALUSrc1), .Jump(Jump1), .Jalr(Jalr1), .Lui(Lui1), .Auipc(Auipc1), .Fence(Fence_signal), .CSRR(CSRR_signal), .DSP_Enable(DSP_Enable_ID));
	mux2to1 cont_mux(.input0({18'b0, DSP_Enable_ID, Jalr1, ALUSrc1, Lui1, Auipc1, Branch1, MemRead1, MemWrite1, CSRR_signal, RegWrite1, MemtoReg1, Jump1}), .input1(32'b0), .select(decode_ctrl_mux_sel), .out(cont_mux_out));
	regfile register_files(.clk(clk), .write(mem_wb_out[2]), .wrAddr(mem_wb_out[104:100]), .wrData(reg_dat_mux_out), .rdAddrA(if_id_out[51:47]), .rdDataA(regA_out), .rdAddrB(if_id_out[56:52]), .rdDataB(regB_out));
	imm_gen immediate_generator(.inst(if_id_out[63:32]), .imm(imm_out));
	ALUControl alu_control(.Opcode(if_id_out[38:32]), .FuncCode({if_id_out[62], if_id_out[46:44]}), .ALUCtl(alu_ctl));
	sign_mask_gen sign_mask_gen_inst(.func3(if_id_out[46:44]), .sign_mask(dataMem_sign_mask));
	csr_file ControlAndStatus_registers(.clk(clk), .write(mem_wb_out[3]), .wrAddr_CSR(mem_wb_out[116:105]), .wrVal_CSR(wb_mux_out), .rdAddr_CSR(if_id_out[63:52]), .rdVal_CSR(rdValOut_CSR));
	mux2to1 RegA_mux(.input0(regA_out), .input1({27'b0, if_id_out[51:47]}), .select(CSRRI_signal), .out(RegA_mux_out));
	mux2to1 RegB_mux(.input0(regB_out), .input1(rdValOut_CSR), .select(CSRR_signal), .out(RegB_mux_out));
	assign CSRRI_signal = CSRR_signal & (if_id_out[46]);
	id_ex id_ex_reg(.clk(clk), .data_in({if_id_out[63:52], regB_out, regA_out, imm_out, alu_ctl, dataMem_sign_mask, if_id_out[46:39], cont_mux_out[13], predict, cont_mux_out[6:0], DSP_Enable_ID}), .data_out(id_ex_out));

	// EX Stage
	mux2to1 ex_cont_mux(.input0({18'b0, id_ex_out[12:0]}), .input1(32'b0), .select(pcsrc), .out(ex_cont_mux_out));
	mux2to1 addr_adder_mux(.input0(id_ex_out[75:44]), .input1(wb_fwd1_mux_out), .select(id_ex_out[11]), .out(addr_adder_mux_out));
	adder addr_adder(.input1(addr_adder_mux_out), .input2(id_ex_out[107:76]), .out(addr_adder_sum));
	mux2to1 alu_mux(.input0(wb_fwd2_mux_out), .input1(id_ex_out[107:76]), .select(id_ex_out[10]), .out(alu_mux_out));
	alu alu_main(.ALUctl(id_ex_out[146:140]), .A(wb_fwd1_mux_out), .B(alu_mux_out), .ALUOut(alu_result), .Branch_Enable(alu_branch_enable));
	mux2to1 lui_mux(.input0(alu_result), .input1(id_ex_out[107:76]), .select(id_ex_out[9]), .out(lui_result));
	dsp dsp_unit (.clk(clk), .ce(1'b1), .A(wb_fwd1_mux_out[15:0]), .B(alu_mux_out[15:0]), .O(dsp_result_EX));
	mux2to1 execute_result_mux (.input0(lui_result), .input1(dsp_result_EX), .select(id_ex_out[178]), .out(execute_stage_result));
	ex_mem ex_mem_reg(.clk(clk), .data_in({id_ex_out[178:151], wb_fwd2_mux_out, execute_stage_result, alu_branch_enable, addr_adder_sum, ex_cont_mux_out[12:0]}), .data_out(ex_mem_out));

	// MEM Stage
	branch_decision branch_decide( .Branch(ex_mem_out[9]), .Predicted(ex_mem_out[10]), .Branch_Enable(ex_mem_out[75]), .Jump(ex_mem_out[0]), .Mispredict(mistake_trigger), .Decision(actual_branch_decision), .Branch_Jump_Trigger(pcsrc));
	mux2to1 auipc_mux(.input0(execute_stage_result), .input1(ex_mem_out[74:43]), .select(ex_mem_out[11]), .out(auipc_mux_out));
	mux2to1 mem_csrr_mux(.input0(auipc_mux_out), .input1(ex_mem_out[139:108]), .select(ex_mem_out[3]), .out(mem_csrr_mux_out));
	mem_wb mem_wb_reg(.clk(clk), .data_in({ex_mem_out[155:140], data_mem_out, mem_csrr_mux_out, ex_mem_out[11], ex_mem_out[2:0]}), .data_out(mem_wb_out));
	mux2to1 mem_regwb_mux(.input0(mem_csrr_mux_out), .input1(data_mem_out), .select(ex_mem_out[1]), .out(mem_regwb_mux_out));
	mux2to1 wb_mux(.input0(mem_wb_out[67:36]), .input1(mem_wb_out[99:68]), .select(mem_wb_out[1]), .out(wb_mux_out));
	mux2to1 reg_dat_mux(.input0(wb_mux_out), .input1(ex_mem_out[43:12]), .select(ex_mem_out[0]), .out(reg_dat_mux_out));

	ForwardingUnit forwarding_unit(.rs1(id_ex_out[160:156]), .rs2(id_ex_out[165:161]), .MEM_RegWriteAddr(ex_mem_out[142:138]), .WB_RegWriteAddr(mem_wb_out[104:100]), .MEM_RegWrite(ex_mem_out[2]), .WB_RegWrite(mem_wb_out[2]), .EX_CSRR_Addr(id_ex_out[178:167]), .MEM_CSRR_Addr(ex_mem_out[155:144]), .WB_CSRR_Addr(mem_wb_out[117:106]), .MEM_CSRR(ex_mem_out[3]), .WB_CSRR(mem_wb_out[3]), .MEM_fwd1(mfwd1), .MEM_fwd2(mfwd2), .WB_fwd1(wfwd1), .WB_fwd2(wfwd2));
	mux2to1 mem_fwd1_mux(.input0(id_ex_out[75:44]), .input1(dataMemOut_fwd_mux_out), .select(mfwd1), .out(mem_fwd1_mux_out));
	mux2to1 mem_fwd2_mux(.input0(id_ex_out[107:76]), .input1(dataMemOut_fwd_mux_out), .select(mfwd2), .out(mem_fwd2_mux_out));
	mux2to1 wb_fwd1_mux(.input0(mem_fwd1_mux_out), .input1(wb_mux_out), .select(wfwd1), .out(wb_fwd1_mux_out));
	mux2to1 wb_fwd2_mux(.input0(mem_fwd2_mux_out), .input1(wb_mux_out), .select(wfwd2), .out(wb_fwd2_mux_out));
	mux2to1 dataMemOut_fwd_mux(.input0(ex_mem_out[105:74]), .input1(data_mem_out), .select(ex_mem_out[1]), .out(dataMemOut_fwd_mux_out));
	assign predict = 1'b0;
	mux2to1 branch_predictor_mux(.input0(fence_mux_out), .input1(32'b0), .select(predict), .out(branch_predictor_mux_out));
	mux2to1 mistaken_branch_mux(.input0(branch_predictor_mux_out), .input1(id_ex_out[43:12]), .select(mistake_trigger), .out(pc_mux0));

	assign decode_ctrl_mux_sel = pcsrc | mistake_trigger;
	assign inst_mux_sel = pcsrc | predict | mistake_trigger | Fence_signal;
	assign inst_mem_in = pc_out;
	assign data_mem_addr = auipc_mux_out;
	assign data_mem_WrData = wb_fwd2_mux_out;
	assign data_mem_memwrite = ex_mem_out[4];
	assign data_mem_memread = ex_mem_out[5];
	assign data_mem_sign_mask = ex_mem_out[145:142];
endmodule
