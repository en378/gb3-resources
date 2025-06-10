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

`include "../include/rv32i-defines.v"
`include "../include/sail-core-defines.v"

/*
 *	Description:
 *
 *		This module implements an ALU control unit optimized to match the
 *		simplified ALU. It maps all shift-related opcodes to the ILLEGAL
 *		ALU operation, ensuring the control unit does not request
 *		operations that have been removed from the ALU for resource
 *		optimization.
 */
module ALUControl(FuncCode, ALUCtl, Opcode);
	input [3:0]		FuncCode;
	input [6:0]		Opcode;
	output reg [6:0]	ALUCtl;

	initial begin
		ALUCtl = 7'b0;
	end

	always @(*) begin
		case (Opcode)
			/*
			 *	LUI, U-Type
			 */
			`kRV32I_INSTRUCTION_OPCODE_LUI:
				ALUCtl = 7'b0000010; // Corresponds to ADD for LUI's B-input (which is 0)

			/*
			 *	AUIPC, U-Type
			 */
			`kRV32I_INSTRUCTION_OPCODE_AUIPC:
				ALUCtl = 7'b0000010; // ADD

			/*
			 *	JAL, UJ-Type
			 */
			`kRV32I_INSTRUCTION_OPCODE_JAL:
				ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_ILLEGAL;

			/*
			 *	JALR, I-Type
			 */
			`kRV32I_INSTRUCTION_OPCODE_JALR:
				ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_ILLEGAL;

			/*
			 *	Branch, SB-Type
			 */
			`kRV32I_INSTRUCTION_OPCODE_BRANCH:
				case (FuncCode[2:0])
					3'b000: ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_BEQ;
					3'b001: ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_BNE;
					3'b100: ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_BLT;
					3'b101: ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_BGE;
					3'b110: ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_BLTU;
					3'b111: ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_BGEU;
					default: ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_ILLEGAL;
				endcase

			/*
			 *	Loads, I-Type
			 */
			`kRV32I_INSTRUCTION_OPCODE_LOAD:
				// All loads use ADD to calculate the memory address
				ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_ADD;

			/*
			 *	Stores, S-Type
			 */
			`kRV32I_INSTRUCTION_OPCODE_STORE:
				// All stores use ADD to calculate the memory address
				ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_ADD;

			/*
			 *	Immediate operations, I-Type
			 */
			`kRV32I_INSTRUCTION_OPCODE_IMMOP:
				case (FuncCode[2:0])
					3'b000: ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_ADD;    // ADDI
					3'b010: ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_SLT;    // SLTI
					3'b011: ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_SLT;    // SLTIU
					3'b100: ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_XOR;    // XORI
					3'b110: ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_OR;     // ORI
					3'b111: ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_AND;    // ANDI
					
					// --- OPTIMIZATION: Shift instructions disabled ---
					3'b001: ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_ILLEGAL; // Was SLLI
					3'b101: ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_ILLEGAL; // Was SRLI/SRAI
					// --- End of Optimization ---

					default: ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_ILLEGAL;
				endcase

			/*
			 *	ADD SUB & logic ops, R-Type
			 */
			`kRV32I_INSTRUCTION_OPCODE_ALUOP:
				case (FuncCode[2:0])
					3'b000: // ADD or SUB
						case(FuncCode[3])
							1'b0: ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_ADD; // ADD
							1'b1: ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_SUB; // SUB
							default: ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_ILLEGAL;
						endcase
					3'b010: ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_SLT;    // SLT
					3'b011: ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_SLT;    // SLTU
					3'b100: ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_XOR;    // XOR
					3'b110: ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_OR;     // OR
					3'b111: ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_AND;    // AND
					
					// --- OPTIMIZATION: Shift instructions disabled ---
					3'b001: ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_ILLEGAL; // Was SLL
					3'b101: ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_ILLEGAL; // Was SRL/SRA
					// --- End of Optimization ---
					
					default: ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_ILLEGAL;
				endcase

			`kRV32I_INSTRUCTION_OPCODE_CSRR:
				case (FuncCode[1:0])
					2'b01: ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_CSRRW; // CSRRW
					2'b10: ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_CSRRS; // CSRRS
					2'b11: ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_CSRRC; // CSRRC
					default: ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_ILLEGAL;
				endcase

			default:
				ALUCtl = `kSAIL_MICROARCHITECTURE_ALUCTL_6to0_ILLEGAL;
		endcase
	end
endmodule
