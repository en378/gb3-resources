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
 *	RISC-V CONTROL UNIT
 */

`include "../include/rv32i-defines.v"
`include "../include/sail-core-defines.v"

module control(
    opcode, MemtoReg, RegWrite, MemWrite, MemRead, Branch,
    ALUSrc, Jump, Jalr, Lui, Auipc, Fence, CSRR,
    DSP_Enable
);
	input [6:0]		opcode;
	output reg		MemtoReg;
	output reg		RegWrite;
	output reg		MemWrite;
	output reg		MemRead;
	output reg		Branch;
	output reg		ALUSrc;
	output reg		Jump;
	output reg		Jalr;
	output reg		Lui;
	output reg		Auipc;
	output reg		Fence;
	output reg		CSRR;
    output reg      DSP_Enable;

	always @(*) begin
        // --- Default values for all control signals ---
        MemtoReg = 1'b0; RegWrite = 1'b0; MemWrite = 1'b0;
        MemRead = 1'b0;  Branch = 1'b0;   ALUSrc = 1'b0;
        Jump = 1'b0;     Jalr = 1'b0;     Lui = 1'b0;
        Auipc = 1'b0;    Fence = 1'b0;    CSRR = 1'b0;
        DSP_Enable = 1'b0;

		case (opcode)
			`kRV32I_INSTRUCTION_OPCODE_LUI:		begin Lui = 1'b1; RegWrite = 1'b1; end
			`kRV32I_INSTRUCTION_OPCODE_AUIPC:	begin Auipc = 1'b1; RegWrite = 1'b1; end
			`kRV32I_INSTRUCTION_OPCODE_JAL:		begin Jump = 1'b1; RegWrite = 1'b1; end
			`kRV32I_INSTRUCTION_OPCODE_JALR:	begin Jalr = 1'b1; ALUSrc = 1'b1; RegWrite = 1'b1; end
			`kRV32I_INSTRUCTION_OPCODE_BRANCH:	Branch = 1'b1;
			`kRV32I_INSTRUCTION_OPCODE_LOAD:	begin MemRead = 1'b1; ALUSrc = 1'b1; MemtoReg = 1'b1; RegWrite = 1'b1; end
			`kRV32I_INSTRUCTION_OPCODE_STORE:	begin MemWrite = 1'b1; ALUSrc = 1'b1; end
			`kRV32I_INSTRUCTION_OPCODE_IMMOP:	begin ALUSrc = 1'b1; RegWrite = 1'b1; end
			`kRV32I_INSTRUCTION_OPCODE_ALUOP:	RegWrite = 1'b1;
			`kRV32I_INSTRUCTION_OPCODE_CSRR:	begin CSRR = 1'b1; RegWrite = 1'b1; end
            `kRV32I_INSTRUCTION_OPCODE_CUSTOM0: begin RegWrite = 1'b1; DSP_Enable = 1'b1; end
		endcase
	end
endmodule

