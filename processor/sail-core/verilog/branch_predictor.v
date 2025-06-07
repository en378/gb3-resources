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
 *		Branch Predictor FSM (1 024‐entry 2‐bit GShare)
 *
 *  - Global history = 10 bits
 *  - Index into PHT = in_addr[11:2] XOR history
 *  - 2‐bit saturating counters stored in SB_RAM40_4K (configured as 1024×4 bits)
 *    • Only bits [1:0] of each word are used
 *    • INIT_xx parameters set to 0 → counters power up “00” (strongly not‐taken)
 *  - One‐cycle pipelined index + decode, synchronous write on negedge capture
 */

module branch_predictor(
		clk,
		actual_branch_decision,
		branch_decode_sig,
		branch_mem_sig,
		in_addr,
		offset,
		branch_addr,
		prediction
	);

	/*
	 *	inputs
	 */
	input			clk;
	input			actual_branch_decision;
	input			branch_decode_sig;
	input			branch_mem_sig;
	input	[31:0]	in_addr;
	input	[31:0]	offset;

	/*
	 *	outputs
	 */
	output	[31:0]	branch_addr;
	output			prediction;

	/*
	 *	internal signals
	 */
	// Global history (10 bits)
	reg	[9:0]	history;
	// Pipeline register for RAM address
	reg	[9:0]	read_addr_reg;
	// Register branch_mem_sig on negedge
	reg			branch_mem_sig_reg;

	// RAM read data (4 bits wide, we use only [1:0])
	wire	[3:0]	ram_rdata;
	// Old 2‐bit counter = ram_rdata[1:0]
	wire	[1:0]	old_counter = ram_rdata[1:0];
	// New 2‐bit saturating counter
	wire	[1:0]	new_counter = actual_branch_decision
					? ((old_counter == 2'b11) ? 2'b11 : old_counter + 1'b1)
					: ((old_counter == 2'b00) ? 2'b00 : old_counter - 1'b1);

	// Write‐enable for PHT (registered on negedge)
	wire			ram_we = branch_mem_sig_reg;
	// Raw 10‐bit index = PC[11:2]
	wire	[9:0]	raw_idx = in_addr[11:2];
	// XOR with global history
	wire	[9:0]	idx = raw_idx ^ history;
	// Write data (4 bits, upper bits = 0)
	wire	[3:0]	ram_wdata = { 2'b00, new_counter };

	/*
	 *	initialize registers
	 */
	initial begin
		history            = 10'b0;
		read_addr_reg      = 10'b0;
		branch_mem_sig_reg = 1'b0;
	end

	/*
	 *	capture branch_mem_sig on negedge so that on the next posedge,
	 *  ram_we = 1 and ram_rdata still reflects old contents
	 */
	always @(negedge clk) begin
		branch_mem_sig_reg <= branch_mem_sig;
	end

	/*
	 *	pipeline index/decode on posedge, update history on actual branch resolve
	 */
	always @(posedge clk) begin
		read_addr_reg <= idx;
		if (branch_mem_sig) begin
			history <= { history[8:0], actual_branch_decision };
		end
	end

	/*
	 *	outputs
	 */
	assign branch_addr = in_addr + offset;
	assign prediction  = ram_rdata[1] & branch_decode_sig;

	/*
	 *	SB_RAM40_4K primitive: 1 024×4b table, synchronous read/write
	 *  • WRITE_MODE = 2 → read returns old data, writes new data same cycle
	 *  • We only use bits [1:0] of each 4‐bit word
	 *  • All INIT_xx = 0 → counters start at “00” (strongly not‐taken)
	 */
	SB_RAM40_4K #(
		.WRITE_MODE(2),
		.INIT_0(256'h0000_0000_0000_0000_0000_0000_0000_0000),
		.INIT_1(256'h0000_0000_0000_0000_0000_0000_0000_0000),
		.INIT_2(256'h0000_0000_0000_0000_0000_0000_0000_0000),
		.INIT_3(256'h0000_0000_0000_0000_0000_0000_0000_0000),
		.INIT_4(256'h0000_0000_0000_0000_0000_0000_0000_0000),
		.INIT_5(256'h0000_0000_0000_0000_0000_0000_0000_0000),
		.INIT_6(256'h0000_0000_0000_0000_0000_0000_0000_0000),
		.INIT_7(256'h0000_0000_0000_0000_0000_0000_0000_0000),
		.INIT_8(256'h0000_0000_0000_0000_0000_0000_0000_0000),
		.INIT_9(256'h0000_0000_0000_0000_0000_0000_0000_0000),
		.INIT_A(256'h0000_0000_0000_0000_0000_0000_0000_0000),
		.INIT_B(256'h0000_0000_0000_0000_0000_0000_0000_0000),
		.INIT_C(256'h0000_0000_0000_0000_0000_0000_0000_0000),
		.INIT_D(256'h0000_0000_0000_0000_0000_0000_0000_0000),
		.INIT_E(256'h0000_0000_0000_0000_0000_0000_0000_0000),
		.INIT_F(256'h0000_0000_0000_0000_0000_0000_0000_0000)
	) gshare_table (
		.RDATA  (ram_rdata),     // read old 4‐bit word (we use [1:0])
		.RADDR  (read_addr_reg), // pipelined index
		.RCLK   (clk),
		.RE     (1'b1),          // always read

		.WCLK   (clk),
		.WADDR  (read_addr_reg), // same address as RADDR
		.WDATA  (ram_wdata),     // {2'b00, new_counter}
		.WE     (ram_we)         // asserted when branch_mem_sig_reg=1
	);

endmodule
