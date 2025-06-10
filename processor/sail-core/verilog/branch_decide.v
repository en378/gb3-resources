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
 *	Description:
 *
 *		This module implements the branch resolution, located in MEM stage
 */


module branch_decision(
	clk,
	Branch,
	Predicted,
	Branch_Enable,
	Jump,
	Decision,
	Mispredict,
	Branch_Jump_Trigger
);

	/*
	 *	inputs
	 */
	input        clk;
	input        Branch;            // actual_branch_decision: 1=taken, 0=not
	input        Predicted;         // predictor’s “predict taken?” (registered)
	input        Branch_Enable;     // high when that branch is valid in MEM
	input        Jump;              // 1 = unconditional jump (optional)

	/*
	 *	outputs
	 */
	output reg   Decision;          // latched actual outcome
	output reg   Mispredict;        // 1 if Predicted ≠ actual
	output reg   Branch_Jump_Trigger; // 1 = flush+redirect PC

	/*
	 *	pipeline registers (Stage 1)
	 */
	reg          branch_en_r;
	reg          pred_r;
	reg          jump_r;

	/*
	 *	initialize registers
	 */
	initial begin
		branch_en_r         = 1'b0;
		pred_r              = 1'b0;
		jump_r              = 1'b0;
		Decision            = 1'b0;
		Mispredict          = 1'b0;
		Branch_Jump_Trigger = 1'b0;
	end

	/*
	 *	Stage 1: capture inputs
	 */
	always @(posedge clk) begin
		branch_en_r <= Branch & Branch_Enable;
		pred_r      <= Predicted;
		jump_r      <= Jump;
	end

	/*
	 *	Stage 2: compute outputs based on Stage 1 registers
	 */
	always @(posedge clk) begin
		// Actual resolved outcome
		Decision <= branch_en_r;

		// Mispredict if predictor was wrong (either direction)
		Mispredict <= (pred_r & ~branch_en_r)   // predicted=1, actual=0
		            | (~pred_r &  branch_en_r); // predicted=0, actual=1

		// Trigger flush+redirect for:
		//   • unconditional jump (jump_r=1)
		//   • predicted=0 but actual=1 (NT→T)
		//   • predicted=1 but actual=0 (T→NT)
		Branch_Jump_Trigger <= jump_r
		                     | (~pred_r &  branch_en_r)
		                     | ( pred_r & ~branch_en_r);
	end

endmodule
