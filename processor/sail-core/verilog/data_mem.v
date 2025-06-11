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

// Data cache, one-hot FSM + pipelined memory interface
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

// Data cache, one-hot FSM + pipelined memory interface with 8× SB_RAM40_4K

module data_mem (
	input           clk,
	input  [31:0]   addr,
	input  [31:0]   write_data,
	input           memwrite,
	input           memread,
	input  [3:0]    sign_mask,
	output reg [31:0] read_data,
	output      [7:0] led,
	output reg      clk_stall
);

	// LED register
	reg [31:0] led_reg;

	// ---------------------------------------------------------------------
	// Pipeline registers (stage 0 → stage 1)
	// ---------------------------------------------------------------------
	reg           memread_buf;
	reg           memwrite_buf;
	reg  [31:0]   write_data_buf;
	reg  [31:0]   addr_buf;
	reg  [3:0]    sign_mask_buf;

	// FSM one-hot encoding
	reg [4:0] state;
	localparam IDLE  = 5'b00001,
	           REQ   = 5'b00010,
	           PROC  = 5'b00100,
	           OUT   = 5'b01000,
	           WRITE = 5'b10000;

	initial state = IDLE;

	// ---------------------------------------------------------------------
	// Stage 1: address decode & control signals
	// ---------------------------------------------------------------------
	wire [1:0] addr_offset    = addr_buf[1:0];
	wire [9:0] block_address  = addr_buf[11:2];
	wire [6:0] inbank_addr    = block_address[6:0];
	wire [2:0] bank           = block_address[9:7];

	// read/write enables for each cycle
	wire ram_re = (state == REQ)   & memread_buf;
	wire ram_we = (state == WRITE) & memwrite_buf;

	// ---------------------------------------------------------------------
	// Stage 2: read data barrel-shift + sign/zero-extend
	// ---------------------------------------------------------------------
	wire [4:0]  shift_amt = {addr_offset, 3'b000};
	wire [31:0] shifted   = word_buf >> shift_amt;

	wire [1:0] ld_size   = sign_mask_buf[2:1];
	wire       ld_signed = sign_mask_buf[0];

	reg [31:0] loaded_data;
	always @(*) begin
		case (ld_size)
			2'b00: if (ld_signed) loaded_data = {{24{shifted[7]}}, shifted[7:0]};
			       else           loaded_data = {24'b0, shifted[7:0]};
			2'b01: if (ld_signed) loaded_data = {{16{shifted[15]}}, shifted[15:0]};
			               else loaded_data = {16'b0, shifted[15:0]};
			default: loaded_data = shifted;
		endcase
	end

	// pipeline register for read_data
	reg [31:0] loaded_reg;

	// ---------------------------------------------------------------------
	// Store‐merge logic (shared combinational)
	// ---------------------------------------------------------------------
	wire [3:0] byte_en_byte = 4'b0001 << addr_offset;
	wire [3:0] byte_en_half = addr_offset[1] ? 4'b1100 : 4'b0011;
	wire [1:0] st_size      = sign_mask_buf[2:1];
	wire [3:0] byte_en      = (st_size == 2'b00) ? byte_en_byte :
	                          (st_size == 2'b01) ? byte_en_half :
	                                               4'b1111;

	wire [31:0] shifted_wr       = write_data_buf << shift_amt;
	wire [31:0] wem32            = {{8{byte_en[3]}},
	                                 {8{byte_en[2]}},
	                                 {8{byte_en[1]}},
	                                 {8{byte_en[0]}}};
	wire [31:0] merged_word      = (word_buf & ~wem32) | (shifted_wr & wem32);
	wire [31:0] replacement_word = merged_word;
	wire [31:0] bram_mask        = ~wem32;

	// ---------------------------------------------------------------------
	// 8× SB_RAM40_4K instantiation
	// ---------------------------------------------------------------------
	wire [31:0] bank_rdata [0:7];

	genvar i;
	generate
	  for (i = 0; i < 8; i = i + 1) begin : banks
	    SB_RAM40_4K #(
	      .WRITE_MODE(2),                   // WRITE_FIRST
	      .READ_MODE (1),                   // SYNCHRONOUS
	      .INIT_FILE ("verilog/data.hex")   // preload
	    ) spram_i (
	      .RDATA  (bank_rdata[i]),
	      .RADDR  (inbank_addr),
	      .RCLK   (clk),
	      .RCLKE  (1'b1),
	      .RE     (ram_re & (bank == i)),
	      .WADDR  (inbank_addr),
	      .WCLK   (clk),
	      .WCLKE  (1'b1),
	      .WE     (ram_we & (bank == i)),
	      .WDATA  (replacement_word),
	      .MASK   (bram_mask)
	    );
	  end
	endgenerate

	// final mux of bank outputs
	reg [31:0] word_buf;
	always @(*) begin
	  case (bank)
	    3'd0:  word_buf = bank_rdata[0];
	    3'd1:  word_buf = bank_rdata[1];
	    3'd2:  word_buf = bank_rdata[2];
	    3'd3:  word_buf = bank_rdata[3];
	    3'd4:  word_buf = bank_rdata[4];
	    3'd5:  word_buf = bank_rdata[5];
	    3'd6:  word_buf = bank_rdata[6];
	    default: word_buf = bank_rdata[7];
	  endcase
	end

	// ---------------------------------------------------------------------
	// LED register update
	// ---------------------------------------------------------------------
	always @(posedge clk) begin
	  if (memwrite && addr == 32'h2000)
	    led_reg <= write_data;
	end
	assign led = led_reg[7:0];

	// ---------------------------------------------------------------------
	// Main FSM + pipelined stages
	// ---------------------------------------------------------------------
	always @(posedge clk) begin
	  case (state)
	    IDLE: begin
	      clk_stall       <= 1'b0;
	      memread_buf     <= memread;
	      memwrite_buf    <= memwrite;
	      write_data_buf  <= write_data;
	      addr_buf        <= addr;
	      sign_mask_buf   <= sign_mask;
	      if (memread) begin
	        state      <= REQ;
	        clk_stall  <= 1'b1;
	      end else if (memwrite) begin
	        state      <= WRITE;
	        clk_stall  <= 1'b1;
	      end
	    end

	    REQ: begin
	      if (memread_buf)
	        state <= PROC;
	      else
	        state <= WRITE;
	    end

	    PROC: begin
	      loaded_reg <= loaded_data;
	      state      <= OUT;
	    end

	    OUT: begin
	      read_data <= loaded_reg;
	      clk_stall <= 1'b0;
	      state     <= IDLE;
	    end

	    WRITE: begin
	      clk_stall <= 1'b0;
	      state     <= IDLE;
	    end
	  endcase
	end

endmodule
