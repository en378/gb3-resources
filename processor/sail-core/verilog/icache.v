/*
 * icache.v — 8-line, 2-word direct-mapped instruction cache
 * Wraps around an instruction memory with handshake (ready)
 * Verilog-2005, Yosys-friendly: uses initial blocks instead of reset signal
 */
module icache(
    input         clk,
    input  [31:0] addr,       // byte address; PC[1:0] assumed 00
    output reg [31:0] data_out,
    output reg        ready,
    // backing memory interface
    output reg [31:0] mem_addr,
    input      [31:0] mem_data
);

    // cache parameters (shrunk version)
    localparam BYTEOFF   = 2;               // byte offset bits (always 00)
    localparam WORDOFF   = 1;               // words per line = 1<<WORDOFF = 2
    localparam INDEX     = 3;               // lines = 1<<INDEX = 8
    localparam TAG       = 32 - BYTEOFF - WORDOFF - INDEX;  // tag bits = 26
    localparam LINES     = 1 << INDEX;
    localparam LINEWORDS = 1 << WORDOFF;

    // FSM states
    localparam IDLE   = 1'b0;
    localparam REFILL = 1'b1;
    reg state;

    // cache storage
    reg [TAG-1:0]       tags   [0:LINES-1];
    reg                 valids [0:LINES-1];
    reg [31:0]          data_array [0:LINES-1][0:LINEWORDS-1];

    // refill pointers
    reg [WORDOFF-1:0]   refill_cnt;
    reg [31:0]          refill_addr;

    // address breakdown
    wire [TAG-1:0]    tag_in = addr[31:BYTEOFF+WORDOFF+INDEX];
    wire [INDEX-1:0]  idx    = addr[BYTEOFF+WORDOFF+INDEX-1:BYTEOFF+WORDOFF];
    wire [WORDOFF-1:0] off   = addr[BYTEOFF+WORDOFF-1:BYTEOFF];

    integer i, j;
    // Initialize all stateful storage for Yosys-friendly startup
    initial begin
        ready       = 1'b1;
        state       = IDLE;
        mem_addr    = 32'b0;
        data_out    = 32'b0;
        refill_cnt  = {WORDOFF{1'b0}};
        refill_addr = 32'b0;
        for (i = 0; i < LINES; i = i + 1) begin
            valids[i] <= 1'b0;
            tags[i]   <= {TAG{1'b0}};
            for (j = 0; j < LINEWORDS; j = j + 1)
                data_array[i][j] <= 32'b0;
        end
    end

    // FSM on rising clock
    always @(posedge clk) begin
        case (state)
            IDLE: begin
                if (valids[idx] && (tags[idx] == tag_in)) begin
                    // cache hit
                    data_out <= data_array[idx][off];
                    ready    <= 1'b1;
                end else begin
                    // miss: start refill for 2-word line
                    ready       <= 1'b0;
                    state       <= REFILL;
                    refill_cnt  <= 0;
                    refill_addr <= addr;
                    mem_addr    <= { addr[31:BYTEOFF+WORDOFF+INDEX], refill_cnt, {WORDOFF{1'b0}} };
                end
            end

            REFILL: begin
                // capture word into cache
                data_array[idx][refill_cnt] <= mem_data;
                if (refill_cnt == (LINEWORDS-1)) begin
                    // last word fetched
                    tags[idx]   <= refill_addr[31:BYTEOFF+WORDOFF+INDEX];
                    valids[idx] <= 1'b1;
                    data_out    <= data_array[idx][refill_cnt];
                    ready       <= 1'b1;
                    state       <= IDLE;
                    mem_addr    <= 32'b0;
                end else begin
                    // fetch next word
                    refill_cnt <= refill_cnt + 1;
                    mem_addr    <= { refill_addr[31:BYTEOFF+WORDOFF+INDEX], refill_cnt + 1, {WORDOFF{1'b0}} };
                end
            end
        endcase
    end

endmodule
