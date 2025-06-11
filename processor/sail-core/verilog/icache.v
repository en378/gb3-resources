/*
 * icache.v — 2-way 4-set, 2-word direct-mapped instruction cache
 * 4 sets (INDEX=2), 2 ways, 2 words/line (WORDOFF=1)
 * Verilog-2005, Yosys-friendly: initial blocks instead of reset
 */
module icache(
    input         clk,
    input  [31:0] addr,
    output reg [31:0] data_out,
    output reg        ready,
    output reg [31:0] mem_addr,
    input      [31:0] mem_data
);

    // parameters for 2-way cache
    localparam BYTEOFF   = 2;
    localparam WORDOFF   = 1;
    localparam SETS      = 4;                // 4 sets
    localparam INDEX     = 2;                // bits for set index
    localparam WAYS      = 2;
    localparam TAG       = 32 - BYTEOFF - WORDOFF - INDEX;
    localparam LINEWORDS = 1 << WORDOFF;

    // FSM states
    localparam IDLE   = 1'b0;
    localparam REFILL = 1'b1;
    reg state;

    // 2-way storage
    reg [TAG-1:0] tags0 [0:SETS-1], tags1 [0:SETS-1];
    reg           val0  [0:SETS-1], val1  [0:SETS-1];
    reg [31:0]    data0 [0:SETS-1][0:LINEWORDS-1];
    reg [31:0]    data1 [0:SETS-1][0:LINEWORDS-1];
    reg           lru   [0:SETS-1];  // 0=>way0 LRU, 1=>way1 LRU

    // refill state
    reg [WORDOFF-1:0] refill_cnt;
    reg [31:0]        refill_addr;

    // index/tag breakdown
    wire [TAG-1:0] tag_in = addr[31:BYTEOFF+WORDOFF+INDEX];
    wire [INDEX-1:0] set   = addr[BYTEOFF+WORDOFF+INDEX-1:BYTEOFF+WORDOFF];
    wire [WORDOFF-1:0] off   = addr[BYTEOFF+WORDOFF-1:BYTEOFF];

    integer i,j;
    initial begin
        ready       = 1;
        state       = IDLE;
        mem_addr    = 0;
        data_out    = 0;
        refill_cnt  = 0;
        refill_addr = 0;
        for(i=0;i<SETS;i=i+1) begin
            val0[i] <= 0; val1[i] <= 0; lru[i] <= 0;
            tags0[i] <= 0; tags1[i] <= 0;
            for(j=0;j<LINEWORDS;j=j+1) begin
                data0[i][j] <= 0;
                data1[i][j] <= 0;
            end
        end
    end

    always @(posedge clk) begin
        case(state)
        IDLE: begin
            // check both ways
            if (val0[set] && tags0[set]==tag_in) begin
                data_out <= data0[set][off]; ready<=1;
            end else if (val1[set] && tags1[set]==tag_in) begin
                data_out <= data1[set][off]; ready<=1;
            end else begin
                // miss, refill into LRU way
                ready       <= 0;
                state       <= REFILL;
                refill_cnt  <= 0;
                refill_addr <= addr;
                mem_addr    <= {addr[31:BYTEOFF+WORDOFF+INDEX], refill_cnt, {WORDOFF{1'b0}}};
            end
        end
        REFILL: begin
            // choose way
            if (!lru[set]) begin
                data0[set][refill_cnt] <= mem_data;
            end else begin
                data1[set][refill_cnt] <= mem_data;
            end
            if (refill_cnt==LINEWORDS-1) begin
                // finish
                if (!lru[set]) begin
                    tags0 [set] <= refill_addr[31:BYTEOFF+WORDOFF+INDEX];
                    val0  [set] <= 1;
                    data_out    <= data0[set][refill_cnt];
                end else begin
                    tags1 [set] <= refill_addr[31:BYTEOFF+WORDOFF+INDEX];
                    val1  [set] <= 1;
                    data_out    <= data1[set][refill_cnt];
                end
                // update LRU: evicted way becomes MRU
                lru[set] <= !lru[set];
                ready    <= 1;
                state    <= IDLE;
                mem_addr <= 0;
            end else begin
                refill_cnt <= refill_cnt+1;
                mem_addr    <= {refill_addr[31:BYTEOFF+WORDOFF+INDEX], refill_cnt+1, {WORDOFF{1'b0}}};
            end
        end
        endcase
    end
endmodule
