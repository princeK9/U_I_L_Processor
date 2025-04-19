
module imem(
    input clk,
    input reset,
    input we,
    input [7:0] addr,
    input [7:0] data_in,
    output [7:0] data_out
);
    reg [7:0] mem [0:255];
    integer i;

    // Write logic
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < 256; i = i + 1)
                mem[i] <= 8'd0;
        end else if (we) begin
            mem[addr] <= data_in;
        end
    end

    // Combinational read (happens instantly when addr changes)
    assign data_out = mem[addr];

endmodule
