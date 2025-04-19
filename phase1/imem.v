// imem.v - Instruction Memory (256 x 8-bit)
module imem(
    input clk,
    input reset,
    input we,               // Write enable
    input [7:0] addr,       // 8-bit address (0-255)
    input [7:0] data_in,    // Data to be written (instruction byte)
    output reg [7:0] data_out // Data output (for readback)
);
    reg [7:0] mem [0:255];
    integer i;
    
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for(i = 0; i < 256; i = i + 1)
                mem[i] <= 8'd0;
            data_out <= 8'd0;
        end else begin
            if (we)
                mem[addr] <= data_in;
            data_out <= mem[addr];  // Read at current address
        end
    end
endmodule

