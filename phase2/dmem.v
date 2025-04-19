// dmem.v - Data Memory (256 x 8-bit)
// This module is intended to be used for runtime data storage.
// It supports synchronous write when 'we' is high and provides the data at the specified address on 'data_out'.
module dmem(
    input clk,
    input reset,
    input we,               // Write enable
    input [7:0] addr,       // 8-bit address (0-255)
    input [7:0] data_in,    // Data to be written
    output reg [7:0] data_out // Data output for read operations
);
    reg [7:0] mem [0:255];
    integer i;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            // Initialize the data memory to 0 on reset
            for(i = 0; i < 256; i = i + 1)
                mem[i] <= 8'd0;
            data_out <= 8'd0;
        end else begin
            if (we)
                mem[addr] <= data_in;
            // Read data from the current address
            data_out <= mem[addr];
        end
    end
endmodule
// module dmem(
//     input clk,
//     input reset,
//     input we,               // Write enable
//     input [7:0] addr,       // 8-bit address (0-255)
//     input [7:0] data_in,    // Data to be written
//     output reg [7:0] data_out // Data output for read operations
// );
//     reg [7:0] memory [0:255];

//     always @(posedge clk) begin
//         if (reset) begin
//             data_out <= 8'd0;
//         end else begin
//             if (we)
//                 memory[addr] <= data_in;
//             data_out <= memory[addr];
//         end
//     end

//     // For debug
//     function [7:0] read;
//         input [7:0] addr;
//         read = memory[addr];
//     endfunction
// endmodule
