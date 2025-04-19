// acc.v - Accumulator Register for the LIPSI Processor Core
// This module implements an 8-bit accumulator.
// On the rising edge of clk, if reset is high, the accumulator is cleared to 0.
// Otherwise, if load is high, the accumulator is updated with data_in.
// If neither reset nor load is asserted, the accumulator retains its current value.

module acc(
    input clk,               
    input reset,              // Synchronous reset (active high)
    input load,               // Load enable signal (when high, load data_in into accumulator)
    input [7:0] data_in,      // Data input 
    output reg [7:0] acc_out  // 8-bit accumulator output
);
    // acc_out is Accumulator output, which is an 8-bit register.
    always @(posedge clk or posedge reset) begin
        if (reset) begin
           acc_out <= 8'd0;   
        end 
        else if (load) begin
            acc_out <= data_in; 
        end
    end

endmodule
