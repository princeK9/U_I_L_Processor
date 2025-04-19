// pc.v - Program Counter for LIPSI Processor

module pc (
    input clk,
    input reset,
    input load,             // Load PC with a value (e.g., jump)
    input inc,              // Increment PC
    input [7:0] load_value, // Value to load when load = 1
    output reg [7:0] pc_out // Current program counter
);

    always @(posedge clk or posedge reset) begin
        if (reset)
            pc_out <= 8'd0;
        else if (load)
            pc_out <= load_value;
        else if (inc)
            pc_out <= pc_out + 1;
        // If neither load nor inc, hold value
    end

endmodule
