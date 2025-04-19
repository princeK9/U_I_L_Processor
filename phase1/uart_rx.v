// module uart_rx (
//     input clk,
//     input reset,
//     input rx,
//     output reg [7:0] data_out,
//     output reg data_ready
// );
//     localparam int BAUD_RATE = 9600;
//     localparam int CLK_FREQ = 100_000_000;
//     localparam int CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;

//     typedef enum logic [2:0] {
//         IDLE = 3'b000,
//         START_BIT = 3'b001,
//         RECEIVE_BITS = 3'b010,
//         STOP_BIT = 3'b011,
//         COMPLETE = 3'b100
//     } state_t;

//     state_t state = IDLE;
//     reg [3:0] bit_count;
//     reg [7:0] shift_reg;
//     reg [15:0] clk_count;
//     reg rx_stable, rx_prev;

//     always @(posedge clk) begin
//         rx_prev <= rx_stable;
//         rx_stable <= rx;
//     end

//     always @(posedge clk or posedge reset) begin
//         if (reset) begin
//             state <= IDLE;
//             bit_count <= 0;
//             shift_reg <= 0;
//             data_out <= 0;
//             data_ready <= 0;
//             clk_count <= 0;
//         end else begin
//             case (state)
//                 IDLE: begin
//                     data_ready <= 0;
//                     if (rx_stable == 0 && rx_prev == 1)
//                         state <= START_BIT;
//                 end

//                 START_BIT: begin
//                     if (clk_count == (CLKS_PER_BIT / 2)) begin
//                         clk_count <= 0;
//                         if (rx_stable == 0)
//                             state <= RECEIVE_BITS;
//                         else
//                             state <= IDLE;
//                     end else
//                         clk_count <= clk_count + 1;
//                 end

//                 RECEIVE_BITS: begin
//                     if (clk_count < CLKS_PER_BIT - 1) begin
//                         clk_count <= clk_count + 1;
//                     end else begin
//                         clk_count <= 0;
//                         shift_reg <= {rx_stable, shift_reg[7:1]};
//                         if (bit_count < 7)
//                             bit_count <= bit_count + 1;
//                         else begin
//                             bit_count <= 0;
//                             state <= STOP_BIT;
//                         end
//                     end
//                 end

//                 STOP_BIT: begin
//                     if (clk_count < CLKS_PER_BIT - 1)
//                         clk_count <= clk_count + 1;
//                     else begin
//                         clk_count <= 0;
//                         if (rx_stable) begin
//                             data_out <= shift_reg;
//                             data_ready <= 1;
//                             state <= COMPLETE;
//                         end else
//                             state <= IDLE;
//                     end
//                 end

//                 COMPLETE: begin
//                     data_ready <= 0;
//                     if (rx_stable)
//                         state <= IDLE;
//                 end
//             endcase
//         end
//     end
// endmodule


module uart_rx (
    input clk,
    input reset,
    input rx,
    output reg [7:0] data_out,
    output reg data_ready
);

    parameter BAUD_RATE = 9600;
    parameter CLK_FREQ = 100_000_000;
    parameter CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;

    reg [2:0] state;
    reg [3:0] bit_count;
    reg [7:0] shift_reg;
    reg [15:0] clk_count;
    reg rx_stable, rx_prev;

    // State encoding
    localparam IDLE        = 3'b000;
    localparam START_BIT   = 3'b001;
    localparam RECEIVE_BITS= 3'b010;
    localparam STOP_BIT    = 3'b011;
    localparam COMPLETE    = 3'b100;

    always @(posedge clk) begin
        rx_prev <= rx_stable;
        rx_stable <= rx;
    end

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state <= IDLE;
            bit_count <= 0;
            shift_reg <= 0;
            data_out <= 0;
            data_ready <= 0;
            clk_count <= 0;
        end else begin
            case (state)
                IDLE: begin
                    data_ready <= 0;
                    if (rx_stable == 0 && rx_prev == 1)
                        state <= START_BIT;
                end

                START_BIT: begin
                    if (clk_count == (CLKS_PER_BIT / 2)) begin
                        clk_count <= 0;
                        if (rx_stable == 0)
                            state <= RECEIVE_BITS;
                        else
                            state <= IDLE;
                    end else
                        clk_count <= clk_count + 1;
                end

                RECEIVE_BITS: begin
                    if (clk_count < CLKS_PER_BIT - 1) begin
                        clk_count <= clk_count + 1;
                    end else begin
                        clk_count <= 0;
                        shift_reg <= {rx_stable, shift_reg[7:1]};
                        if (bit_count < 7)
                            bit_count <= bit_count + 1;
                        else begin
                            bit_count <= 0;
                            state <= STOP_BIT;
                        end
                    end
                end

                STOP_BIT: begin
                    if (clk_count < CLKS_PER_BIT - 1)
                        clk_count <= clk_count + 1;
                    else begin
                        clk_count <= 0;
                        if (rx_stable) begin
                            data_out <= shift_reg;
                            data_ready <= 1;
                            state <= COMPLETE;
                        end else
                            state <= IDLE;
                    end
                end

                COMPLETE: begin
                    data_ready <= 0;
                    if (rx_stable)
                        state <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end
endmodule
