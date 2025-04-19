// // uart_to_imem.v - Connects UART RX to Instruction Memory (IMEM)
// module uart_to_imem(
//     input clk,
//     input reset,
//     input rx,
//     output reg [7:0] last_received_byte,  // Latest assembled instruction byte
//     output reg [7:0] mem_data_out,        // Last written instruction (from IMEM)
//     output reg waiting_led,               // High when waiting for second nibble
//     output reg rx_done                    // High after timeout (no RX for 5 sec)
// );

//     // Wires from UART receiver
//     wire [7:0] uart_data;
//     wire uart_valid;

//     // Instantiate working UART RX (your original design)
//     uart_rx uart_receiver (
//         .clk(clk),
//         .reset(reset),
//         .rx(rx),
//         .data_out(uart_data),
//         .data_ready(uart_valid)
//     );

//     // Instruction Memory interface
//     wire [7:0] imem_data_out;
//     reg        imem_we;
//     reg [7:0]  write_ptr;
//     reg [3:0]  nibble_buffer;
//     reg        half_full;

//     // Timeout counter
//     parameter TIMEOUT_COUNT = 500_000_000;  // 5 seconds at 100 MHz
//     reg [31:0] timeout_counter;

//     // Instantiate IMEM
//     imem instruction_memory (
//         .clk(clk),
//         .reset(reset),
//         .we(imem_we),
//         .addr(write_ptr),
//         .data_in({nibble_buffer, ascii_to_hex(uart_data)}),
//         .data_out(imem_data_out)
//     );

//     // ASCII-to-Hex conversion
//     function automatic [3:0] ascii_to_hex(input [7:0] ascii);
//         begin
//             if (ascii >= 8'd48 && ascii <= 8'd57)
//                 ascii_to_hex = ascii - 8'd48;
//             else if (ascii >= 8'd65 && ascii <= 8'd70)
//                 ascii_to_hex = ascii - 8'd55;
//             else if (ascii >= 8'd97 && ascii <= 8'd102)
//                 ascii_to_hex = ascii - 8'd87;
//             else
//                 ascii_to_hex = 4'd0;
//         end
//     endfunction

//     // Main control logic
//     always @(posedge clk or posedge reset) begin
//         if (reset) begin
//             write_ptr      <= 8'd0;
//             half_full      <= 1'b0;
//             waiting_led    <= 1'b0;
//             rx_done        <= 1'b0;
//             timeout_counter<= 32'd0;
//             imem_we        <= 1'b0;
//         end else begin
//             // Timeout logic
//             if (!uart_valid)
//                 timeout_counter <= timeout_counter + 1;
//             else
//                 timeout_counter <= 0;

//             rx_done <= (timeout_counter >= TIMEOUT_COUNT);

//             // UART byte processing
//             if (uart_valid) begin
//                 last_received_byte <= uart_data;
//                 if (!half_full) begin
//                     nibble_buffer <= ascii_to_hex(uart_data);
//                     half_full     <= 1'b1;
//                     waiting_led   <= 1'b1;
//                     imem_we       <= 1'b0;
//                 end else begin
//                     last_received_byte <= {nibble_buffer, ascii_to_hex(uart_data)};
//                     imem_we       <= 1'b1;
//                     write_ptr     <= write_ptr + 1;
//                     half_full     <= 1'b0;
//                     waiting_led   <= 1'b0;
//                 end
//             end else begin
//                 imem_we <= 1'b0;  // Only write for 1 clock cycle
//             end
//         end
//     end

//     // Connect to output
//     always @(posedge clk) begin
//         mem_data_out <= imem_data_out;
//     end

// endmodule

module uart_to_imem(
    input clk,
    input reset,
    input rx,
    output reg [7:0] last_received_byte,
    output reg [7:0] mem_data_out,
    output reg waiting_led,
    output reg rx_done
);

    wire [7:0] uart_data;
    wire uart_valid;

    uart_rx uart_receiver (
        .clk(clk),
        .reset(reset),
        .rx(rx),
        .data_out(uart_data),
        .data_ready(uart_valid)
    );

    wire [7:0] imem_data_out;
    reg imem_we;
    reg [7:0] write_ptr;
    reg [3:0] nibble_buffer;
    reg [3:0] second_nibble;
    reg half_full;

    parameter TIMEOUT_COUNT = 500_000_000;
    reg [31:0] timeout_counter;

    imem instruction_memory (
        .clk(clk),
        .reset(reset),
        .we(imem_we),
        .addr(write_ptr),
        .data_in({nibble_buffer, second_nibble}),
        .data_out(imem_data_out)
    );

    function automatic [3:0] ascii_to_hex(input [7:0] ascii);
        begin
            if (ascii >= 8'd48 && ascii <= 8'd57)
                ascii_to_hex = ascii - 8'd48;
            else if (ascii >= 8'd65 && ascii <= 8'd70)
                ascii_to_hex = ascii - 8'd55;
            else if (ascii >= 8'd97 && ascii <= 8'd102)
                ascii_to_hex = ascii - 8'd87;
            else
                ascii_to_hex = 4'd0;
        end
    endfunction

    reg write_enable_flag;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            write_ptr         <= 8'd0;
            half_full         <= 1'b0;
            waiting_led       <= 1'b0;
            rx_done           <= 1'b0;
            timeout_counter   <= 32'd0;
            imem_we           <= 1'b0;
            nibble_buffer     <= 4'd0;
            second_nibble     <= 4'd0;
            last_received_byte<= 8'd0;
            write_enable_flag <= 1'b0;
        end else begin
            if (!uart_valid)
                timeout_counter <= timeout_counter + 1;
            else
                timeout_counter <= 0;

            rx_done <= (timeout_counter >= TIMEOUT_COUNT);

            imem_we <= 1'b0;

            if (uart_valid) begin
                last_received_byte <= uart_data;
                if (!half_full) begin
                    nibble_buffer <= ascii_to_hex(uart_data);
                    half_full     <= 1'b1;
                    waiting_led   <= 1'b1;
                end else begin
                    second_nibble     <= ascii_to_hex(uart_data);
                    last_received_byte<= {nibble_buffer, ascii_to_hex(uart_data)};
                    half_full         <= 1'b0;
                    waiting_led       <= 1'b0;
                    imem_we           <= 1'b1;
                    write_enable_flag <= 1'b1;  // Flag to increment in next cycle
                end
            end

            if (write_enable_flag) begin
                write_ptr         <= write_ptr + 1;
                write_enable_flag <= 1'b0;
            end
        end
    end

    always @(posedge clk) begin
        mem_data_out <= imem_data_out;
    end

endmodule
