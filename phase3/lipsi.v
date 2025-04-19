module top_level (
    input        clk,
    input        reset,
    input        rx,
    output [3:0] an,
    output [6:0] seg,
    output       waiting_led,
    output       rx_done,
    output       done
);

    wire [7:0] last_received_byte;
    wire [7:0] imem_data_to_fsm;
    wire       waiting_led_internal;
    wire       rx_done_internal;
    wire [7:0] fsm_pc_addr;
    wire [7:0] final_acc;

    uart_to_imem u_uart_to_imem (
        .clk(clk),
        .reset(reset),
        .rx(rx),
        .fsm_pc_addr(fsm_pc_addr),
        .last_received_byte(last_received_byte),
        .imem_data_out(imem_data_to_fsm),
        .waiting_led(waiting_led_internal),
        .rx_done(rx_done_internal)
    );

    fsm u_fsm (
        .clk(clk),
        .reset(reset),
        .rx_done(rx_done_internal),
        .instruction_in(imem_data_to_fsm),
        .fsm_pc_addr(fsm_pc_addr),
        .done(done),
        .final_acc(final_acc)
    );

    display_driver u_display_driver (
        .clk(clk),
        .reset(reset),
        .binary_value({last_received_byte, final_acc}),
        .an_out(an),
        .seg_out(seg)
    );

    assign waiting_led = waiting_led_internal;
    assign rx_done     = rx_done_internal;

endmodule

module uart_to_imem(
    input clk,
    input reset,
    input rx,
    input [7:0] fsm_pc_addr,
    output reg [7:0] last_received_byte,
    output [7:0] imem_data_out,
    output reg waiting_led,
    output reg rx_done
);

    wire [7:0] uart_data;
    wire       uart_valid;

    uart_rx uart_receiver (
        .clk(clk),
        .reset(reset),
        .rx(rx),
        .data_out(uart_data),
        .data_ready(uart_valid)
    );

    wire [7:0] imem_read_data;
    reg        imem_we;
    reg [7:0]  write_ptr;
    reg [3:0]  nibble_buffer;
    reg [3:0]  second_nibble;
    reg        half_full;

    parameter TIMEOUT_COUNT = 500_000_000;
    reg [31:0] timeout_counter;

    imem instruction_memory (
        .clk(clk),
        .reset(reset),
        .we(imem_we),
        .addr(imem_we ? write_ptr : fsm_pc_addr),
        .data_in({nibble_buffer, second_nibble}),
        .data_out(imem_read_data)
    );

    assign imem_data_out = imem_read_data;

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
            write_ptr        <= 8'd0;
            half_full        <= 1'b0;
            waiting_led      <= 1'b0;
            rx_done          <= 1'b0;
            timeout_counter  <= 32'd0;
            imem_we          <= 1'b0;
            nibble_buffer    <= 4'd0;
            second_nibble    <= 4'd0;
            last_received_byte <= 8'd0;
            write_enable_flag <= 1'b0;
        end else begin
            if (!uart_valid && !rx_done) begin
                if (timeout_counter < TIMEOUT_COUNT)
                    timeout_counter <= timeout_counter + 1;
                else
                    rx_done <= 1'b1;
             end else begin
                 timeout_counter <= 0;
             end

            imem_we <= 1'b0;

            if (uart_valid) begin
                last_received_byte <= uart_data;
                if (!half_full) begin
                    nibble_buffer <= ascii_to_hex(uart_data);
                    half_full     <= 1'b1;
                    waiting_led   <= 1'b1;
                end else begin
                    second_nibble    <= ascii_to_hex(uart_data);
                    last_received_byte <= {nibble_buffer, ascii_to_hex(uart_data)};
                    half_full        <= 1'b0;
                    waiting_led      <= 1'b0;
                    imem_we          <= 1'b1;
                    write_enable_flag<= 1'b1;
                end
            end

            if (write_enable_flag) begin
                write_ptr <= write_ptr + 1;
                write_enable_flag <= 1'b0;
            end
        end
    end

endmodule

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

    always @(posedge clk) begin
        rx_prev <= rx_stable;
        rx_stable <= rx;
    end

    localparam IDLE = 3'b000;
    localparam START_BIT = 3'b001;
    localparam RECEIVE_BITS = 3'b010;
    localparam STOP_BIT = 3'b011;
    localparam COMPLETE = 3'b100;

    initial begin
        state = IDLE;
        data_ready = 1'b0;
        clk_count = 0;
        bit_count = 0;
    end

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state <= IDLE;
            bit_count <= 0;
            shift_reg <= 0;
            data_out <= 0;
            data_ready <= 1'b0;
            clk_count <= 0;
        end else begin
            if (state != COMPLETE)
                data_ready <= 1'b0;

            case (state)
                IDLE: begin
                    if (rx_stable == 0 && rx_prev == 1) begin
                        state <= START_BIT;
                        clk_count <= 0;
                    end
                end

                START_BIT: begin
                    if (clk_count == (CLKS_PER_BIT / 2) - 1) begin
                       if (rx_stable == 0) begin
                            state <= RECEIVE_BITS;
                            clk_count <= 0;
                            bit_count <= 0;
                        end else begin
                            state <= IDLE;
                        end
                    end else begin
                        clk_count <= clk_count + 1;
                    end
                end

                RECEIVE_BITS: begin
                    if (clk_count == CLKS_PER_BIT - 1) begin
                        clk_count <= 0;
                        shift_reg <= {rx_stable, shift_reg[7:1]};
                        if (bit_count < 7) begin
                            bit_count <= bit_count + 1;
                        end else begin
                           state <= STOP_BIT;
                        end
                    end else begin
                        clk_count <= clk_count + 1;
                    end
                end

               STOP_BIT: begin
                    if (clk_count == CLKS_PER_BIT - 1) begin
                        clk_count <= 0;
                        if (rx_stable == 1) begin
                            data_out <= shift_reg;
                            data_ready <= 1'b1;
                            state <= COMPLETE;
                        end else begin
                            state <= IDLE;
                        end
                    end else begin
                        clk_count <= clk_count + 1;
                    end
                end

                COMPLETE: begin
                    data_ready <= 1'b0;
                    state <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end
endmodule

module fsm (
    input clk,
    input reset,
    input rx_done,
    input [7:0] instruction_in,
    output [7:0] fsm_pc_addr,
    output reg done,
    output [7:0] final_acc
);

    parameter IDLE = 4'd0;
    parameter FETCH = 4'd1;
    parameter DECODE = 4'd2;
    parameter EXEC1 = 4'd3;
    parameter EXEC2 = 4'd4;
    parameter HALT = 4'd6;

    reg [3:0] state, next_state;

    reg [7:0] instr;
    reg [7:0] op_byte;
    reg [7:0] effective_addr;

    reg pc_inc, pc_load;
    reg [7:0] pc_load_val;
    wire [7:0] pc_addr_internal;

    reg acc_load;
    wire [7:0] acc_out;

    reg dmem_we;
    reg [7:0] dmem_addr, dmem_data_in;
    wire [7:0] dmem_data_out;

    reg [7:0] alu_A, alu_B;
    reg [2:0] alu_op;
    reg alu_shift_en;
    reg [1:0] alu_shift_type;
    reg alu_carry_in_comb;
    wire [7:0] alu_result;
    wire alu_cout;
    wire alu_zero;

    reg carry_flag;
    reg zero_flag;
    reg update_carry_flag;
    reg update_zero_flag;

    pc u_pc (
        .clk(clk),
        .reset(reset),
        .load(pc_load),
        .inc(pc_inc),
        .load_value(pc_load_val),
        .pc_out(pc_addr_internal)
    );
    assign fsm_pc_addr = pc_addr_internal;

    acc u_acc (
        .clk(clk),
        .reset(reset),
        .load(acc_load),
        .data_in(alu_result),
        .acc_out(acc_out)
    );
    assign final_acc = acc_out;

    dmem u_dmem (
        .clk(clk),
        .reset(reset),
        .we(dmem_we),
        .addr(dmem_addr),
        .data_in(dmem_data_in),
        .data_out(dmem_data_out)
    );

    alu u_alu (
        .opcode(alu_op),
        .A(alu_A),
        .op(alu_B),
        .carry_in(alu_carry_in_comb),
        .shift_en(alu_shift_en),
        .shift_type(alu_shift_type),
        .result(alu_result),
        .carry_out(alu_cout),
        .zero(alu_zero)
    );

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state <= IDLE;
            instr <= 8'b0;
            op_byte <= 8'b0;
            effective_addr <= 8'b0;
            carry_flag <= 1'b0;
            zero_flag <= 1'b1;
        end else begin
            state <= next_state;
            if (update_carry_flag) begin
                carry_flag <= carry_flag + alu_cout;
            end
            if (update_zero_flag) begin
                zero_flag <= alu_zero;
            end
            if (state == FETCH) begin
                instr <= instruction_in;
            end
            if (state == DECODE && (instr[7:4] == 4'b1100 || instr[7:4] == 4'b1101)) begin
                op_byte <= instruction_in;
            end
            if (state == EXEC1 && (instr[7:4] == 4'b1010 || instr[7:4] == 4'b1011)) begin
                effective_addr <= dmem_data_out;
            end
        end
    end

    always @(*) begin
        next_state = state;
        pc_inc = 1'b0;
        pc_load = 1'b0;
        pc_load_val = 8'b0;
        acc_load = 1'b0;
        dmem_we = 1'b0;
        dmem_addr = 8'b0;
        dmem_data_in = 8'b0;
        alu_op = 3'b111;
        alu_shift_en = 1'b0;
        alu_shift_type = 2'b00;
        alu_carry_in_comb = carry_flag;
        alu_A = acc_out;
        alu_B = 8'b0;
        done = 1'b0;
        update_carry_flag = 1'b0;
        update_zero_flag = 1'b0;

        case (state)
            IDLE: begin
                if (rx_done) begin
                    next_state = FETCH;
                end
            end
            FETCH: begin
                next_state = DECODE;
                pc_inc = 1'b1;
            end
            DECODE: begin
                if (instr[7] == 1'b0) begin
                    dmem_addr = {4'd0, instr[3:0]};
                    next_state = EXEC1;
                end else begin
                    case (instr[7:4])
                        4'b1000: begin
                            dmem_addr = {4'd0, instr[3:0]};
                            dmem_data_in = acc_out;
                            dmem_we = 1'b1;
                            next_state = FETCH;
                        end
                        4'b1001: begin
                            dmem_addr = {4'd0, instr[3:0]};
                            dmem_data_in = pc_addr_internal + 1;
                            dmem_we = 1'b1;
                            pc_load = 1'b1;
                            pc_load_val = acc_out;
                            next_state = FETCH;
                        end
                        4'b1010: begin
                            dmem_addr = {4'd0, instr[3:0]};
                            next_state = EXEC1;
                        end
                        4'b1011: begin
                            dmem_addr = {4'd0, instr[3:0]};
                            next_state = EXEC1;
                        end
                        4'b1100: begin
                            next_state = EXEC1;
                            pc_inc = 1'b1;
                        end
                        4'b1101: begin
                            next_state = EXEC1;
                            pc_inc = 1'b1;
                        end
                        4'b1110: begin
                            alu_shift_en = 1'b1;
                            alu_shift_type = instr[1:0];
                            alu_A = acc_out;
                            alu_carry_in_comb = carry_flag;
                            acc_load = 1'b1;
                            update_zero_flag = 1'b1;
                            if (instr[1:0] == 2'b10 || instr[1:0] == 2'b11) begin
                                update_carry_flag = 1'b1;
                            end
                            next_state = FETCH;
                        end
                        4'b1111: begin
                            if (instr[3:0] == 4'b1111) begin
                                done = 1'b1;
                                next_state = HALT;
                            end else begin
                                next_state = FETCH;
                            end
                        end
                        default: begin
                            next_state = HALT;
                        end
                    endcase
                end
            end
            EXEC1: begin
                if (instr[7] == 1'b0) begin
                    alu_op = instr[6:4];
                    alu_B = dmem_data_out;
                    alu_A = acc_out;
                    alu_carry_in_comb = carry_flag;
                    acc_load = 1'b1;
                    update_zero_flag = 1'b1;
                    if (alu_op == 3'b000 || alu_op == 3'b001 || alu_op == 3'b010 || alu_op == 3'b011) begin
                        update_carry_flag = 1'b1;
                    end
                    next_state = FETCH;
                end else begin
                    case (instr[7:4])
                        4'b1010: begin
                            dmem_addr = effective_addr;
                            next_state = EXEC2;
                        end
                        4'b1011: begin
                            next_state = EXEC2;
                        end
                        4'b1100: begin
                            alu_op = instr[2:0];
                            alu_B = op_byte;
                            alu_A = acc_out;
                            alu_carry_in_comb = carry_flag;
                            acc_load = 1'b1;
                            update_zero_flag = 1'b1;
                            if (alu_op == 3'b000 || alu_op == 3'b001 || alu_op == 3'b010 || alu_op == 3'b011) begin
                                update_carry_flag = 1'b1;
                            end
                            next_state = FETCH;
                        end
                        4'b1101: begin
                            case (instr[1:0])
                                2'b00: begin
                                    pc_load = 1'b1;
                                    pc_load_val = op_byte;
                                end
                                2'b01: begin
                                    if (carry_flag) begin
                                        pc_load = 1'b1;
                                        pc_load_val = op_byte;
                                    end
                                end
                                2'b10: begin
                                    if (zero_flag) begin
                                        pc_load = 1'b1;
                                        pc_load_val = op_byte;
                                    end
                                end
                                2'b11: begin
                                    if (!zero_flag) begin
                                        pc_load = 1'b1;
                                        pc_load_val = op_byte;
                                    end
                                end
                            endcase
                            next_state = FETCH;
                        end
                        default: begin
                            next_state = HALT;
                        end
                    endcase
                end
            end
            EXEC2: begin
                case (instr[7:4])
                    4'b1010: begin
                        alu_op = 3'b111;
                        alu_B = dmem_data_out;
                        alu_A = acc_out;
                        acc_load = 1'b1;
                        update_carry_flag = 1'b0;
                        update_zero_flag = 1'b0;
                        next_state = FETCH;
                    end
                    4'b1011: begin
                        dmem_addr = effective_addr;
                        dmem_data_in = acc_out;
                        dmem_we = 1'b1;
                        update_carry_flag = 1'b0;
                        update_zero_flag = 1'b0;
                        next_state = FETCH;
                    end
                    default: begin
                        next_state = HALT;
                    end
                endcase
            end
            HALT: begin
                done = 1'b1;
                next_state = HALT;
            end
            default: begin
                next_state = HALT;
            end
        endcase
    end

endmodule

module alu (
    input [2:0] opcode,
    input [7:0] A,
    input [7:0] op,
    input carry_in,
    input shift_en,
    input [1:0] shift_type,
    output reg [7:0] result,
    output reg carry_out,
    output zero
);

    wire [8:0] add_full = A + op;
    wire [8:0] sub_full = A - op;
    wire [8:0] adc_full = A + op + carry_in;
    wire [8:0] sbb_full = A - op - carry_in;

    always @(*) begin
        carry_out = 0;

        if (shift_en) begin
            case (shift_type)
                2'b00: result = {A[6:0], A[7]};
                2'b01: result = {A[0], A[7:1]};
                2'b10: begin result = {A[6:0], carry_in}; carry_out = A[7]; end
                2'b11: begin result = {carry_in, A[7:1]}; carry_out = A[0]; end
                default: result = A;
            endcase
        end else begin
            case (opcode)
                3'b000: begin result = add_full[7:0]; carry_out = add_full[8]; end
                3'b001: begin result = sub_full[7:0]; carry_out = sub_full[8]; end
                3'b010: begin result = adc_full[7:0]; carry_out = adc_full[8]; end
                3'b011: begin result = sbb_full[7:0]; carry_out = sbb_full[8]; end
                3'b100: result = A & op;
                3'b101: result = A | op;
                3'b110: result = A ^ op;
                3'b111: result = op;
                default: result = 8'd0;
            endcase
        end
    end

    assign zero = (result == 8'd0);

endmodule


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

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < 256; i = i + 1)
                mem[i] <= 8'd0;
        end else if (we) begin
            mem[addr] <= data_in;
        end
    end

    assign data_out = mem[addr];

endmodule


module dmem(
    input clk,
    input reset,
    input we,
    input [7:0] addr,
    input [7:0] data_in,
    output reg [7:0] data_out
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
            data_out <= mem[addr];
        end
    end
endmodule


module pc (
    input clk,
    input reset,
    input load,
    input inc,
    input [7:0] load_value,
    output reg [7:0] pc_out
);

    always @(posedge clk or posedge reset) begin
        if (reset)
            pc_out <= 8'd0;
        else if (load)
            pc_out <= load_value;
        else if (inc)
            pc_out <= pc_out + 1;
    end

endmodule


module acc(
    input clk,
    input reset,
    input load,
    input [7:0] data_in,
    output reg [7:0] acc_out
);

    always @(posedge clk or posedge reset) begin
        if (reset)
            acc_out <= 8'd0;
        else if (load)
            acc_out <= data_in;
    end

endmodule


module display_driver(
    input clk,
    input reset,
    input [15:0] binary_value,
    output reg [3:0] an_out,
    output [6:0] seg_out
);

    reg [19:0] refresh_counter;
    localparam REFRESH_RATE_DIVIDER = 1000;

    wire [1:0] digit_select;
    reg [3:0] current_digit_val;

    always @(posedge clk or posedge reset) begin
        if (reset)
            refresh_counter <= 20'd0;
        else
            refresh_counter <= refresh_counter + 1;
    end

    assign digit_select = refresh_counter[19:18];

    always @(*) begin
        case (digit_select)
            2'd3: begin
                an_out = 4'b0111;
                current_digit_val = binary_value[15:12];
            end
            2'd2: begin
                an_out = 4'b1011;
                current_digit_val = binary_value[11:8];
            end
            2'd1: begin
                an_out = 4'b1101;
                current_digit_val = binary_value[7:4];
            end
            2'd0: begin
                an_out = 4'b1110;
                current_digit_val = binary_value[3:0];
            end
            default: begin
                an_out = 4'b1111;
                current_digit_val = 4'b0000;
            end
        endcase
    end

    seven_seg_decoder decoder (
        .digit(current_digit_val),
        .seg(seg_out)
    );

endmodule

module seven_seg_decoder(
    input [3:0] digit,
    output reg [6:0] seg
);

    always @(*) begin
        case (digit)
            4'h0: seg = 7'b1000000;
            4'h1: seg = 7'b1111001;
            4'h2: seg = 7'b0100100;
            4'h3: seg = 7'b0110000;
            4'h4: seg = 7'b0011001;
            4'h5: seg = 7'b0010010;
            4'h6: seg = 7'b0000010;
            4'h7: seg = 7'b1111000;
            4'h8: seg = 7'b0000000;
            4'h9: seg = 7'b0010000;
            4'hA: seg = 7'b0001000;
            4'hB: seg = 7'b0000011;
            4'hC: seg = 7'b1000110;
            4'hD: seg = 7'b0100001;
            4'hE: seg = 7'b0000110;
            4'hF: seg = 7'b0001110;
            default: seg = 7'b1111111;
        endcase
    end
endmodule
