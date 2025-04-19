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
                carry_flag <=  alu_cout;
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