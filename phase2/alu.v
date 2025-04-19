// module alu (
//     input  [2:0] opcode,
//     input  [7:0] A,
//     input  [7:0] op,
//     input        carry_in,
//     input        shift_en,
//     input  [1:0] shift_type,
//     output reg [7:0] result,
//     output reg       carry_out,
//     output           zero
// );

//     wire [8:0] add_full  = {1'b0, A} + {1'b0, op};
//     wire [8:0] adc_full  = {1'b0, A} + {1'b0, op} + {8'd0, carry_in};
//     wire [8:0] sub_full  = {1'b0, A} + {1'b0, ~op} + 9'd1;
//     wire [8:0] sbb_full  = {1'b0, A} + {1'b0, ~op} + {1'b0, ~carry_in} + 9'd1;

//     assign zero = (result == 8'd0);

//     always @(*) begin
//         if (shift_en) begin
//             carry_out = 1'b0;
//             case (shift_type)
//                 2'b00: begin
//                     result    = {A[6:0], A[7]};
//                     carry_out = A[7];
//                 end
//                 2'b01: begin
//                     result    = {A[0], A[7:1]};
//                     carry_out = A[0];
//                 end
//                 2'b10: begin
//                     result    = {A[6:0], carry_in};
//                     carry_out = A[7];
//                 end
//                 2'b11: begin
//                     result    = {carry_in, A[7:1]};
//                     carry_out = A[0];
//                 end
//             endcase
//         end else begin
//             carry_out = 1'b0;
//             case (opcode)
//                 3'b000: begin
//                     result    = add_full[7:0];
//                     carry_out = add_full[8];
//                 end
//                 3'b001: begin
//                     result    = sub_full[7:0];
//                     carry_out = sub_full[8];
//                 end
//                 3'b010: begin
//                     result    = adc_full[7:0];
//                     carry_out = adc_full[8];
//                 end
//                 3'b011: begin
//                     result    = sbb_full[7:0];
//                     carry_out = sbb_full[8];
//                 end
//                 3'b100: begin
//                     result    = A & op;
//                     carry_out = 1'b0;
//                 end
//                 3'b101: begin
//                     result    = A | op;
//                     carry_out = 1'b0;
//                 end
//                 3'b110: begin
//                     result    = A ^ op;
//                     carry_out = 1'b0;
//                 end
//                 3'b111: begin
//                     result    = op;
//                     carry_out = carry_in;
//                 end
//                 default: begin
//                     result    = 8'd0;
//                     carry_out = 1'b0;
//                 end
//             endcase
//         end
//     end
// endmodule

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