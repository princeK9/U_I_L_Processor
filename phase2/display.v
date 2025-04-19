module display_driver(
    input clk,
    input reset,
    input [7:0] binary_value,  
    output [3:0] an_out,   
    output [6:0] seg_out   
);

    reg [3:0] an_out_reg;
    wire [6:0] seg_out_reg;  
    reg [19:0] refresh_counter;
    wire digit_select;
    reg [3:0] current_digit;

    always @(posedge clk or posedge reset) begin
        if (reset)
            refresh_counter <= 0;
        else
            refresh_counter <= refresh_counter + 1;
    end

    assign digit_select = refresh_counter[17]; 

    always @(*) begin
        case (digit_select)
            1'b0: begin
                an_out_reg = 4'b1110;  
                current_digit = binary_value[3:0];  
            end
            1'b1: begin
                an_out_reg = 4'b1101;  
                current_digit = binary_value[7:4];  
            end
            default: begin
                an_out_reg = 4'b1111;
                current_digit = 4'b0000;
            end
        endcase
    end

    seven_seg_binary_decoder decoder(
        .digit(current_digit),
        .seg(seg_out_reg)
    );

    assign an_out  = an_out_reg;
    assign seg_out = seg_out_reg;  

endmodule



module seven_seg_binary_decoder(
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
