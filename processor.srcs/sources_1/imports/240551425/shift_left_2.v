module shift_left_2(data_operandA, shifted);
    input [31:0] data_operandA;
    output [31:0] shifted;

    wire [31:0] w1;
    shift_left_1 first(data_operandA, w1);
    shift_left_1 second(w1, shifted);
    

endmodule