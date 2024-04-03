module barrel_shifter_right(data_operandA, ctrl_shiftamt, shifted);
    input [31:0] data_operandA;
    input [4:0] ctrl_shiftamt;
    output [31:0] shifted;

    wire [31:0] w1, w2, w3, w4, w5, w6, w7, w8, w9;

    shift_right_16 sixteen(data_operandA, w1);
    mux_2 first(w2, ctrl_shiftamt[4], data_operandA, w1);

    shift_right_8 eight(w2, w3);
    mux_2 second(w4, ctrl_shiftamt[3], w2, w3);

    shift_right_4 four(w4, w5);
    mux_2 third(w6, ctrl_shiftamt[2], w4, w5);

    shift_right_2 two(w6, w7);
    mux_2 fourth(w8, ctrl_shiftamt[1], w6, w7);

    shift_right_1 one(w8, w9);
    mux_2 fifth(shifted, ctrl_shiftamt[0], w8, w9);

endmodule