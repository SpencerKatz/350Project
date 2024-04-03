module shift_right_1(data_operandA, shifted);
    input [31:0] data_operandA;
    output [31:0] shifted;

    assign shifted[0]  = data_operandA[1];
    assign shifted[1]  = data_operandA[2];
    assign shifted[2]  = data_operandA[3];
    assign shifted[3]  = data_operandA[4];
    assign shifted[4]  = data_operandA[5];
    assign shifted[5]  = data_operandA[6];
    assign shifted[6]  = data_operandA[7];
    assign shifted[7]  = data_operandA[8];
    assign shifted[8]  = data_operandA[9];
    assign shifted[9]  = data_operandA[10];
    assign shifted[10] = data_operandA[11];
    assign shifted[11] = data_operandA[12];
    assign shifted[12] = data_operandA[13];
    assign shifted[13] = data_operandA[14];
    assign shifted[14] = data_operandA[15];
    assign shifted[15] = data_operandA[16];
    assign shifted[16] = data_operandA[17];
    assign shifted[17] = data_operandA[18];
    assign shifted[18] = data_operandA[19];
    assign shifted[19] = data_operandA[20];
    assign shifted[20] = data_operandA[21];
    assign shifted[21] = data_operandA[22];
    assign shifted[22] = data_operandA[23];
    assign shifted[23] = data_operandA[24];
    assign shifted[24] = data_operandA[25];
    assign shifted[25] = data_operandA[26];
    assign shifted[26] = data_operandA[27];
    assign shifted[27] = data_operandA[28];
    assign shifted[28] = data_operandA[29];
    assign shifted[29] = data_operandA[30];
    assign shifted[30] = data_operandA[31];
    assign shifted[31] = data_operandA[31];

endmodule