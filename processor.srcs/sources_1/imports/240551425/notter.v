module notter(data_operandB, notted);
    input [31:0] data_operandB;
    output [31:0] notted;

    not NOT0(notted[0], data_operandB[0]);
    not NOT1(notted[1], data_operandB[1]);
    not NOT2(notted[2], data_operandB[2]);
    not NOT3(notted[3], data_operandB[3]);
    not NOT4(notted[4], data_operandB[4]);
    not NOT5(notted[5], data_operandB[5]);
    not NOT6(notted[6], data_operandB[6]);
    not NOT7(notted[7], data_operandB[7]);
    not NOT8(notted[8], data_operandB[8]);
    not NOT9(notted[9], data_operandB[9]);
    not NOT10(notted[10], data_operandB[10]);
    not NOT11(notted[11], data_operandB[11]);
    not NOT12(notted[12], data_operandB[12]);
    not NOT13(notted[13], data_operandB[13]);
    not NOT14(notted[14], data_operandB[14]);
    not NOT15(notted[15], data_operandB[15]);
    not NOT16(notted[16], data_operandB[16]);
    not NOT17(notted[17], data_operandB[17]);
    not NOT18(notted[18], data_operandB[18]);
    not NOT19(notted[19], data_operandB[19]);
    not NOT20(notted[20], data_operandB[20]);
    not NOT21(notted[21], data_operandB[21]);
    not NOT22(notted[22], data_operandB[22]);
    not NOT23(notted[23], data_operandB[23]);
    not NOT24(notted[24], data_operandB[24]);
    not NOT25(notted[25], data_operandB[25]);
    not NOT26(notted[26], data_operandB[26]);
    not NOT27(notted[27], data_operandB[27]);
    not NOT28(notted[28], data_operandB[28]);
    not NOT29(notted[29], data_operandB[29]);
    not NOT30(notted[30], data_operandB[30]);
    not NOT31(notted[31], data_operandB[31]);

endmodule