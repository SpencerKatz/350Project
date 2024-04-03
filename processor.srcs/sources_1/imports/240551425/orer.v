    module orer(data_operandA, data_operandB, ored);

        input [31:0] data_operandA, data_operandB;
        output [31:0] ored;

        or OR0(ored[0], data_operandA[0], data_operandB[0]);
        or OR1(ored[1], data_operandA[1], data_operandB[1]);
        or OR2(ored[2], data_operandA[2], data_operandB[2]);
        or OR3(ored[3], data_operandA[3], data_operandB[3]);
        or OR4(ored[4], data_operandA[4], data_operandB[4]);
        or OR5(ored[5], data_operandA[5], data_operandB[5]);
        or OR6(ored[6], data_operandA[6], data_operandB[6]);
        or OR7(ored[7], data_operandA[7], data_operandB[7]);
        or OR8(ored[8], data_operandA[8], data_operandB[8]);
        or OR9(ored[9], data_operandA[9], data_operandB[9]);
        or OR10(ored[10], data_operandA[10], data_operandB[10]);
        or OR11(ored[11], data_operandA[11], data_operandB[11]);
        or OR12(ored[12], data_operandA[12], data_operandB[12]);
        or OR13(ored[13], data_operandA[13], data_operandB[13]);
        or OR14(ored[14], data_operandA[14], data_operandB[14]);
        or OR15(ored[15], data_operandA[15], data_operandB[15]);
        or OR16(ored[16], data_operandA[16], data_operandB[16]);
        or OR17(ored[17], data_operandA[17], data_operandB[17]);
        or OR18(ored[18], data_operandA[18], data_operandB[18]);
        or OR19(ored[19], data_operandA[19], data_operandB[19]);
        or OR20(ored[20], data_operandA[20], data_operandB[20]);
        or OR21(ored[21], data_operandA[21], data_operandB[21]);
        or OR22(ored[22], data_operandA[22], data_operandB[22]);
        or OR23(ored[23], data_operandA[23], data_operandB[23]);
        or OR24(ored[24], data_operandA[24], data_operandB[24]);
        or OR25(ored[25], data_operandA[25], data_operandB[25]);
        or OR26(ored[26], data_operandA[26], data_operandB[26]);
        or OR27(ored[27], data_operandA[27], data_operandB[27]);
        or OR28(ored[28], data_operandA[28], data_operandB[28]);
        or OR29(ored[29], data_operandA[29], data_operandB[29]);
        or OR30(ored[30], data_operandA[30], data_operandB[30]);
        or OR31(ored[31], data_operandA[31], data_operandB[31]);

    endmodule