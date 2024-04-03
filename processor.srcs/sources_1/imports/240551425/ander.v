module ander(data_operandA, data_operandB, anded);

        input [31:0] data_operandA, data_operandB;
        output [31:0] anded;

        and AND0(anded[0], data_operandA[0], data_operandB[0]);
        and AND1(anded[1], data_operandA[1], data_operandB[1]);
        and AND2(anded[2], data_operandA[2], data_operandB[2]);
        and AND3(anded[3], data_operandA[3], data_operandB[3]);
        and AND4(anded[4], data_operandA[4], data_operandB[4]);
        and AND5(anded[5], data_operandA[5], data_operandB[5]);
        and AND6(anded[6], data_operandA[6], data_operandB[6]);
        and AND7(anded[7], data_operandA[7], data_operandB[7]);
        and AND8(anded[8], data_operandA[8], data_operandB[8]);
        and AND9(anded[9], data_operandA[9], data_operandB[9]);
        and AND10(anded[10], data_operandA[10], data_operandB[10]);
        and AND11(anded[11], data_operandA[11], data_operandB[11]);
        and AND12(anded[12], data_operandA[12], data_operandB[12]);
        and AND13(anded[13], data_operandA[13], data_operandB[13]);
        and AND14(anded[14], data_operandA[14], data_operandB[14]);
        and AND15(anded[15], data_operandA[15], data_operandB[15]);
        and AND16(anded[16], data_operandA[16], data_operandB[16]);
        and AND17(anded[17], data_operandA[17], data_operandB[17]);
        and AND18(anded[18], data_operandA[18], data_operandB[18]);
        and AND19(anded[19], data_operandA[19], data_operandB[19]);
        and AND20(anded[20], data_operandA[20], data_operandB[20]);
        and AND21(anded[21], data_operandA[21], data_operandB[21]);
        and AND22(anded[22], data_operandA[22], data_operandB[22]);
        and AND23(anded[23], data_operandA[23], data_operandB[23]);
        and AND24(anded[24], data_operandA[24], data_operandB[24]);
        and AND25(anded[25], data_operandA[25], data_operandB[25]);
        and AND26(anded[26], data_operandA[26], data_operandB[26]);
        and AND27(anded[27], data_operandA[27], data_operandB[27]);
        and AND28(anded[28], data_operandA[28], data_operandB[28]);
        and AND29(anded[29], data_operandA[29], data_operandB[29]);
        and AND30(anded[30], data_operandA[30], data_operandB[30]);
        and AND31(anded[31], data_operandA[31], data_operandB[31]);

        endmodule