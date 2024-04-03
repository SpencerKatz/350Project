module CLA_block(data_operandA, data_operandB, Cin, Pout, Gout, Sum, csecondlast);
    input [7:0] data_operandA, data_operandB;
    input Cin;
    output [7:0] Sum;
    output Pout,Gout;
    output csecondlast;

    wire g0,g1,g2,g3,g4,g5,g6,g7;
    wire p0,p1,p2,p3,p4,p5,p6,p7;
    wire c1, c2, c3, c4, c5, c6, c7;
    wire w1, w2, w3, w4, w5, w6, w7, w8, w9, w10;
    wire w11, w12, w13, w14, w15, w16, w17, w18, w19, w20;
    wire w21, w22, w23, w24, w25, w26, w27, w28, w29, w30;
    wire w31, w32, w33, w34, w35;

    ALU_adder add0(Sum[0],data_operandA[0], data_operandB[0], Cin);
    // first cin
    and AND0(g0, data_operandA[0], data_operandB[0]);
    or OR0(p0, data_operandA[0], data_operandB[0]); 
    and AND1(w1, p0, Cin);
    or OR1(c1, w1, g0);
    ALU_adder add1(Sum[1],data_operandA[1], data_operandB[1], c1);
    // second cin
    and AND2(g1, data_operandA[1], data_operandB[1]);
    or OR2(p1, data_operandA[1], data_operandB[1]); 
    and AND3(w2, p1,g0);
    and AND4(w3, p1,p0,Cin);
    or OR3(c2,w2,w3,g1);
    ALU_adder add2(Sum[2],data_operandA[2], data_operandB[2], c2);
    // third cin
    and AND5(g2, data_operandA[2], data_operandB[2]);
    or OR4(p2, data_operandA[2], data_operandB[2]);
    and AND6(w4, p2, g1);
    and AND7(w5, p2,p1,g0);
    and AND8(w6,p2,p1,p0,Cin);
    or OR5(c3, g2,w4,w5,w6);
    ALU_adder add3(Sum[3],data_operandA[3], data_operandB[3], c3);
    // fourth cin
    and AND9(g3, data_operandA[3], data_operandB[3]);
    or OR6(p3, data_operandA[3], data_operandB[3]);
    and AND10(w7,p3,g2);
    and AND11(w8,p3,p2,g1);
    and AND12(w9,p3,p2,p1,g0);
    and AND13(w10,p3,p2,p1,p0,Cin);
    or OR7(c4, g3, w7, w8, w9, w10);
    ALU_adder add4(Sum[4],data_operandA[4], data_operandB[4], c4);
    // fifth cin
    and AND14(g4, data_operandA[4], data_operandB[4]);
    or OR8(p4, data_operandA[4], data_operandB[4]);
    and AND15(w11,p4,g3);
    and AND16(w12, p4, p3, g2);
    and AND17(w13, p4, p3, p2, g1);
    and AND18(w14, p4, p3, p2, p1, g0);
    and AND19(w15, p4,p3,p2,p1,p0,Cin);
    or OR9(c5, g4, w11, w12, w13, w14, w15);
    ALU_adder add5(Sum[5],data_operandA[5], data_operandB[5], c5);
    // sixth cin
    and AND20(g5, data_operandA[5], data_operandB[5]);
    or OR10(p5, data_operandA[5], data_operandB[5]);
    and AND21(w16,p5,g4);
    and AND22(w17,p5,p4,g3);
    and AND23(w18,p5,p4,p3,g2);
    and AND24(w19,p5,p4,p3,p2,g1);
    and AND25(w20,p5,p4,p3,p2,p1,g0);
    and AND26(w21,p5,p4,p3,p2,p1,p0,Cin);
    or OR11(c6,g5,w16,w17,w18,w19,w20,w21);
    ALU_adder add6(Sum[6],data_operandA[6], data_operandB[6], c6);
    // seventh cin
    and AND27(g6, data_operandA[6], data_operandB[6]);
    or OR12(p6, data_operandA[6], data_operandB[6]);
    and AND28(w22, p6, g5);
    and AND29(w23, p6, p5, g4);
    and AND30(w24, p6, p5, p4, g3);
    and AND31(w25, p6, p5, p4, p3, g2);
    and AND32(w26, p6, p5, p4, p3, p2, g1);
    and AND33(w27, p6, p5, p4, p3, p2, p1, g0);
    and AND34(w28, p6, p5, p4, p3, p2, p1, p0, Cin);
    or OR13(c7, g6, w22, w23, w24, w25, w26, w27, w28);
    ALU_adder add7(Sum[7],data_operandA[7], data_operandB[7], c7);
    assign csecondlast = c7;
    // end
    and AND35(g7, data_operandA[7], data_operandB[7]);
    or OR14(p7, data_operandA[7], data_operandB[7]);
    and AND36(Pout,p7,p6,p5,p4,p3,p2,p1,p0);
    and AND37(w29, p7, g6);
    and AND38(w30, p7, p6, g5);
    and AND39(w31, p7, p6, p5, g4);
    and AND40(w32, p7, p6, p5, p4, g3);
    and AND41(w33, p7, p6, p5, p4, p3, g2);
    and AND42(w34, p7, p6, p5, p4, p3, p2, g1);
    and AND43(w35, p7, p6, p5, p4, p3, p2, p1, g0);
    or OR15(Gout,g7,w29,w30,w31,w32,w33,w34,w35);
endmodule