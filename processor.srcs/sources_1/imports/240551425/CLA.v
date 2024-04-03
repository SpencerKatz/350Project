module CLA(data_operandA, data_operandB, sum, overflow, Cin);

        input [31:0] data_operandA, data_operandB;
        input Cin;
        output [31:0] sum;
        output overflow;

        wire w1, w2,w3,w4,w5,w6,w7,w8,w9,w10,w11,w12,w13,w14, w15, w16, w17, c7;
        wire P0,P1,P2,P3,G0,G1,G2,G3;
        
        CLA_block cla1(data_operandA[7:0], data_operandB[7:0], Cin, P0, G0, sum[7:0], w15);
        and AND1(w1, P0, Cin);
        or OR1(w2, G0, w1);
        CLA_block cla2(data_operandA[15:8], data_operandB[15:8], w2, P1, G1, sum[15:8], w16);

        and AND2(w3, P1, G0);
        and AND3(w4, P1,P0,Cin);
        or OR2(w5, G1, w3, w4);
        
        CLA_block cla3(data_operandA[23:16], data_operandB[23:16], w5, P2, G2, sum[23:16], w17);

        and AND4(w6,P2,G1);
        and AND5(w7, P2,P1,G0);
        and AND6(w8, P2,P1,P0,Cin);
        or OR3(w9,G2,w6,w7,w8);

        CLA_block cla4(data_operandA[31:24], data_operandB[31:24], w9, P3, G3, sum[31:24], c7);

        and AND7(w10,P3,G2);
        and AND8(w11,P3,P2,G1);
        and AND9(w12,P3,P2,P1,G0);
        and AND10(w13,P3,P2,P1,P0,Cin);
        or OR4(w14,G3,w10,w11,w12,w13);

        xor XOR1(overflow,c7,w14);
        
endmodule
        