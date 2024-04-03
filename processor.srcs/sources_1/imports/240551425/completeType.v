module completeType(q_imem, add, sub, addi, mul, div, j, bne, jal, jr, blt, sw, lw, bex, setx);

input [31:0] q_imem;
output add, sub, addi, mul, div, j, bne, jal, jr, blt, sw, lw, bex, setx;

wire opp31, opp30, opp29, opp28, opp27;
wire opp6, opp5, opp4, opp3, opp2, opp1;
wire rType, iType, jType, memInstruction;

not NOT0(opp31, q_imem[31]);
not NOT1(opp30, q_imem[30]);
not NOT2(opp29, q_imem[29]);
not NOT3(opp28, q_imem[28]);
not NOT4(opp27, q_imem[27]);

and AND1(rType, opp31, opp30, opp29, opp28, opp27);

and AND2(addi, opp31, opp30, q_imem[29], opp28, q_imem[27]);

and AND3(j, opp31, opp30, opp29, opp28, q_imem[27]);

and AND4(bne, opp31, opp30, opp29, q_imem[28], opp27);

and AND5(jal, opp31, opp30, opp29, q_imem[28], q_imem[27]);

and AND6(jr, opp31, opp30, q_imem[29], opp28, opp27);

and AND7(blt, opp31, opp30, q_imem[29], q_imem[28], opp27);

and AND8(sw, opp31, opp30, q_imem[29],q_imem[28],q_imem[27]);

and AND9(lw, opp31, q_imem[30], opp29, opp28, opp27);

and AND12(bex, q_imem[31], opp30, q_imem[29], q_imem[28], opp27);

and AND13(setx, q_imem[31], opp30, q_imem[29], opp28, q_imem[27]);


or OR0(iType, addi, bne, blt, sw, lw);

or OR1(jType, j, jal, jr);

or OR2(memInstruction, sw, lw);

not NOT5(opp6, q_imem[6]);
not NOT6(opp5, q_imem[5]);
not NOT7(opp4, q_imem[4]);
not NOT8(opp3, q_imem[3]);
not NOT9(opp2, q_imem[2]);

and AND10(mul, rType, opp6, opp5, q_imem[4], q_imem[3], opp2);

and AND11(div, rType, opp6, opp5, q_imem[4], q_imem[3], q_imem[2]);

and AND14(add, rType, opp6, opp5, opp4, opp3, opp2);

and AND15(sub, rType, opp6, opp5, opp4, opp3, q_imem[2]);


endmodule