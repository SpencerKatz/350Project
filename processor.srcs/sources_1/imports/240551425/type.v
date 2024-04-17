module type(q_imem, rType, iType, jType, memInstruction, sw);

input wire [31:0] q_imem;
output wire rType, iType, jType, memInstruction, sw;

wire add, addi, sub, ands, ors, sll, sra, mul, div, j, bne, jal, jr, blt, lw;
wire opp31, opp30, opp29, opp28, opp27;

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

or OR0(iType, addi, bne, blt, sw, lw);

or OR1(jType, j, jal, jr);

or OR2(memInstruction, sw, lw);



endmodule