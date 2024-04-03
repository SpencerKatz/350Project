module Memory(clock, reset, opCode, writeEnable, lw);

input clock, reset;
input [31:0] opCode;

output writeEnable, lw;


wire opp31, opp30, opp29, opp28, opp27;
wire lw, sw;

not NOT0(opp31, opCode[31]);
not NOT1(opp30, opCode[30]);
not NOT2(opp29, opCode[29]);
not NOT3(opp28, opCode[28]);
not NOT4(opp27, opCode[27]);

and AND8(sw, opp31, opp30, opCode[29], opCode[28], opCode[27]);

and AND9(lw, opp31, opCode[30], opp29, opp28, opp27);

assign writeEnable = sw;



endmodule