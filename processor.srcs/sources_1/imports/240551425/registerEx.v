module registerEx(ctrl_writeEnable, ctrl_reset, clock, data_writeReg, read);

input clock, ctrl_writeEnable, ctrl_reset;
input [31:0] data_writeReg;
output [31:0] read;

dffe_ref zero(read[0], data_writeReg[0], clock, ctrl_writeEnable, ctrl_reset);
dffe_ref first(read[1], data_writeReg[1], clock, ctrl_writeEnable, ctrl_reset);
dffe_ref second(read[2], data_writeReg[2], clock, ctrl_writeEnable, ctrl_reset);
dffe_ref third(read[3], data_writeReg[3], clock, ctrl_writeEnable, ctrl_reset);
dffe_ref fourth(read[4], data_writeReg[4], clock, ctrl_writeEnable, ctrl_reset);
dffe_ref fifth(read[5], data_writeReg[5], clock, ctrl_writeEnable, ctrl_reset);
dffe_ref sixth(read[6], data_writeReg[6], clock, ctrl_writeEnable, ctrl_reset);
dffe_ref seventh(read[7], data_writeReg[7], clock, ctrl_writeEnable, ctrl_reset);
dffe_ref eighth(read[8], data_writeReg[8], clock, ctrl_writeEnable, ctrl_reset);
dffe_ref ninth(read[9], data_writeReg[9], clock, ctrl_writeEnable, ctrl_reset);
dffe_ref tenth(read[10], data_writeReg[10], clock, ctrl_writeEnable, ctrl_reset);
dffe_ref eleventh(read[11], data_writeReg[11], clock, ctrl_writeEnable, ctrl_reset);
dffe_ref twelfth(read[12], data_writeReg[12], clock, ctrl_writeEnable, ctrl_reset);
dffe_ref thierteenth(read[13], data_writeReg[13], clock, ctrl_writeEnable, ctrl_reset);
dffe_ref fourteenth(read[14], data_writeReg[14], clock, ctrl_writeEnable, ctrl_reset);
dffe_ref fifteenth(read[15], data_writeReg[15], clock, ctrl_writeEnable, ctrl_reset);
dffe_ref sixteenth(read[16], data_writeReg[16], clock, ctrl_writeEnable, ctrl_reset);
dffe_ref seventeenth(read[17], data_writeReg[17], clock, ctrl_writeEnable, ctrl_reset);
dffe_ref eighteenth(read[18], data_writeReg[18], clock, ctrl_writeEnable, ctrl_reset);
dffe_ref nineteenth(read[19], data_writeReg[19], clock, ctrl_writeEnable, ctrl_reset);
dffe_ref twenty(read[20], data_writeReg[20], clock, ctrl_writeEnable, ctrl_reset);
dffe_ref twentyone(read[21], data_writeReg[21], clock, ctrl_writeEnable, ctrl_reset);
dffe_ref twentytwo(read[22], data_writeReg[22], clock, ctrl_writeEnable, ctrl_reset);
dffe_ref twentythree(read[23], data_writeReg[23], clock, ctrl_writeEnable, ctrl_reset);
dffe_ref twentyfour(read[24], data_writeReg[24], clock, ctrl_writeEnable, ctrl_reset);
dffe_ref twentyfive(read[25], data_writeReg[25], clock, ctrl_writeEnable, ctrl_reset);
dffe_ref twentysix(read[26], data_writeReg[26], clock, ctrl_writeEnable, ctrl_reset);
dffe_ref twentyseven(read[27], data_writeReg[27], clock, ctrl_writeEnable, ctrl_reset);
dffe_ref twentyeight(read[28], data_writeReg[28], clock, ctrl_writeEnable, ctrl_reset);
dffe_ref twentynine(read[29], data_writeReg[29], clock, ctrl_writeEnable, ctrl_reset);
dffe_ref thirty(read[30], data_writeReg[30], clock, ctrl_writeEnable, ctrl_reset);
dffe_ref thirtyone(read[31], data_writeReg[31], clock, ctrl_writeEnable, ctrl_reset);



endmodule