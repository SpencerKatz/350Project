module Execute(clock, reset, rs1, rs2, opCode, result, extend, lessThan, notEqual, over);

input clock, reset;
input [31:0] rs1, rs2, opCode;
output [31:0] result;
output lessThan, notEqual;

wire rType, iType, jType, memInstruction;
wire [31:0] useR1, useR2;
output [31:0] extend;
wire [4:0] useMe;
wire [4:0] shift;
wire[31:0] result;
wire dontCare;
wire [31:0] dontCare2;
wire add, sub, addi, mul, div, j, bne, jal, jr, blt, sw, lw, bex, setx;

wire notUsed;
wire overflow;
output over;

completeType c1(opCode, add, sub, addi, mul, div, j, bne, jal, jr, blt, sw, lw, bex, setx);

assign extend[16:0] = opCode[16:0];
assign extend[31:17] = opCode[16] ? 15'b111111111111111 : 15'b0;

or OR0(iType, addi, sw, lw);

assign useR1 = rs1;

assign useR2 = iType ? extend : rs2;

assign useMe = iType ? 5'b0 : opCode[6:2];

assign shift = opCode[11:7];

alu ALU(useR1, useR2, useMe, shift, result, dontCare, dontCare, over);

alu ALU2(useR1, useR2, 5'b00001, dontCare, dontCare2, notEqual, lessThan, overflow);





endmodule