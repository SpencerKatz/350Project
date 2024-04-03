module Decode(clock, q_imem, ctrl_readRegA, ctrl_readRegB);

input clock;
input [31:0] q_imem;

output [4:0] ctrl_readRegA, ctrl_readRegB;

wire rType, iType, jType, memInstruction;
wire notUsed;

wire add, sub, addi, mul, div, j, bne, jal, jr, blt, sw, lw, bex, setx;
wire branchJ;
wire [4:0] inter, inter2;
wire [4:0] inter3;



completeType c1(q_imem, add, sub, addi, mul, div, j, bne, jal, jr, blt, sw, lw, bex, setx);

or OR1(branchJ, bne, blt, jr);

// assign ctrl_readRegA = branchJ ? q_imem[26:22] : q_imem[21:17];

assign inter3 = branchJ ? q_imem[26:22] : q_imem[21:17];

assign ctrl_readRegA = bex ? 5'b11110 : inter3;
// 

type TYPE0(q_imem, rType, iType, jType, memInstruction, notUsed);

// assign inter = branchJ ? q_imem[21:17] : q_imem[16:12];
// assign ctrl_readRegB = memInstruction ? q_imem[26:22] : inter;

assign inter = branchJ ? q_imem[21:17] : q_imem[16:12];
assign inter2 = memInstruction ? q_imem[26:22] : inter;
assign ctrl_readRegB = bex ? 5'b0 : inter2;



endmodule

// assign inter3 = branchJ ? q_imem[26:22] : q_imem[21:17];

// assign ctrl_readRegA = bex ? 5'b11110 : inter3;



// assign inter = branchJ ? q_imem[21:17] : q_imem[16:12];
// assign inter2 = memInstruction ? q_imem[26:22] : inter;
// assign ctrl_readRegB = bex ? 5'b0 : inter2;