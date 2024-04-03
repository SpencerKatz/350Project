module bypassing(FDInstruction, DXInstruction, XMInstruction, 
MWInstruction, rs1, rs2, oppOut, data_writeReg, saveB, useRS1,
useRS2, useMem, hazardHere);

input [31:0] FDInstruction, DXInstruction, XMInstruction, MWInstruction;
input [31:0] rs1, rs2, oppOut, data_writeReg, saveB;

output [31:0] useRS1, useRS2, useMem;
output hazardHere;

wire mx1, wx2, ALUInBmx, ALUInBwx, wm;

hazard h1(FDInstruction, DXInstruction, XMInstruction, 
MWInstruction, mx1, wx2, ALUInBmx, ALUInBwx, wm, hazardHere);

assign useRS1 = mx1 ? oppOut : wx2 ? data_writeReg : rs1;

assign useRS2 = ALUInBmx ? oppOut : ALUInBwx ? data_writeReg : rs2;

assign useMem = wm ? data_writeReg : saveB;


endmodule