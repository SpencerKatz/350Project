module hazard(FDInstruction, DXInstruction, XMInstruction, 
MWInstruction, mx1, wx2, ALUInBmx, ALUInBwx, wm, hazardHere);

input [31:0] FDInstruction, DXInstruction, XMInstruction, MWInstruction;
output mx1;
output wx2;
output ALUInBmx;
output ALUInBwx;
output wm;
output hazardHere;

wire rType1, rType2, rType3, rType4;
wire iType1, iType2, iType3, iType4;
wire jType1, jType2, jType3, jType4;
wire memInstruction1, memInstruction2, memInstruction3, memInstruction4;
wire sw1, sw2, sw3, sw4;
wire ri1, ri2, ri3, ri4;

type TYPE1(FDInstruction, rType1, iType1, jType1, memInstruction1, sw1);
type TYPE2(DXInstruction, rType2, iType2, jType2, memInstruction2, sw2);
type TYPE3(XMInstruction, rType3, iType3, jType3, memInstruction3, sw3);
type TYPE4(MWInstruction, rType4, iType4, jType4, memInstruction4, sw4);

or OR1(ri1, rType1, iType1);
or OR2(ri2, rType2, iType2);
or OR3(ri3, rType3, iType3);
or OR4(ri4, rType4, iType4);


// module type(q_imem, rType, iType, jType, memInstruction, sw);
wire mx1Inter, wx2Inter, ALUInter1, ALUInter2, wmInter;

wire notsw1;
wire notsw2;
not NOT0(notsw1, storeXM);
not NOT1(notsw2, storeMW);


assign mx1Inter = (!branch && ((DXInstruction[21:17] == XMInstruction[26:22]) && (DXInstruction[21:17] != 5'b0)) || (jr && (DXInstruction[26:22] == XMInstruction[26:22]))) 
|| (branch && (DXInstruction[26:22] == XMInstruction[26:22]));
and AND0(mx1, mx1Inter, (ri2 || jr), ri3 && !branch2, notsw1);

assign wx2Inter = (!branch && ((DXInstruction[21:17] == MWInstruction[26:22]) && (DXInstruction[21:17] != 5'b0)) || (jr && (DXInstruction[26:22] == MWInstruction[26:22])))
|| (branch && (DXInstruction[26:22] == MWInstruction[26:22]));

and AND1(wx2, wx2Inter, (ri2 || jr), ri4 && !branch3, notsw2);

assign ALUInter1 = (!branch && ((DXInstruction[16:12] == XMInstruction[26:22]) && (DXInstruction[16:12] != 5'b0))) || (branch && (!storeXM) && DXInstruction[21:17] != 5'b0 && (DXInstruction[21:17] == XMInstruction[26:22]));
and AND2(ALUInBmx, ALUInter1, branch || rType2, ri3 && !branch2);

assign ALUInter2 = (!branch && ((DXInstruction[16:12] == MWInstruction[26:22]) && (DXInstruction[16:12] != 5'b0))) || (branch && (!storeMW) && DXInstruction[21:17] != 5'b0 && (DXInstruction[21:17] == MWInstruction[26:22]));
and AND3(ALUInBwx, ALUInter2, branch || rType2, ri4 && !branch3);
// load and stores
wire [31:0] oppXM, oppMW;
wire loadXM, storeXM, loadMW, storeMW;

assign oppXM = ~XMInstruction;
assign oppMW = ~MWInstruction;

and AND4(loadXM, oppXM[31], XMInstruction[30], oppXM[29], oppXM[28], oppXM[27]);
and AND5(loadMW, oppMW[31], MWInstruction[30], oppMW[29], oppMW[28], oppMW[27]);

and AND6(storeXM,oppXM[31], oppXM[30], XMInstruction[29], XMInstruction[28], XMInstruction[27]);
and AND7(storeMW,oppMW[31], oppMW[30], MWInstruction[29], MWInstruction[28], MWInstruction[27]);
//

assign wmInter = (XMInstruction[26:22] == MWInstruction[26:22]) && (XMInstruction[26:22] != 5'b0);
and AND8(wm, wmInter, storeXM, ri4);

// more loads and stores
wire[31:0] oppDX, oppFD;
wire loadFD, storeFD, loadDX, storeDX;


assign oppFD = ~FDInstruction;
assign oppDX = ~DXInstruction;

and AND9(loadFD, oppFD[31], FDInstruction[30], oppFD[29], oppFD[28], oppFD[27]);
and AND10(loadDX, oppDX[31], DXInstruction[30], oppDX[29], oppDX[28], oppDX[27]);

and AND11(storeFD, oppFD[31], oppFD[30], FDInstruction[29], FDInstruction[28], FDInstruction[27]);
and AND12(storeDX, oppDX[31], oppDX[30], DXInstruction[29], DXInstruction[28], DXInstruction[27]);
//

assign hazardHere = loadDX && (((FDInstruction[21:17] == DXInstruction[26:22]) 
&& ri1) || ((FDInstruction[16:12] == DXInstruction[26:22]) && rType1) || ((FDInstruction[26:22] == DXInstruction[26:22]) && (branch0 
|| jr0))) && !storeFD;

// jr 
wire jr;

and AND13(jr, oppDX[31], oppDX[30], DXInstruction[29], oppDX[28], oppDX[27]);

wire jr0;
and AND22(jr0, oppFD[31], oppFD[30], FDInstruction[29], oppFD[28], oppFD[27]);

// branch
wire branch;
wire blt, bne;

and AND14(blt, oppDX[31], oppDX[30], DXInstruction[29], DXInstruction[28], oppDX[27]);

and AND15(bne, oppDX[31], oppDX[30], oppDX[29], DXInstruction[28], oppDX[27]);

or OR5(branch, blt, bne);

wire branch2;
wire blt2, bne2;

and AND16(blt2, oppXM[31], oppXM[30], XMInstruction[29], XMInstruction[28], oppXM[27]);

and AND17(bne2, oppXM[31], oppXM[30], oppXM[29], XMInstruction[28], oppXM[27]);

or OR6(branch2, blt2, bne2);

wire branch3;
wire blt3, bne3;

and AND18(blt3, oppMW[31], oppMW[30], MWInstruction[29], MWInstruction[28], oppMW[27]);

and AND19(bne3, oppMW[31], oppMW[30], oppMW[29], MWInstruction[28], oppMW[27]);

or OR7(branch3, blt3, bne3);

wire branch0;
wire blt0, bne0;

and AND20(blt0, oppFD[31], oppFD[30], FDInstruction[29], FDInstruction[28], oppFD[27]);

and AND21(bne0, oppFD[31], oppFD[30], oppFD[29], FDInstruction[28], oppFD[27]);

or OR8(branch0, blt0, bne0);

endmodule