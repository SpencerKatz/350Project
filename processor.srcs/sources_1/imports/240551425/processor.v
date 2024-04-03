/**
 * READ THIS DESCRIPTION!
 *
 * This is your processor module that will contain the bulk of your code submission. You are to implement
 * a 5-stage pipelined processor in this module, accounting for hazards and implementing bypasses as
 * necessary.
 *
 * Ultimately, your processor will be tested by a master skeleton, so the
 * testbench can see which controls signal you active when. Therefore, there needs to be a way to
 * "inject" imem, dmem, and regfile interfaces from some external controller module. The skeleton
 * file, Wrapper.v, acts as a small wrapper around your processor for this purpose. Refer to Wrapper.v
 * for more details.
 *
 * As a result, this module will NOT contain the RegFile nor the memory modules. Study the inputs 
 * very carefully - the RegFile-related I/Os are merely signals to be sent to the RegFile instantiated
 * in your Wrapper module. This is the same for your memory elements. 
 *
 *
 */
module processor(
    // Control signals
    clock,                          // I: The master clock
    reset,                          // I: A reset signal

    // Imem
    address_imem,                   // O: The address of the data to get from imem
    q_imem,                         // I: The data from imem

    // Dmem
    address_dmem,                   // O: The address of the data to get or put from/to dmem
    data,                           // O: The data to write to dmem
    wren,                           // O: Write enable for dmem
    q_dmem,                         // I: The data from dmem

    // Regfile
    ctrl_writeEnable,               // O: Write enable for RegFile
    ctrl_writeReg,                  // O: Register to write to in RegFile
    ctrl_readRegA,                  // O: Register to read from port A of RegFile
    ctrl_readRegB,                  // O: Register to read from port B of RegFile
    data_writeReg,                  // O: Data to write to for RegFile
    data_readRegA,                  // I: Data from port A of RegFile
    data_readRegB                   // I: Data from port B of RegFile
	 
	);

	// Control signals
	input clock, reset;
	
	// Imem
    output [31:0] address_imem;
	input [31:0] q_imem;

	// Dmem
	output [31:0] address_dmem, data;
	output wren;
	input [31:0] q_dmem;

	// Regfile
	output ctrl_writeEnable;
	output [4:0] ctrl_writeReg, ctrl_readRegA, ctrl_readRegB;
	output [31:0] data_writeReg;
	input [31:0] data_readRegA, data_readRegB;

	/* YOUR CODE STARTS HERE */
    wire [31:0] finish;
    wire [31:0] code;
    wire [31:0] location;
    wire firstInter;
    wire firstIteration;
    wire [31:0] fetchInter;
    wire [31:0] fetch_data;
    wire notClock;
    wire [31:0] interPc;

    not NOT12(notClock, clock);

    // first iteration
    or OR0(firstInter, firstIteration, notClock);

    dffe_ref first(firstIteration, firstInter, notClock, 1'b1, reset);
    // fetch
    wire [31:0] pastInstruction, rs1, rs2;
    wire [31:0] pc2, interInstruct1, interInstruct2, interInstruct3;

    assign address_imem = fetch_data;

    Fetch fetch(notClock, fetch_data, finish, branch, jump, target, newLocation, jr, useRS1, hazardHere);

    assign interPc = (muldivstall || hazardHere) ? fetch_data : finish;

    registerEx pc(1'b1, reset, notClock, interPc, fetch_data);
    // 
    assign interInstruct1 = nop ? 32'b0 : q_imem;
    assign interInstruct3 = (muldivstall || hazardHere) ? pastInstruction : interInstruct1;
    //

    registerEx pc1(1'b1, reset, notClock, fetch_data, pc2);
    
    registerEx instruction(1'b1, reset, notClock, interInstruct3, pastInstruction);


    // decode
    wire [31:0] interInstruct;
    wire [31:0] newInstruction;
    wire [31:0] muldivStallReg1, muldivStallReg2;

    Decode decode(notClock, pastInstruction, ctrl_readRegA, ctrl_readRegB);

    assign muldivStallReg1 = muldivstall ? useRS1 : data_readRegA;
    assign muldivStallReg2 = muldivstall ? useRS2 : data_readRegB;

    registerEx rs1reg(1'b1, reset, notClock, muldivStallReg1, rs1);

    registerEx rs2reg(1'b1, reset, notClock, muldivStallReg2, rs2);

    // 
    assign interInstruct = nop ? 32'b0 : pastInstruction;
    assign interInstruct2 = muldivstall ? newInstruction : interInstruct;
    //

    registerEx holdIns(1'b1, reset, notClock, interInstruct2, newInstruction);

    registerEx pc10(1'b1, reset, notClock, pc2, pc3);
    
   // execute
    wire [31:0] result, result1;
    wire [31:0] useResult;
    wire [31:0] memInstruction;
    wire [31:0] saveB;
    wire [31:0] extended;
    wire [31:0] newLocation;
    wire o1;
    wire lessThan, notEqual;
    wire addi, mul, div, j, bne, jal, jr, blt, sw1, lw1;
    wire add, sub;
    wire [31:0] target;
    wire jump;
    wire branch, branch1, branch2;
    wire [31:0] pc3;
    wire [31:0] multR, divR;
    wire multE, divE;
    wire multRdy, divRdy;
    wire [31:0] dontUse;
    wire bexJump, bex, setx;
    wire nop;
    wire [31:0] resultM;
    wire [31:0] resultD;
    wire mulStall;
    wire divStall;
    wire muldivstall;
    wire notmrdy, notdrdy, muldiv12;
    wire mully, divvy;
    wire mully2, divvy2;
    wire mully3, divvy3;
    wire lastStall;
    wire over;
    
    registerEx instruction1 (1'b1, reset, notClock, exceptUse, memInstruction);

    Execute execute(notClock, reset, useRS1, useRS2, newInstruction, result1, dontUse, lessThan, notEqual, over);

    //
    assign extended[15:0] = newInstruction[15:0];
    assign extended[31:16] = newInstruction[15] ? 16'b1111111111111111 : 16'b0;

    CLA CLA1(pc2, extended, newLocation, o1, 1'b0);
    completeType COMPLETETYPE(newInstruction, add, sub, addi, mul, div, j, bne, jal, jr, blt, sw1, lw1, bex, setx);

    assign target[26:0] = newInstruction[26:0];
    assign target[31:27] = newInstruction[26] ? 5'b11111 : 5'b0;

    // or OR4(jump, j, jal);

    and AND14(bexJump, notEqual, bex);
    or OR4(jump, j, jal, bexJump, jr);

    //

    and AND12(branch1, notEqual, bne);
    and AND13(branch2, lessThan, blt);

    or OR5(branch, branch1, branch2);

    //
    or OR6(nop, branch, jump, hazardHere);
    //

    // 
    // assign result = jal ? pc2 : result1;
    assign resultM = jal ? pc2 : result1;
    //
    dffe_ref m1(lastStall, muldivstall, notClock, 1'b1, 1'b0);

    assign mully = lastStall ? 1'b0 : mul;
    assign divvy = lastStall ? 1'b0 : div;
    // not NOT14(mully3, mully);
    // and AND16(mully2, mully3, mul);
    

    // not NOT15(divvy3, divvy);
    // and AND17(divvy2, divvy3, div);
    wire [31:0] resultInter;
    wire mulEuse;
    wire [31:0] exceptUse;

   

    multdiv md1(useRS1, useRS2, mully, divvy, notClock, multR, multE, multRdy);

    not NOT13(notmrdy, multRdy);

    and AND18(mulEuse, multE, multRdy);

    and AND15(muldivstall, muldiv12, notmrdy);

    or OR14(muldiv12, mul, div);

    assign resultInter = muldiv12 ? multR : resultM;
    assign result = errorOccured ? errorOut : resultInter;

    // exceptions
    wire errorOccured;
    wire [31:0] errorOut;

    exceptions e11(add, sub, addi, mul, div, over, mulEuse, errorOccured, errorOut);

     assign exceptUse[31:27] = newInstruction[31:27];
     assign exceptUse [21:0] = newInstruction[21:0];
     assign exceptUse[26:22] = errorOccured ? 5'b11110 : newInstruction[26:22];
    
    

    //

    registerEx resultreg(1'b1, reset, notClock, result, useResult);

    

    registerEx saveBreg(1'b1, reset, notClock, useRS2, saveB);
    // Memory
    wire lw;
    wire [31:0] saveResult;
    wire [31:0] lastInstruction;
    wire [31:0] saveMemory;
    wire[31:0] useMem;

    // Memory memory(clock, reset, memInstruction, wren, lw);

    //
    Memory memory(notClock, reset, memInstruction, wren, lw);
    //

    assign address_dmem = useResult;

    assign data = useMemory;

    registerEx saveMemoutput(1'b1, reset, notClock, q_dmem, useMem);
    registerEx saveResultreg(1'b1, reset, notClock, useResult, saveResult);

    registerEx instruction2(1'b1, reset, notClock, memInstruction, lastInstruction);

    // registerEx saveMem (1'b1, reset, clock, q_dmem, saveMemory);

    // writeback
    wire lw2;
    wire notUsed;
    wire [31:0] writeBack;
    wire rType, iType, jType;
    wire interControl;
    wire sw, notsw;
    wire notUsed2;
    wire bne1, blt1, jally;
    wire notbne, notblt;
    wire [5:0] interReg, interReg2;
    wire ctrlInter2;
    wire bex1, setx1;
    wire[31:0] value;
    wire ctrl_writeEnableInter;

    Memory memory2(notClock, reset, lastInstruction, notUsed, lw2);
    // assign writeBack = lw2 ? saveMemory : saveResult;
    assign value[26:0] =  lastInstruction[26:0];
    assign value[31:27] = lastInstruction[26] ? 5'b11111 : 5'b0;

    assign writeBack = lw2 ? useMem : saveResult;
    assign data_writeReg = setx1 ? value : writeBack;

    // assign data_writeReg = writeBack;

    assign interReg = lastInstruction[26:22];

    type TYPE0(lastInstruction, rType, iType, jType, notUsed, sw);

    or OR3(interControl, rType, iType);

    not NOT5(notsw, sw);

    completeType c2(lastInstruction, notUsed, notUsed, notUsed2, notUsed2, notUsed2, notUsed2, bne1, jally, notUsed2, blt1, notUsed2, notUsed2, bex1, setx1);

    not NOT10(notbne, bne1);

    not NOT11(notblt, blt1);

    and AND10(ctrlInter2, interControl, notsw, notbne, notblt);

    or OR12(ctrl_writeEnableInter, ctrlInter2, jally, setx1);

    assign ctrl_writeEnable = (lastInstruction[26:22] == 5'b0 && (rType || iType)) ? 1'b0 : ctrl_writeEnableInter;

    // Jal
    // assign ctrl_writeReg = jally ? 5'b11111 : interReg;
    //

    // will likely have to move these things up to make bypassing easier

    assign interReg2 = jally ? 5'b11111 : interReg;
    assign ctrl_writeReg = setx1 ? 5'b11110 : interReg2;
    //

    // and AND10(ctrl_writeEnable, interControl, 1'b0);
    //
    // bypassing
    wire [31:0] useRS1, useRS2, useMemory;
    wire hazardHere;

    bypassing b1(pastInstruction, exceptUse, memInstruction, 
    lastInstruction, rs1, rs2, useResult, data_writeReg, saveB,
    useRS1, useRS2, useMemory, hazardHere);

    //
    
	/* END CODE */

endmodule



//
 // and AND14(bexJump, notEqual, bex);
    // or OR4(jump, j, jal, bexJump);
    //

//
// assign interReg2 = jally ? 5'b11111 : interReg;
    // assign ctrl_writeReg = setx1 ? 5'b11110 : interReg2;
    //