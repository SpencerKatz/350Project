`timescale 1ns / 1ps
/**
 * 
 * READ THIS DESCRIPTION:
 *
 * This is the Wrapper module that will serve as the header file combining your processor, 
 * RegFile and Memory elements together.
 *
 * This file will be used to generate the bitstream to upload to the FPGA.
 * We have provided a sibling file, Wrapper_tb.v so that you can test your processor's functionality.
 * 
 * We will be using our own separate Wrapper_tb.v to test your code. You are allowed to make changes to the Wrapper files 
 * for your own individual testing, but we expect your final processor.v and memory modules to work with the 
 * provided Wrapper interface.
 * 
 * Refer to Lab 5 documents for detailed instructions on how to interface 
 * with the memory elements. Each imem and dmem modules will take 12-bit 
 * addresses and will allow for storing of 32-bit values at each address. 
 * Each memory module should receive a single clock. At which edges, is 
 * purely a design choice (and thereby up to you). 
 * 
 * You must change line 36 to add the memory file of the test you created using the assembler
 * For example, you would add sample inside of the quotes on line 38 after assembling sample.s
 *
 **/

module Wrapper (clock, reset2, LED, JA, JB, JC, SW);
	input wire clock, reset2;
	input wire [15:0] SW;
	output wire [1:1] JA;
	output wire [1:1] JB, JC;
	
	wire reset;
	assign reset = ~reset2;
	output wire [15:0] LED;

	wire rwe, mwe;
	wire[4:0] rd, rs1, rs2;
	wire[31:0] instAddr, instData, 
		rData, regA, regB,
		memAddr, memDataIn, memDataOut;
		
		assign LED = instAddr[15:0];


	// ADD YOUR MEMORY FILE HERE
	localparam INSTR_FILE = "addi_basic";
	
	// Main Processing Unit
	processor CPU(.clock(clock), .reset(reset), 
								
		// ROM
		.address_imem(instAddr), .q_imem(instData),
									
		// Regfile
		.ctrl_writeEnable(rwe),     .ctrl_writeReg(rd),
		.ctrl_readRegA(rs1),     .ctrl_readRegB(rs2), 
		.data_writeReg(rData), .data_readRegA(regA), .data_readRegB(regB),
									
		// RAM
		.wren(mwe), .address_dmem(memAddr), 
		.data(memDataIn), .q_dmem(memDataOut)); 
	
	// Instruction Memory (ROM)
	ROM #(.MEMFILE({INSTR_FILE, ".mem"}))
	InstMem(.clk(clock), 
		.addr(instAddr[11:0]), 
		.dataOut(instData));
	
	// Register File
	regfile RegisterFile(.clock(clock), 
		.ctrl_writeEnable(rwe), .ctrl_reset(reset), 
		.ctrl_writeReg(rd),
		.ctrl_readRegA(rs1), .ctrl_readRegB(rs2), 
		.data_writeReg(rData), .data_readRegA(regA), .data_readRegB(regB));
						
	// Processor Memory (RAM)
	RAM ProcMem(.clk(clock), 
		.wEn(mwe), 
		.addr(memAddr[11:0]), 
		.dataIn(memDataIn), 
		.dataOut(memDataOut));
		
	
	wire chSel, audioOut, audioEn;
	wire tone;
	
	
	pwn p1(.clk(clock), .tone(tone), .chSel(chSel), .audioOut(audioOut), .audioEn(audioEn), .SW(SW));
	assign JA[1] = audioOut;
	assign JC[1] = 1'b1;
	wire [23:0] div;
	assign div = 100000000/9600; 
	
	wire [30:0] i_setup;
	assign i_setup = {1'b0, 2'b00, 1'b0, 1'b0, 2'b00, 24'b0};
	wire o_wr;
	wire [7:0] data;
	
    rxuart receiver(.i_clk(clock), .i_reset(reset2), .i_setup(i_setup), .i_uart_rx(JC[1]), .o_wr(o_wr), .o_data(data), .o_break(), .o_parity_err(), .o_frame_err(), .o_ck_uart());
    
    
    wire [30:0] i_setup_tx;
	assign i_setup_tx = {1'b0, 2'b00, 1'b0, 1'b0, 2'b00, 24'b0};
	
	wire o_uart_tx, o_busy;
    
    wire[7:0] i_data;
    assign i_data = 8'b00000101;
    
    txuart transmitter(.i_clk(clock), .i_reset(reset2), .i_setup(i_setup_tx), .o_uart_tx(JB[1]), .i_wr(1'b1), .i_data(i_data), .i_break(1'b0), .i_cts_n(1'b1), .o_busy(o_busy));

    assign LED[7:0] = 8'b00000101;
    assign LED[9] = data[7];
    assign LED[8] = data[6];
    assign LED[9] = data[5];
    assign LED[8] = data[4];
    assign LED[9] = data[3];
    assign LED[8] = data[2];
    assign LED[9] = data[1];
    assign LED[8] = data[0];



endmodule
