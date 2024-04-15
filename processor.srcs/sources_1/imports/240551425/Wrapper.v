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

module Wrapper (
    input wire clock,
    input wire reset2,
    input wire [15:0] SW,
    output wire [1:1] JA,
    output wire [15:0] LED
);

	wire reset;
	assign reset = ~reset2;

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
    

    // Define wire to connect txuart output to register
    wire txuart_output;

    // Instantiate txuart module
    txuart transmitter (
        .i_clk(clock),
        .i_reset(reset),
        .i_setup(setup),
        .i_break(1'b0), // Assuming no break condition
        .i_wr(i_wr),
        .i_data(dataToSend),
        .i_cts_n(1'b1), // Assuming hardware flow control is not used
        .o_uart_tx(txuart_output), // Connect to intermediate wire
        .o_busy(o_busy)
    );

    // Assign txuart output to register
    always @(*) begin
        o_uart_tx = txuart_output;
    end
    
    // Registers for controlling the UART transmission
    reg [7:0] dataToSend_reg;
    reg i_wr_reg;
    reg o_busy_reg;

    // Sequential logic for controlling UART transmission
    always @(posedge clock or posedge reset2) begin
        if (reset2) begin
            // Reset state
            i_wr_reg <= 1'b0;
            dataToSend_reg <= 8'b0;
            o_busy_reg <= 1'b0;
        end else begin
            // Update i_wr and dataToSend based on external inputs
            i_wr_reg <= i_wr;
            if (i_wr_reg) begin
                dataToSend_reg <= i_data;
                o_busy_reg <= 1'b1; // Signal that data transmission is in progress
            end else begin
                o_busy_reg <= 1'b0; // Signal that data transmission is complete
            end
        end
    end

    // Connect the registered signals to the txuart module
    assign dataToSend = dataToSend_reg;
    assign i_wr = i_wr_reg;
    assign o_busy = o_busy_reg;

    // setup wire
    wire [31:0] setup;
    assign setup[30] = 1'b1;
    assign setup[29:28] = 2'b00;
    assign setup[27] = 1'b0;
    assign setup[26] = 1'b0;
    assign setup[25] = 1'b1;
    assign setup[23:0] = (100000000 / 9600);

    wire i_wr;
    wire [7:0] dataToSend;
    reg o_uart_tx;
    wire o_busy;

    
    
endmodule
