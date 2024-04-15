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
    output wire [1:1] JC,
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
    

    
// UART control signals
    reg i_wr;                     // Write strobe for UART
    reg [7:0] i_data;             // Data to be sent over UART
    wire o_uart_tx;               // UART transmit output
    wire o_busy;                  // UART busy indicator

    // Setup configuration for the UART
    wire [30:0] uart_setup;
    assign uart_setup = {1'b0,    // Disable hardware flow control
                         2'b00,   // 8-bit words
                         1'b0,    // 1 stop bit
                         1'b0,    // No parity
                         1'b0,    // Parity not fixed (irrelevant here)
                         1'b0,    // Space parity (irrelevant here)
                         24'd868  // Clock divider for baud rate
                         };

    // Instantiate the txuart module with the defined parameters
    txuart #(
        .INITIAL_SETUP(31'd868)   // Default setup can be dynamically changed via uart_setup
    ) uart_transmitter (
        .i_clk(clock),
        .i_reset(reset),
        .i_setup(uart_setup),
        .i_break(1'b0),           // No break condition
        .i_wr(i_wr),
        .i_data(i_data),
        .i_cts_n(1'b1),           // Not using hardware flow control
        .o_uart_tx(o_uart_tx),
        .o_busy(o_busy)
    );

    // Logic to control UART transmission
    // Example: Sending a byte when not busy
    always @(posedge clock or posedge reset) begin
        if (reset) begin
            i_wr <= 0;
            i_data <= 8'h00;      // Default data
        end else if (!o_busy && some_condition_to_send) begin
            i_wr <= 1;            // Trigger sending data
            i_data <= 8'h01;  // Data to send
        end else begin
            i_wr <= 0;            // Reset write strobe
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
    
    assign JC[1] = o_uart_tx;

    
    
endmodule
