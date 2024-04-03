module regfile (
	clock,
	ctrl_writeEnable, ctrl_reset, ctrl_writeReg,
	ctrl_readRegA, ctrl_readRegB, data_writeReg,
	data_readRegA, data_readRegB
);

	input clock, ctrl_writeEnable, ctrl_reset;
	input [4:0] ctrl_writeReg, ctrl_readRegA, ctrl_readRegB;
	input [31:0] data_writeReg;

	output [31:0] data_readRegA, data_readRegB;

	wire [31:0] whichRegA, whichRegB, whichRegWrite;

	assign whichRegA = 1'b1 << ctrl_readRegA;
	assign whichRegB = 1'b1 << ctrl_readRegB;
	assign whichRegWrite = 1'b1 << ctrl_writeReg;
generate 
	genvar i;
	for (i = 1 ; i < 32 ; i=i+1) begin : gen_loop
		wire [31:0] w1;
		wire w2, w3;
		assign w2 = whichRegWrite[i] ? 1'b1: 1'b0; 
		and AND1(w3, w2, ctrl_writeEnable);
		registerEx register0(.ctrl_writeEnable(w3), .ctrl_reset(ctrl_reset), .clock(clock), .data_writeReg(data_writeReg), .read(w1));
		assign data_readRegA = whichRegA[i] ? w1: 32'bz;
		assign data_readRegB = whichRegB[i] ? w1: 32'bz;
	end 
endgenerate
	assign data_readRegA = whichRegA[0] ? 32'b0: 32'bz;
	assign data_readRegB = whichRegB[0] ? 32'b0: 32'bz;

endmodule
