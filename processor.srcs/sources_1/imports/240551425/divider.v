module divider(data_operandA, data_operandB, data_result, clock, firstIteration, except);

input [31:0] data_operandA, data_operandB;
input clock, firstIteration;

output [31:0] data_result;
output except;

wire signed [63:0] quotient, inter, inter2;
wire [31:0] oppB1, oppB, oppA1, oppA, useA, useB, useOppB1, useOppB;
wire w1, w6, w7, w9, w8, w10, w11;
wire [31:0] w2, w3, w4,w5;
wire [31:0] upper, upper2;

assign oppB1 = ~data_operandB;
CLA CLA1 (oppB1, 32'b0, oppB,w1, 1'b1);

assign oppA1 = ~data_operandA;
CLA CLA3 (oppA1, 32'b0, oppA,w9, 1'b1);

mux_2 MUX6 (useA, data_operandA[31], data_operandA, oppA);
mux_2 MUX7 (useB, data_operandB[31], data_operandB, oppB);


mux_2 MUX0(w2,firstIteration, w4, useA);
mux_2 MUX1(w3,firstIteration, w5, 32'b0);


assign useOppB1 = ~useB;
CLA CLA4 (useOppB1, 32'b0,useOppB,w1, 1'b1);

assign quotient [63:32] = w3;
assign quotient [31:0] = w2;

assign inter = quotient << 1;

CLA CLA2(inter[63:32], useOppB, upper, w10, 1'b0);

mux_2 MUX2 (inter2[63:32], upper[31], upper, inter[63:32]);
assign inter2[31:1] = inter[31:1];
mux_2 MUX3(inter2[0], upper[31],1'b1,1'b0);

assign data_result = inter2[31:0];
 
or OR11(w7, w1, w8);
assign except = 1'b0;

dffe_ref flip1(w11, w7,clock, 1'b1, 1'b0);


registerEx reg1(1'b1, 1'b0, clock, inter2[31:0], w4);

// registerEx reg2(1'b1, 1'b0, clock, data_operandA, data_operandA);

registerEx reg3(1'b1, 1'b0, clock, inter2[63:32], w5);

endmodule