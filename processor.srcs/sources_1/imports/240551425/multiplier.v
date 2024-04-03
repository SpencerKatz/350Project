module multiplier(data_operandA, data_operandB, floatBit, outFloatBit, overflow, firstIteration, data_result, clock, clear, upper32);

input [31:0] data_operandA, data_operandB;
input floatBit, firstIteration, clock, clear;
output [31:0] data_result, upper32;
output outFloatBit, overflow;

wire nottedFirstBit, nottedFloat, negControl, posControl, Cin, w10, w11, w12, w13, w14, w15, w16;
wire [31:0] whichAdd, oppA, sum, w2, w3, w4, w5, whichAdd2, w8, w9;
wire signed [63:0] multiplied, inter;

assign oppA = ~data_operandA;

mux_2 MUX0(w2,firstIteration, w4, data_operandB);
mux_2 MUX1(w3,firstIteration, w5, 32'b0);


not NOT1(nottedFloat, floatBit);
not NOT2(nottedFirstBit, w2[0]);

and AND1(negControl, w2[0], nottedFloat);
and AND2(posControl, nottedFirstBit, floatBit);

mux_2 MUX2(whichAdd2, posControl,  32'b0, data_operandA);
mux_2 MUX3(whichAdd, negControl, whichAdd2, oppA);
mux_2 MUX4(Cin, negControl, 1'b0, 1'b1);

CLA CLA1(w3, whichAdd, sum, w10, Cin);

assign multiplied[63:32] = sum;
assign multiplied[31:0] = w2;
assign outFloatBit = multiplied[0];
// check arithmetic
assign inter = multiplied >>> 1;
// barrel_shifter_right right_shift(multiplied,5'b00001,inter);
assign data_result = inter[31:0];
assign upper32 = inter[63:32];

and AND33(w11, inter[63], inter[62], inter[61], inter[60], inter[59],
          inter[58], inter[57], inter[56], inter[55], inter[54],
          inter[53], inter[52], inter[51], inter[50], inter[49],
          inter[48], inter[47], inter[46], inter[45], inter[44],
          inter[43], inter[42], inter[41], inter[40], inter[39],
          inter[38], inter[37], inter[36], inter[35], inter[34],
          inter[33], inter[32], inter[31]);
or OR33(w12, inter[63], inter[62], inter[61], inter[60], inter[59],
          inter[58], inter[57], inter[56], inter[55], inter[54],
          inter[53], inter[52], inter[51], inter[50], inter[49],
          inter[48], inter[47], inter[46], inter[45], inter[44],
          inter[43], inter[42], inter[41], inter[40], inter[39],
          inter[38], inter[37], inter[36], inter[35], inter[34],
          inter[33], inter[32], inter[31]);


not NOT30(w13, w11);



// and AND22(overflow, w12, w13);

//
// mux_2 MUX1(w15,firstIteration, w5, 1'b0);
//
wire w25;
assign w25 = firstIteration ? 1'b0 : w15;
//

or OR23(w14, w10, w25);

dffe_ref flip1(w15,w14,clock, 1'b1, clear);

and AND22(w16, w12, w13);

or OR24(overflow, w16, w14);
//


registerEx reg1(1'b1, clear, clock, inter[31:0], w4);

// registerEx reg2(1'b1, 1'b0, clock, data_operandA, data_operandA);

registerEx reg3(1'b1, clear, clock, inter[63:32], w5);




endmodule