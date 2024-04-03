module exceptions(add, sub, addi, mul, div, over, mulE, error, errorOut);

input add, sub, addi, mul, div, over, mulE;
output [31:0] errorOut;
output error;

wire addE, subE, addiE, mulEo, divE; 
wire [31:0] inter0, inter1, inter2, inter3, inter4;

and AND0(addE, add, over);
assign inter0 = addE ? 32'b00000000000000000000000000000001 : 32'b0;

and AND1(subE, sub, over);
assign inter1 = subE ? 32'b00000000000000000000000000000011 : inter0;

and AND2(addiE, addi, over);
assign inter2 = addiE ? 32'b00000000000000000000000000000010 : inter1;

and AND3(mulEo, mul, mulE);
assign inter3 = mulEo ? 32'b00000000000000000000000000000100 : inter2;

and AND4(divE, div, mulE);
assign inter4 = divE ? 32'b00000000000000000000000000000101 : inter3;

assign errorOut = inter4;

or OR0(error, addE, subE, addiE, mulEo, divE);


endmodule