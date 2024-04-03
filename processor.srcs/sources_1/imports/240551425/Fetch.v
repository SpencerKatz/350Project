module Fetch(clock, start, finish, branch, jump, target, newLocation, jr, rs1, hazardHere);
input clock, branch, jump, jr;
input [31:0] start, target, newLocation, rs1;
input hazardHere;
output [31:0] finish;

wire overflow; 
wire[31:0] beforeFinish, finish1, finish2;
wire [31:0] counting;

CLA CLA1(start, 32'b0, beforeFinish, overflow, 1'b1);

assign finish1 = jump ? target : beforeFinish;
assign finish2 = branch ? newLocation : finish1;
assign finish = (jr && !hazardHere) ? rs1 : finish2;

endmodule