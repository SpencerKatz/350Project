module ALU_adder(S,A, B, Cin);
    input A, B, Cin;
    output S;
    
    xor XOR1(S,A,B,Cin);

endmodule