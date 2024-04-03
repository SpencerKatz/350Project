module alu(data_operandA, data_operandB, ctrl_ALUopcode, ctrl_shiftamt, data_result, isNotEqual, isLessThan, overflow);
        
    input [31:0] data_operandA, data_operandB;
    input [4:0] ctrl_ALUopcode, ctrl_shiftamt;

    output [31:0] data_result;
    output isNotEqual, isLessThan, overflow;

    wire[31:0] lefted, righted, anded, ored, summed,notted,subbed,sn,zero;

    wire w1, w2;

    assign zero = 32'b00000000000000000000000000000000;

    barrel_shifter_left left_shift(data_operandA,ctrl_shiftamt,lefted);
    barrel_shifter_right right_shift(data_operandA,ctrl_shiftamt,righted);

    orer OR1(data_operandA,data_operandB,ored);
    ander AND1(data_operandA,data_operandB,anded);

    CLA added(.data_operandA(data_operandA),.data_operandB(data_operandB),.sum(summed),.overflow(w2),.Cin(0));


    notter NOT1(data_operandB,notted);
    
    CLA subbed1(.data_operandA(data_operandA),.data_operandB(notted),.sum(subbed),.overflow(w1),.Cin(1));

    

    
    mux_32 mux1(data_result,ctrl_ALUopcode,summed,subbed,anded,ored,lefted,righted,zero,zero,zero,zero,zero,zero,zero,zero,zero,zero,zero,zero,zero,zero,zero,zero,zero,zero,zero,zero,zero,zero,zero,zero,zero,zero);

    mux_32 mux2(overflow, ctrl_ALUopcode, w2, w1, zero,zero,zero,zero,zero,zero,zero,zero,zero,zero,zero,zero,zero,zero,zero,zero,zero,zero,zero,zero,zero,zero,zero,zero,zero,zero,zero,zero,zero,zero);
    
    // assign isLessThan = data_result[31];
    xor XOR1(isLessThan, data_result[31], overflow);
    or OR2 (isNotEqual, data_result[0], data_result[1], data_result[2], data_result[3], data_result[4], data_result[5],
            data_result[6], data_result[7], data_result[8], data_result[9], data_result[10], data_result[11],
            data_result[12], data_result[13], data_result[14], data_result[15], data_result[16], data_result[17],
            data_result[18], data_result[19], data_result[20], data_result[21], data_result[22], data_result[23],
            data_result[24], data_result[25], data_result[26], data_result[27], data_result[28], data_result[29],
            data_result[30], data_result[31]);

    // add your code here:

endmodule