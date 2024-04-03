
module multdiv(
	data_operandA, data_operandB, 
	ctrl_MULT, ctrl_DIV, 
	clock, 
	data_result, data_exception, data_resultRDY);

    input [31:0] data_operandA, data_operandB;
    input ctrl_MULT, ctrl_DIV, clock;

    output [31:0] data_result;
    output data_exception, data_resultRDY;
    
    wire [31:0] count, oppCount, upper32, oppUpper32, w1, w2, data_resultAlmost;
    wire firstIteration, floatBit, outFloatBit, acFloatBit;
    wire overflow;
    wire clear, notClock, notrdy, rdy, notrdy2;
    wire almost;
    wire w3, w4, oppUpper;
    wire [31:0] divis_result, divis_result2, mult_result, oppDivis, notB, divAnswer;
    wire w5, negDivis, negException, multException, multInter, remOpp, ctrl, interExcept, divExcept;
    wire w22, w23, w24, w25, w26, w27, w28, w29, w30, w31, w32, w33, w34, w35, overrideInt, overrideInt2, overide;
    wire w36, w37, w38, w39, w40, w41;

    //
    not NOT40(w26, data_operandA[31]);
    not NOT41(w27, data_operandB[31]);
    or OR40(w22, w26, data_operandA[30], data_operandA[29], data_operandA[28], data_operandA[27], data_operandA[26], data_operandA[25], data_operandA[24], data_operandA[23], data_operandA[22], data_operandA[21], data_operandA[20], data_operandA[19], data_operandA[18], data_operandA[17], data_operandA[16], data_operandA[15], data_operandA[14], data_operandA[13], data_operandA[12], data_operandA[11], data_operandA[10], data_operandA[9], data_operandA[8], data_operandA[7], data_operandA[6], data_operandA[5], data_operandA[4], data_operandA[3], data_operandA[2], data_operandA[1], data_operandA[0]);
    or OR41(w23, w27, data_operandB[30], data_operandB[29], data_operandB[28], data_operandB[27], data_operandB[26], data_operandB[25], data_operandB[24], data_operandB[23], data_operandB[22], data_operandB[21], data_operandB[20], data_operandB[19], data_operandB[18], data_operandB[17], data_operandB[16], data_operandB[15], data_operandB[14], data_operandB[13], data_operandB[12], data_operandB[11], data_operandB[10], data_operandB[9], data_operandB[8], data_operandB[7], data_operandB[6], data_operandB[5], data_operandB[4], data_operandB[3], data_operandB[2], data_operandB[1], data_operandB[0]);

    not NOT42 (w24, w22);
    not NOT43 (w25, w23);
    
    not NOT44(w28, data_operandA[0]);
    not NOT45(w29, data_operandB[0]);
    or OR42(w30, w28, data_operandA[31], data_operandA[30], data_operandA[29], data_operandA[28], data_operandA[27], data_operandA[26], data_operandA[25], data_operandA[24], data_operandA[23], data_operandA[22], data_operandA[21], data_operandA[20], data_operandA[19], data_operandA[18], data_operandA[17], data_operandA[16], data_operandA[15], data_operandA[14], data_operandA[13], data_operandA[12], data_operandA[11], data_operandA[10], data_operandA[9], data_operandA[8], data_operandA[7], data_operandA[6], data_operandA[5], data_operandA[4], data_operandA[3], data_operandA[2], data_operandA[1]);
    or OR43(w31, w29, data_operandB[31], data_operandB[30], data_operandB[29], data_operandB[28], data_operandB[27], data_operandB[26], data_operandB[25], data_operandB[24], data_operandB[23], data_operandB[22], data_operandB[21], data_operandB[20], data_operandB[19], data_operandB[18], data_operandB[17], data_operandB[16], data_operandB[15], data_operandB[14], data_operandB[13], data_operandB[12], data_operandB[11], data_operandB[10], data_operandB[9], data_operandB[8], data_operandB[7], data_operandB[6], data_operandB[5], data_operandB[4], data_operandB[3], data_operandB[2], data_operandB[1]);

    or OR46(w36, data_operandA[0], data_operandA[31], data_operandA[30], data_operandA[29], data_operandA[28], data_operandA[27], data_operandA[26], data_operandA[25], data_operandA[24], data_operandA[23], data_operandA[22], data_operandA[21], data_operandA[20], data_operandA[19], data_operandA[18], data_operandA[17], data_operandA[16], data_operandA[15], data_operandA[14], data_operandA[13], data_operandA[12], data_operandA[11], data_operandA[10], data_operandA[9], data_operandA[8], data_operandA[7], data_operandA[6], data_operandA[5], data_operandA[4], data_operandA[3], data_operandA[2], data_operandA[1]);
    or OR47(w37, data_operandB[0], data_operandB[31], data_operandB[30], data_operandB[29], data_operandB[28], data_operandB[27], data_operandB[26], data_operandB[25], data_operandB[24], data_operandB[23], data_operandB[22], data_operandB[21], data_operandB[20], data_operandB[19], data_operandB[18], data_operandB[17], data_operandB[16], data_operandB[15], data_operandB[14], data_operandB[13], data_operandB[12], data_operandB[11], data_operandB[10], data_operandB[9], data_operandB[8], data_operandB[7], data_operandB[6], data_operandB[5], data_operandB[4], data_operandB[3], data_operandB[2], data_operandB[1]);

    not NOT46(w32, w30);
    not NOT47(w33, w31);

    not NOT48(w38, w36);
    not NOT49(w39, w37);

    or OR48(w40, w32, w38);
    or OR49(w41, w33, w39);

    and AND40(w34, w41, w24);
    and AND41(w35, w40, w25);

    mux_2 MUX40(overrideInt,w34, 1'b0, 1'b1);
    mux_2 MUX41(overrideInt2,w35, 1'b0, 1'b1);
    or OR44(overide,overrideInt, overrideInt2);
    // 
    or OR45(ctrl, ctrl_DIV, ctrl_MULT);


    not NOT10(notClock, clock);
    counter C1(1'b1, clock, rdy, count, data_resultRDY, ctrl);
    // and AND10(data_resultRDY, notClock, almost);
    // mux_2 MUX1(clear, firstIteration, 1'b0, 1'b1);
    
    
    assign oppCount = ~count;
    // and AND12(data_resultRDY, rdy, oppCount[0]);
    // not NOT11(notrdy, data_resultRDY);
    // not NOT12(notrdy2, rdy);
    assign data_resultRDY = rdy;
    // assign clear = data_resultRDY;
    // and AND13(clear, notrdy2, oppCount[1], oppCount[2], oppCount[3], oppCount[4], count[0]);
    and AND1(firstIteration, oppCount[0], oppCount[1], oppCount[2], oppCount[3], oppCount[4]);
    // assign firstIteration = ctrl_MULT;
    mux_2 MUX0(acFloatBit, firstIteration, floatBit, 1'b0);


    

    multiplier M1(data_operandA,data_operandB,acFloatBit,outFloatBit, multException ,firstIteration, mult_result, clock, 1'b0, upper32);
    dffe_ref flip1(floatBit,outFloatBit,clock, 1'b1, 1'b0);

    dffe_ref flip2(remOpp,ctrl_MULT,clock, ctrl, 1'b0);

    divider D1(data_operandA, data_operandB, divis_result, clock, firstIteration, interExcept);

    assign oppDivis = ~divis_result;
    
    CLA CLA1 (oppDivis, 32'b0,divis_result2, w5, 1'b1);

    xor XOR1 (negDivis, data_operandA[31], data_operandB[31]);

    

    assign notB = ~data_operandB;

    and AND2(negException, notB[31], notB[30], notB[29], notB[28], notB[27], notB[26], notB[25], notB[24], notB[23], notB[22], notB[21], notB[20], notB[19], notB[18], notB[17], notB[16], notB[15], notB[14], notB[13], notB[12], notB[11], notB[10], notB[9], notB[8], notB[7], notB[6], notB[5], notB[4], notB[3], notB[2], notB[1], notB[0]);

    or OR11(divExcept, negException, interExcept);

    mux_2 MUX2(divAnswer, negDivis, divis_result, divis_result2);

    // assign data_exception = negException;

    mux_2 MUX6 (multInter, overide, multException, 1'b0);

    mux_2 MUX3 (data_exception, remOpp, divExcept, multInter);
    
    mux_2 MUX4 (data_resultAlmost,remOpp, divAnswer, mult_result);

     mux_2 MUX5 (data_result, data_resultRDY, 32'b0, data_resultAlmost);

   // assign data_result = data_resultAlmost;

  

    // add your code here

endmodule

// iverilog -Wimplicit -o multdiv.vvp multiplier.v multdiv.v tflipflop.v counter.v multdiv_tb.v mux_2.v CLA.v CLA_block.v ALU_adder.v registerEx.v dffe_ref.v divider.v