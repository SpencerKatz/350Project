module counter(T, Clock, ctrl_End, count, data_resultRDY, ctrl_MULT);
    input T, Clock, data_resultRDY, ctrl_MULT;
    output ctrl_End;
    output [4:0] count;
    wire w1, w2, w3, w4, w5, w6, w7, w8, w9, w10, w11, w12, w13, w14, w15;
    wire choose, choose2, choose3;
    wire notClock;

    mux_2 MUX0(choose, ctrl_MULT, 1'b0, 1'b1);

    tflipflop T1(T, Clock, w1, w2, ctrl_MULT);

    tflipflop T2(w1, Clock, w3, w4, ctrl_MULT);
    and AND1(w5, w1, w3);

    tflipflop T3(w5, Clock, w6, w7, ctrl_MULT);
    and AND2(w8, w5, w6);

    tflipflop T4(w8, Clock, w9, w10, ctrl_MULT);
    and AND3(w11, w8, w9);

    tflipflop T5(w11, Clock, w12, w13, ctrl_MULT);
    and AND4(w14, w11, w12);

    and AND5(ctrl_End, w1, w3, w6, w9, w12);

    // not NOT0(choose, w14);
    // and AND6(choose2, w1, choose);
    
    // assign ctrl_End = w14;
    // tflipflop T6(w14, Clock, ctrl_End, w15, ctrl_MULT);


    assign count[0] = w1;
    assign count[1] = w3;
    assign count[2] = w6;
    assign count[3] = w9;
    assign count[4] = w12;



endmodule