module tflipflop (T, Clock, Q, notQ, clear);
    input wire T, Clock, clear;
    output wire Q, notQ;
    wire notT, w1, D, w2;

    not NOT1(notT, T);
    and AND1(w1, notT, Q);
    and AND2(w2, T, notQ);
    or OR1(D, w1, w2);

    dffe_ref flip1(Q, D, Clock, 1'b1, clear);
    not NOT2(notQ, Q);

endmodule