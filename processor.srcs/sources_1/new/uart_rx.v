module uart_rx (
    input wire clk,               // Clock input
    input wire rst_n,             // Active low reset
    input wire rx,                // UART receive line
    output reg [7:0] rx_data,     // Received data
    output reg rx_dv              // Data valid signal after reception
);

    // Parameters
    parameter CLK_FREQ = 100000000;  // Clock frequency in Hz
    parameter BAUD_RATE = 9600;     // Baud rate
    parameter BIT_CNT_MAX = CLK_FREQ / BAUD_RATE;  // Bit interval count

    // State definitions
    localparam IDLE = 0;
    localparam START_BIT = 1;
    localparam DATA_BITS = 2;
    localparam STOP_BIT = 3;
    localparam CLEANUP = 4;

    // State machine variables
    reg [3:0] state = IDLE;
    reg [3:0] bit_cnt = 0;
    reg [31:0] tick_cnt = 0;
    reg [7:0] shift_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_data <= 0;
            state <= IDLE;
            rx_dv <= 1'b0;
            tick_cnt <= 0;
            bit_cnt <= 0;
        end else begin
            case (state)
                IDLE: begin
                    rx_dv <= 1'b0;
                    if (rx == 1'b0) begin  // Start bit detected
                        tick_cnt <= 0;
                        bit_cnt <= 0;
                        state <= START_BIT;
                    end
                end
                START_BIT: begin
                    if (tick_cnt < (BIT_CNT_MAX / 2)) begin
                        tick_cnt <= tick_cnt + 1;
                    end else begin
                        tick_cnt <= 0;
                        state <= DATA_BITS;
                    end
                end
                DATA_BITS: begin
                    if (tick_cnt < BIT_CNT_MAX - 1) begin
                        tick_cnt <= tick_cnt + 1;
                    end else begin
                        tick_cnt <= 0;
                        shift_reg[bit_cnt] <= rx;
                        if (bit_cnt < 7) begin
                            bit_cnt <= bit_cnt + 1;
                        end else begin
                            bit_cnt <= 0;
                            state <= STOP_BIT;
                        end
                    end
                end
                STOP_BIT: begin
                    if (tick_cnt < BIT_CNT_MAX - 1) begin
                        tick_cnt <= tick_cnt + 1;
                    end else begin
                        tick_cnt <= 0;
                        rx_data <= shift_reg;
                        rx_dv <= 1'b1;
                        state <= CLEANUP;
                    end
                end
                CLEANUP: begin
                    state <= IDLE;
                end
                default: begin
                    state <= IDLE;
                end
            endcase
        end
    end

endmodule