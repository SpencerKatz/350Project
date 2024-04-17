module uart_tx (
    input wire clk,               // Clock input
    input wire rst_n,             // Active low reset
    input wire [7:0] tx_data,     // Data to be transmitted
    input wire tx_en,             // Transmit enable
    output reg tx,                // UART transmit line
    output reg tx_busy            // Transmit busy signal
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
            tx <= 1'b1;            // Drive line to idle state
            state <= IDLE;
            tx_busy <= 1'b0;
            tick_cnt <= 0;
            bit_cnt <= 0;
        end else begin
            case (state)
                IDLE: begin
                    tx <= 1'b1;
                    if (tx_en && !tx_busy) begin
                        shift_reg <= tx_data;
                        tick_cnt <= 0;
                        bit_cnt <= 0;
                        state <= START_BIT;
                        tx_busy <= 1'b1;
                    end
                end
                START_BIT: begin
                    tx <= 1'b0;
                    if (tick_cnt < BIT_CNT_MAX - 1) begin
                        tick_cnt <= tick_cnt + 1;
                    end else begin
                        tick_cnt <= 0;
                        state <= DATA_BITS;
                    end
                end
                DATA_BITS: begin
                    tx <= shift_reg[0];
                    if (tick_cnt < BIT_CNT_MAX - 1) begin
                        tick_cnt <= tick_cnt + 1;
                    end else begin
                        tick_cnt <= 0;
                        shift_reg <= shift_reg >> 1;
                        if (bit_cnt < 7) begin
                            bit_cnt <= bit_cnt + 1;
                        end else begin
                            bit_cnt <= 0;
                            state <= STOP_BIT;
                        end
                    end
                end
                STOP_BIT: begin
                    tx <= 1'b1;
                    if (tick_cnt < BIT_CNT_MAX - 1) begin
                        tick_cnt <= tick_cnt + 1;
                    end else begin
                        tick_cnt <= 0;
                        state <= CLEANUP;
                    end
                end
                CLEANUP: begin
                    tx_busy <= 1'b0;
                    state <= IDLE;
                end
                default: begin
                    state <= IDLE;
                end
            endcase
        end
    end

endmodule