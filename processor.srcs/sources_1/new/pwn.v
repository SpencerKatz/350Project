
module pwn(
    input        clk, 		// System Clock Input 100 Mhz
    output       chSel,		// Channel select; 0 for rising edge, 1 for falling edge
    output       audioOut,	// PWM signal to the audio jack	
    output       audioEn);	// Audio Enable

	localparam MHz = 1000000;
	localparam SYSTEM_FREQ = 100*MHz; // System clock frequency
	localparam Max = 100;
	localparam Min = 0;

	assign chSel   = 1'b0;  // Collect Mic Data on the rising edge 
	assign audioEn = 1'b1;  // Enable Audio Output

	// Initialize the frequency array. FREQs[0] = 261
	reg[10:0] data[0:15];
//	initial begin
//		$readmemh("data.mem", data);
//	end
	
	////////////////////
	// Your Code Here //
	////////////////////

	//use switch input to index into FREQ
	
	assign tone_freq = 125;

	wire[17:0] limit;

	assign limit = (SYSTEM_FREQ)/((tone_freq)*2) - 1;

	
	reg desired_clk = 0;
	reg[17:0] counter = 0;
	always @(posedge clk) begin
		if(counter < limit)
			counter <= counter + 1;
		else begin
				counter <= 0;
				desired_clk <= ~desired_clk;
		end 
	end

	wire [6:0] duty_cycle, duty_cycle_out;
	assign duty_cycle = (desired_clk ? Max : Min);
   
   
   
    // assign duty_cycle_out = (select === 4'b0000) ? (duty_cycle_mic) : ((duty_cycle + duty_cycle_mic)/2);

	PWMSerializers pwm_serial(.clk(clk), .reset(reset), .duty_cycle(RAM_duty_cycle), .signal(audioOut)); 
	
	
//	-----------------------------------------------------------------------------------

endmodule