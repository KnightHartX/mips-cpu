`timescale 1ns / 1ps
module top_tb ();
    reg clk, reset;
    reg [15:0] switches;
    wire [15:0] leds;
    reg btn_u, btn_d, btn_l, btn_r;
    top _top(clk, reset, switches, btn_u, btn_d, btn_l, btn_r, leds);

    always #5
        clk = ~clk;

    initial fork
        clk = 0;
        reset = 0;
        btn_u = 0;
        btn_d = 0;
        btn_l = 0;
        btn_r = 0;
        switches = 16'h0000;
//        # 1000 reset = 1;
//        # 200 reset = 0;
//        # 5 reset=0;
        # 50 switches = 16'b0000000000000011;
        
    	# 300 btn_u = 1;
    	# 400 btn_u = 0;
    	
    	# 1300 btn_d = 1;
        # 1400 btn_d = 0;
        
        # 2300 btn_l = 1;
        # 2400 btn_l = 0;
        
	    # 3300 btn_d = 1;
        # 3400 btn_d = 0;
    join
endmodule // top_tb
