`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// VGA verilog template
// Author:  Da Cheng
//////////////////////////////////////////////////////////////////////////////////
module vga_demo(ClkPort, vga_h_sync, vga_v_sync, vga_r, vga_g, vga_b, Sw0, Sw1, btnU, btnD, btnL, btnR,
	St_ce_bar, St_rp_bar, Mt_ce_bar, Mt_St_oe_bar, Mt_St_we_bar,
	An0, An1, An2, An3, Ca, Cb, Cc, Cd, Ce, Cf, Cg, Dp,
	LD0, LD1, LD2, LD3, LD4, LD5, LD6, LD7);
	
	input ClkPort, Sw0, btnU, btnD, btnL, btnR, Sw0, Sw1;
	output St_ce_bar, St_rp_bar, Mt_ce_bar, Mt_St_oe_bar, Mt_St_we_bar;
	output vga_h_sync, vga_v_sync, vga_r, vga_g, vga_b;
	output An0, An1, An2, An3, Ca, Cb, Cc, Cd, Ce, Cf, Cg, Dp;
	output LD0, LD1, LD2, LD3, LD4, LD5, LD6, LD7;
	reg vga_r, vga_g, vga_b;
	
	//////////////////////////////////////////////////////////////////////////////////////////
	
	

	/*  LOCAL SIGNALS */
	wire	reset, start, ClkPort, board_clk, clk, button_clk, Left, Right, Down, lost, Rotate;
	wire [159:0] blocks;
	reg [10:0] blockss;
	wire [31:0] score;

	BUF BUF1 (board_clk, ClkPort); 	
	BUF BUF2 (reset, Sw0);
	BUF BUF3 (start, Sw1);
	
	tetris dut ( 
	.Reset(reset),
	.Clk(clk),
	.Start(start), 
	.Ack(start), 
	.Left(Left),
	.Right(Right),
	.blocks(blocks),
	.q_Lose(lost),
	.Down(Down),
	.score(score),
	.Rotate(Rotate)
	);
	
	ee201_debouncer #(.N_dc(25)) ee201_debouncer_1 
        (.CLK(clk), .RESET(reset), .PB(btnL), .DPB( ), .SCEN(), .MCEN(Left), .CCEN( ));
	
	ee201_debouncer #(.N_dc(25)) ee201_debouncer_2 
        (.CLK(clk), .RESET(reset), .PB(btnR), .DPB( ), .SCEN(), .MCEN(Right), .CCEN( ));
		  
	ee201_debouncer #(.N_dc(25)) ee201_debouncer_3 
        (.CLK(clk), .RESET(reset), .PB(btnD), .DPB( ), .SCEN(), .MCEN( ), .CCEN(Down));	
		  
   ee201_debouncer #(.N_dc(25)) ee201_debouncer_4 
        (.CLK(clk), .RESET(reset), .PB(btnU), .DPB( ), .SCEN(), .MCEN(Rotate), .CCEN());
	
	reg [100:0]	DIV_CLK;
	always @ (posedge board_clk, posedge reset)  
	begin : CLOCK_DIVIDER
      if (reset)
			DIV_CLK <= 0;
      else
			DIV_CLK <= DIV_CLK + 1'b1;
	end	

	assign	button_clk = DIV_CLK[18];
	assign	clk = DIV_CLK[1];
	assign 	{St_ce_bar, St_rp_bar, Mt_ce_bar, Mt_St_oe_bar, Mt_St_we_bar} = {5'b11111};
	
	wire inDisplayArea;
	wire [9:0] CounterX;
	wire [9:0] CounterY;

	hvsync_generator syncgen(.clk(clk), .reset(reset),.vga_h_sync(vga_h_sync), .vga_v_sync(vga_v_sync), .inDisplayArea(inDisplayArea), .CounterX(CounterX), .CounterY(CounterY));
	
	/////////////////////////////////////////////////////////////////
	///////////////		VGA control starts here		/////////////////
	/////////////////////////////////////////////////////////////////
	reg [9:0] positionY;
	reg [9:0] positionX;
	reg [9:0] Btemp;

	
	integer i;
	

	always @ (posedge DIV_CLK[26])
		if(reset)
				begin
					positionY<=90;
					blockss[2] = 1;

				end

	always @(posedge DIV_CLK[26])
		begin

			if(reset)
				begin
					positionX<=300;


				end
//			else if(btnD && ~btnU && ~btnL && ~btnR && positionY < 385)
//				positionY<=positionY;//+2;
//			else if(~btnD && btnU && ~btnL && ~btnR && positionY > 115)
//				positionY<=positionY;//-2;
//			else if(~btnD && ~btnU && btnL && ~btnR && positionX > 215)
//				positionX<=positionX-2;
//			else if(~btnD && ~btnU && ~btnL && btnR && positionX < 385)
//				positionX<=positionX+2;
			
			
			/*
			for(i = 0; i < 200; i = i+1)
				begin
					if(blockss[i] == 1)
						begin
							Btemp = Btemp && CounterX > 200+20*(i%8) && CounterX < 220+20*(i%8) && CounterY<400-20*(i/8) && CounterY>380-20*(i/8);
						end
				end
				*/

		end	
	

	wire R = CounterX>(200 * lost) && CounterX<(360*lost) && CounterY>0 && CounterY<(400*lost);//CounterY>(positionY-10) && CounterY<=(positionY+10) && CounterX>=(positionX-10) && CounterX<=(positionX+10);//CounterX[8:5]==7;
	wire G = CounterX>(200 * !lost) && CounterX<(360 * !lost) && CounterY > 0 && CounterY < (400 * !lost);// && CounterY[5:3]==1;
	wire B = CounterX > (200+20*(0%8))*blocks[0] && CounterX < (220+20*(0%8))*blocks[0] && CounterY<(400-20*(0/8))*blocks[0] && CounterY>(380-20*(0/8))*blocks[0];
	wire B1 = CounterX > (200+20*(1%8))*blocks[1] && CounterX < (220+20*(1%8))*blocks[1] && CounterY<(400-20*(1/8))*blocks[1] && CounterY>(380-20*(1/8))*blocks[1];
	wire B2 = CounterX > (200+20*(2%8))*blocks[2] && CounterX < (220+20*(2%8))*blocks[2] && CounterY<(400-20*(2/8))*blocks[2] && CounterY>(380-20*(2/8))*blocks[2];
	wire B3 = CounterX > (200+20*(3%8))*blocks[3] && CounterX < (220+20*(3%8))*blocks[3] && CounterY<(400-20*(3/8))*blocks[3] && CounterY>(380-20*(3/8))*blocks[3];
	wire B4 = CounterX > (200+20*(4%8))*blocks[4] && CounterX < (220+20*(4%8))*blocks[4] && CounterY<(400-20*(4/8))*blocks[4] && CounterY>(380-20*(4/8))*blocks[4];
	wire B5 = CounterX > (200+20*(5%8))*blocks[5] && CounterX < (220+20*(5%8))*blocks[5] && CounterY<(400-20*(5/8))*blocks[5] && CounterY>(380-20*(5/8))*blocks[5];
	wire B6 = CounterX > (200+20*(6%8))*blocks[6] && CounterX < (220+20*(6%8))*blocks[6] && CounterY<(400-20*(6/8))*blocks[6] && CounterY>(380-20*(6/8))*blocks[6];
	wire B7 = CounterX > (200+20*(7%8))*blocks[7] && CounterX < (220+20*(7%8))*blocks[7] && CounterY<(400-20*(7/8))*blocks[7] && CounterY>(380-20*(7/8))*blocks[7];
	wire B8 = CounterX > (200+20*(8%8))*blocks[8] && CounterX < (220+20*(8%8))*blocks[8] && CounterY<(400-20*(8/8))*blocks[8] && CounterY>(380-20*(8/8))*blocks[8];
	wire B9 = CounterX > (200+20*(9%8))*blocks[9] && CounterX < (220+20*(9%8))*blocks[9] && CounterY<(400-20*(9/8))*blocks[9] && CounterY>(380-20*(9/8))*blocks[9];
	wire B10 = CounterX > (200+20*(10%8))*blocks[10] && CounterX < (220+20*(10%8))*blocks[10] && CounterY<(400-20*(10/8))*blocks[10] && CounterY>(380-20*(10/8))*blocks[10];
	wire B11 = CounterX > (200+20*(11%8))*blocks[11] && CounterX < (220+20*(11%8))*blocks[11] && CounterY<(400-20*(11/8))*blocks[11] && CounterY>(380-20*(11/8))*blocks[11];
	wire B12 = CounterX > (200+20*(12%8))*blocks[12] && CounterX < (220+20*(12%8))*blocks[12] && CounterY<(400-20*(12/8))*blocks[12] && CounterY>(380-20*(12/8))*blocks[12];
	wire B13 = CounterX > (200+20*(13%8))*blocks[13] && CounterX < (220+20*(13%8))*blocks[13] && CounterY<(400-20*(13/8))*blocks[13] && CounterY>(380-20*(13/8))*blocks[13];
	wire B14 = CounterX > (200+20*(14%8))*blocks[14] && CounterX < (220+20*(14%8))*blocks[14] && CounterY<(400-20*(14/8))*blocks[14] && CounterY>(380-20*(14/8))*blocks[14];
	wire B15 = CounterX > (200+20*(15%8))*blocks[15] && CounterX < (220+20*(15%8))*blocks[15] && CounterY<(400-20*(15/8))*blocks[15] && CounterY>(380-20*(15/8))*blocks[15];
	wire B16 = CounterX > (200+20*(16%8))*blocks[16] && CounterX < (220+20*(16%8))*blocks[16] && CounterY<(400-20*(16/8))*blocks[16] && CounterY>(380-20*(16/8))*blocks[16];
	wire B17 = CounterX > (200+20*(17%8))*blocks[17] && CounterX < (220+20*(17%8))*blocks[17] && CounterY<(400-20*(17/8))*blocks[17] && CounterY>(380-20*(17/8))*blocks[17];
	wire B18 = CounterX > (200+20*(18%8))*blocks[18] && CounterX < (220+20*(18%8))*blocks[18] && CounterY<(400-20*(18/8))*blocks[18] && CounterY>(380-20*(18/8))*blocks[18];
	wire B19 = CounterX > (200+20*(19%8))*blocks[19] && CounterX < (220+20*(19%8))*blocks[19] && CounterY<(400-20*(19/8))*blocks[19] && CounterY>(380-20*(19/8))*blocks[19];
	wire B20 = CounterX > (200+20*(20%8))*blocks[20] && CounterX < (220+20*(20%8))*blocks[20] && CounterY<(400-20*(20/8))*blocks[20] && CounterY>(380-20*(20/8))*blocks[20];
	wire B21 = CounterX > (200+20*(21%8))*blocks[21] && CounterX < (220+20*(21%8))*blocks[21] && CounterY<(400-20*(21/8))*blocks[21] && CounterY>(380-20*(21/8))*blocks[21];
	wire B22 = CounterX > (200+20*(22%8))*blocks[22] && CounterX < (220+20*(22%8))*blocks[22] && CounterY<(400-20*(22/8))*blocks[22] && CounterY>(380-20*(22/8))*blocks[22];
	wire B23 = CounterX > (200+20*(23%8))*blocks[23] && CounterX < (220+20*(23%8))*blocks[23] && CounterY<(400-20*(23/8))*blocks[23] && CounterY>(380-20*(23/8))*blocks[23];
	wire B24 = CounterX > (200+20*(24%8))*blocks[24] && CounterX < (220+20*(24%8))*blocks[24] && CounterY<(400-20*(24/8))*blocks[24] && CounterY>(380-20*(24/8))*blocks[24];
	wire B25 = CounterX > (200+20*(25%8))*blocks[25] && CounterX < (220+20*(25%8))*blocks[25] && CounterY<(400-20*(25/8))*blocks[25] && CounterY>(380-20*(25/8))*blocks[25];
	wire B26 = CounterX > (200+20*(26%8))*blocks[26] && CounterX < (220+20*(26%8))*blocks[26] && CounterY<(400-20*(26/8))*blocks[26] && CounterY>(380-20*(26/8))*blocks[26];
	wire B27 = CounterX > (200+20*(27%8))*blocks[27] && CounterX < (220+20*(27%8))*blocks[27] && CounterY<(400-20*(27/8))*blocks[27] && CounterY>(380-20*(27/8))*blocks[27];
	wire B28 = CounterX > (200+20*(28%8))*blocks[28] && CounterX < (220+20*(28%8))*blocks[28] && CounterY<(400-20*(28/8))*blocks[28] && CounterY>(380-20*(28/8))*blocks[28];
	wire B29 = CounterX > (200+20*(29%8))*blocks[29] && CounterX < (220+20*(29%8))*blocks[29] && CounterY<(400-20*(29/8))*blocks[29] && CounterY>(380-20*(29/8))*blocks[29];
	wire B30 = CounterX > (200+20*(30%8))*blocks[30] && CounterX < (220+20*(30%8))*blocks[30] && CounterY<(400-20*(30/8))*blocks[30] && CounterY>(380-20*(30/8))*blocks[30];
	wire B31 = CounterX > (200+20*(31%8))*blocks[31] && CounterX < (220+20*(31%8))*blocks[31] && CounterY<(400-20*(31/8))*blocks[31] && CounterY>(380-20*(31/8))*blocks[31];
	wire B32 = CounterX > (200+20*(32%8))*blocks[32] && CounterX < (220+20*(32%8))*blocks[32] && CounterY<(400-20*(32/8))*blocks[32] && CounterY>(380-20*(32/8))*blocks[32];
	wire B33 = CounterX > (200+20*(33%8))*blocks[33] && CounterX < (220+20*(33%8))*blocks[33] && CounterY<(400-20*(33/8))*blocks[33] && CounterY>(380-20*(33/8))*blocks[33];
	wire B34 = CounterX > (200+20*(34%8))*blocks[34] && CounterX < (220+20*(34%8))*blocks[34] && CounterY<(400-20*(34/8))*blocks[34] && CounterY>(380-20*(34/8))*blocks[34];
	wire B35 = CounterX > (200+20*(35%8))*blocks[35] && CounterX < (220+20*(35%8))*blocks[35] && CounterY<(400-20*(35/8))*blocks[35] && CounterY>(380-20*(35/8))*blocks[35];
	wire B36 = CounterX > (200+20*(36%8))*blocks[36] && CounterX < (220+20*(36%8))*blocks[36] && CounterY<(400-20*(36/8))*blocks[36] && CounterY>(380-20*(36/8))*blocks[36];
	wire B37 = CounterX > (200+20*(37%8))*blocks[37] && CounterX < (220+20*(37%8))*blocks[37] && CounterY<(400-20*(37/8))*blocks[37] && CounterY>(380-20*(37/8))*blocks[37];
	wire B38 = CounterX > (200+20*(38%8))*blocks[38] && CounterX < (220+20*(38%8))*blocks[38] && CounterY<(400-20*(38/8))*blocks[38] && CounterY>(380-20*(38/8))*blocks[38];
	wire B39 = CounterX > (200+20*(39%8))*blocks[39] && CounterX < (220+20*(39%8))*blocks[39] && CounterY<(400-20*(39/8))*blocks[39] && CounterY>(380-20*(39/8))*blocks[39];
	wire B40 = CounterX > (200+20*(40%8))*blocks[40] && CounterX < (220+20*(40%8))*blocks[40] && CounterY<(400-20*(40/8))*blocks[40] && CounterY>(380-20*(40/8))*blocks[40];
	wire B41 = CounterX > (200+20*(41%8))*blocks[41] && CounterX < (220+20*(41%8))*blocks[41] && CounterY<(400-20*(41/8))*blocks[41] && CounterY>(380-20*(41/8))*blocks[41];
	wire B42 = CounterX > (200+20*(42%8))*blocks[42] && CounterX < (220+20*(42%8))*blocks[42] && CounterY<(400-20*(42/8))*blocks[42] && CounterY>(380-20*(42/8))*blocks[42];
	wire B43 = CounterX > (200+20*(43%8))*blocks[43] && CounterX < (220+20*(43%8))*blocks[43] && CounterY<(400-20*(43/8))*blocks[43] && CounterY>(380-20*(43/8))*blocks[43];
	wire B44 = CounterX > (200+20*(44%8))*blocks[44] && CounterX < (220+20*(44%8))*blocks[44] && CounterY<(400-20*(44/8))*blocks[44] && CounterY>(380-20*(44/8))*blocks[44];
	wire B45 = CounterX > (200+20*(45%8))*blocks[45] && CounterX < (220+20*(45%8))*blocks[45] && CounterY<(400-20*(45/8))*blocks[45] && CounterY>(380-20*(45/8))*blocks[45];
	wire B46 = CounterX > (200+20*(46%8))*blocks[46] && CounterX < (220+20*(46%8))*blocks[46] && CounterY<(400-20*(46/8))*blocks[46] && CounterY>(380-20*(46/8))*blocks[46];
	wire B47 = CounterX > (200+20*(47%8))*blocks[47] && CounterX < (220+20*(47%8))*blocks[47] && CounterY<(400-20*(47/8))*blocks[47] && CounterY>(380-20*(47/8))*blocks[47];
	wire B48 = CounterX > (200+20*(48%8))*blocks[48] && CounterX < (220+20*(48%8))*blocks[48] && CounterY<(400-20*(48/8))*blocks[48] && CounterY>(380-20*(48/8))*blocks[48];
	wire B49 = CounterX > (200+20*(49%8))*blocks[49] && CounterX < (220+20*(49%8))*blocks[49] && CounterY<(400-20*(49/8))*blocks[49] && CounterY>(380-20*(49/8))*blocks[49];
	wire B50 = CounterX > (200+20*(50%8))*blocks[50] && CounterX < (220+20*(50%8))*blocks[50] && CounterY<(400-20*(50/8))*blocks[50] && CounterY>(380-20*(50/8))*blocks[50];
	wire B51 = CounterX > (200+20*(51%8))*blocks[51] && CounterX < (220+20*(51%8))*blocks[51] && CounterY<(400-20*(51/8))*blocks[51] && CounterY>(380-20*(51/8))*blocks[51];
	wire B52 = CounterX > (200+20*(52%8))*blocks[52] && CounterX < (220+20*(52%8))*blocks[52] && CounterY<(400-20*(52/8))*blocks[52] && CounterY>(380-20*(52/8))*blocks[52];
	wire B53 = CounterX > (200+20*(53%8))*blocks[53] && CounterX < (220+20*(53%8))*blocks[53] && CounterY<(400-20*(53/8))*blocks[53] && CounterY>(380-20*(53/8))*blocks[53];
	wire B54 = CounterX > (200+20*(54%8))*blocks[54] && CounterX < (220+20*(54%8))*blocks[54] && CounterY<(400-20*(54/8))*blocks[54] && CounterY>(380-20*(54/8))*blocks[54];
	wire B55 = CounterX > (200+20*(55%8))*blocks[55] && CounterX < (220+20*(55%8))*blocks[55] && CounterY<(400-20*(55/8))*blocks[55] && CounterY>(380-20*(55/8))*blocks[55];
	wire B56 = CounterX > (200+20*(56%8))*blocks[56] && CounterX < (220+20*(56%8))*blocks[56] && CounterY<(400-20*(56/8))*blocks[56] && CounterY>(380-20*(56/8))*blocks[56];
	wire B57 = CounterX > (200+20*(57%8))*blocks[57] && CounterX < (220+20*(57%8))*blocks[57] && CounterY<(400-20*(57/8))*blocks[57] && CounterY>(380-20*(57/8))*blocks[57];
	wire B58 = CounterX > (200+20*(58%8))*blocks[58] && CounterX < (220+20*(58%8))*blocks[58] && CounterY<(400-20*(58/8))*blocks[58] && CounterY>(380-20*(58/8))*blocks[58];
	wire B59 = CounterX > (200+20*(59%8))*blocks[59] && CounterX < (220+20*(59%8))*blocks[59] && CounterY<(400-20*(59/8))*blocks[59] && CounterY>(380-20*(59/8))*blocks[59];
	wire B60 = CounterX > (200+20*(60%8))*blocks[60] && CounterX < (220+20*(60%8))*blocks[60] && CounterY<(400-20*(60/8))*blocks[60] && CounterY>(380-20*(60/8))*blocks[60];
	wire B61 = CounterX > (200+20*(61%8))*blocks[61] && CounterX < (220+20*(61%8))*blocks[61] && CounterY<(400-20*(61/8))*blocks[61] && CounterY>(380-20*(61/8))*blocks[61];
	wire B62 = CounterX > (200+20*(62%8))*blocks[62] && CounterX < (220+20*(62%8))*blocks[62] && CounterY<(400-20*(62/8))*blocks[62] && CounterY>(380-20*(62/8))*blocks[62];
	wire B63 = CounterX > (200+20*(63%8))*blocks[63] && CounterX < (220+20*(63%8))*blocks[63] && CounterY<(400-20*(63/8))*blocks[63] && CounterY>(380-20*(63/8))*blocks[63];
	wire B64 = CounterX > (200+20*(64%8))*blocks[64] && CounterX < (220+20*(64%8))*blocks[64] && CounterY<(400-20*(64/8))*blocks[64] && CounterY>(380-20*(64/8))*blocks[64];
	wire B65 = CounterX > (200+20*(65%8))*blocks[65] && CounterX < (220+20*(65%8))*blocks[65] && CounterY<(400-20*(65/8))*blocks[65] && CounterY>(380-20*(65/8))*blocks[65];
	wire B66 = CounterX > (200+20*(66%8))*blocks[66] && CounterX < (220+20*(66%8))*blocks[66] && CounterY<(400-20*(66/8))*blocks[66] && CounterY>(380-20*(66/8))*blocks[66];
	wire B67 = CounterX > (200+20*(67%8))*blocks[67] && CounterX < (220+20*(67%8))*blocks[67] && CounterY<(400-20*(67/8))*blocks[67] && CounterY>(380-20*(67/8))*blocks[67];
	wire B68 = CounterX > (200+20*(68%8))*blocks[68] && CounterX < (220+20*(68%8))*blocks[68] && CounterY<(400-20*(68/8))*blocks[68] && CounterY>(380-20*(68/8))*blocks[68];
	wire B69 = CounterX > (200+20*(69%8))*blocks[69] && CounterX < (220+20*(69%8))*blocks[69] && CounterY<(400-20*(69/8))*blocks[69] && CounterY>(380-20*(69/8))*blocks[69];
	wire B70 = CounterX > (200+20*(70%8))*blocks[70] && CounterX < (220+20*(70%8))*blocks[70] && CounterY<(400-20*(70/8))*blocks[70] && CounterY>(380-20*(70/8))*blocks[70];
	wire B71 = CounterX > (200+20*(71%8))*blocks[71] && CounterX < (220+20*(71%8))*blocks[71] && CounterY<(400-20*(71/8))*blocks[71] && CounterY>(380-20*(71/8))*blocks[71];
	wire B72 = CounterX > (200+20*(72%8))*blocks[72] && CounterX < (220+20*(72%8))*blocks[72] && CounterY<(400-20*(72/8))*blocks[72] && CounterY>(380-20*(72/8))*blocks[72];
	wire B73 = CounterX > (200+20*(73%8))*blocks[73] && CounterX < (220+20*(73%8))*blocks[73] && CounterY<(400-20*(73/8))*blocks[73] && CounterY>(380-20*(73/8))*blocks[73];
	wire B74 = CounterX > (200+20*(74%8))*blocks[74] && CounterX < (220+20*(74%8))*blocks[74] && CounterY<(400-20*(74/8))*blocks[74] && CounterY>(380-20*(74/8))*blocks[74];
	wire B75 = CounterX > (200+20*(75%8))*blocks[75] && CounterX < (220+20*(75%8))*blocks[75] && CounterY<(400-20*(75/8))*blocks[75] && CounterY>(380-20*(75/8))*blocks[75];
	wire B76 = CounterX > (200+20*(76%8))*blocks[76] && CounterX < (220+20*(76%8))*blocks[76] && CounterY<(400-20*(76/8))*blocks[76] && CounterY>(380-20*(76/8))*blocks[76];
	wire B77 = CounterX > (200+20*(77%8))*blocks[77] && CounterX < (220+20*(77%8))*blocks[77] && CounterY<(400-20*(77/8))*blocks[77] && CounterY>(380-20*(77/8))*blocks[77];
	wire B78 = CounterX > (200+20*(78%8))*blocks[78] && CounterX < (220+20*(78%8))*blocks[78] && CounterY<(400-20*(78/8))*blocks[78] && CounterY>(380-20*(78/8))*blocks[78];
	wire B79 = CounterX > (200+20*(79%8))*blocks[79] && CounterX < (220+20*(79%8))*blocks[79] && CounterY<(400-20*(79/8))*blocks[79] && CounterY>(380-20*(79/8))*blocks[79];
	wire B80 = CounterX > (200+20*(80%8))*blocks[80] && CounterX < (220+20*(80%8))*blocks[80] && CounterY<(400-20*(80/8))*blocks[80] && CounterY>(380-20*(80/8))*blocks[80];
	wire B81 = CounterX > (200+20*(81%8))*blocks[81] && CounterX < (220+20*(81%8))*blocks[81] && CounterY<(400-20*(81/8))*blocks[81] && CounterY>(380-20*(81/8))*blocks[81];
	wire B82 = CounterX > (200+20*(82%8))*blocks[82] && CounterX < (220+20*(82%8))*blocks[82] && CounterY<(400-20*(82/8))*blocks[82] && CounterY>(380-20*(82/8))*blocks[82];
	wire B83 = CounterX > (200+20*(83%8))*blocks[83] && CounterX < (220+20*(83%8))*blocks[83] && CounterY<(400-20*(83/8))*blocks[83] && CounterY>(380-20*(83/8))*blocks[83];
	wire B84 = CounterX > (200+20*(84%8))*blocks[84] && CounterX < (220+20*(84%8))*blocks[84] && CounterY<(400-20*(84/8))*blocks[84] && CounterY>(380-20*(84/8))*blocks[84];
	wire B85 = CounterX > (200+20*(85%8))*blocks[85] && CounterX < (220+20*(85%8))*blocks[85] && CounterY<(400-20*(85/8))*blocks[85] && CounterY>(380-20*(85/8))*blocks[85];
	wire B86 = CounterX > (200+20*(86%8))*blocks[86] && CounterX < (220+20*(86%8))*blocks[86] && CounterY<(400-20*(86/8))*blocks[86] && CounterY>(380-20*(86/8))*blocks[86];
	wire B87 = CounterX > (200+20*(87%8))*blocks[87] && CounterX < (220+20*(87%8))*blocks[87] && CounterY<(400-20*(87/8))*blocks[87] && CounterY>(380-20*(87/8))*blocks[87];
	wire B88 = CounterX > (200+20*(88%8))*blocks[88] && CounterX < (220+20*(88%8))*blocks[88] && CounterY<(400-20*(88/8))*blocks[88] && CounterY>(380-20*(88/8))*blocks[88];
	wire B89 = CounterX > (200+20*(89%8))*blocks[89] && CounterX < (220+20*(89%8))*blocks[89] && CounterY<(400-20*(89/8))*blocks[89] && CounterY>(380-20*(89/8))*blocks[89];
	wire B90 = CounterX > (200+20*(90%8))*blocks[90] && CounterX < (220+20*(90%8))*blocks[90] && CounterY<(400-20*(90/8))*blocks[90] && CounterY>(380-20*(90/8))*blocks[90];
	wire B91 = CounterX > (200+20*(91%8))*blocks[91] && CounterX < (220+20*(91%8))*blocks[91] && CounterY<(400-20*(91/8))*blocks[91] && CounterY>(380-20*(91/8))*blocks[91];
	wire B92 = CounterX > (200+20*(92%8))*blocks[92] && CounterX < (220+20*(92%8))*blocks[92] && CounterY<(400-20*(92/8))*blocks[92] && CounterY>(380-20*(92/8))*blocks[92];
	wire B93 = CounterX > (200+20*(93%8))*blocks[93] && CounterX < (220+20*(93%8))*blocks[93] && CounterY<(400-20*(93/8))*blocks[93] && CounterY>(380-20*(93/8))*blocks[93];
	wire B94 = CounterX > (200+20*(94%8))*blocks[94] && CounterX < (220+20*(94%8))*blocks[94] && CounterY<(400-20*(94/8))*blocks[94] && CounterY>(380-20*(94/8))*blocks[94];
	wire B95 = CounterX > (200+20*(95%8))*blocks[95] && CounterX < (220+20*(95%8))*blocks[95] && CounterY<(400-20*(95/8))*blocks[95] && CounterY>(380-20*(95/8))*blocks[95];
	wire B96 = CounterX > (200+20*(96%8))*blocks[96] && CounterX < (220+20*(96%8))*blocks[96] && CounterY<(400-20*(96/8))*blocks[96] && CounterY>(380-20*(96/8))*blocks[96];
	wire B97 = CounterX > (200+20*(97%8))*blocks[97] && CounterX < (220+20*(97%8))*blocks[97] && CounterY<(400-20*(97/8))*blocks[97] && CounterY>(380-20*(97/8))*blocks[97];
	wire B98 = CounterX > (200+20*(98%8))*blocks[98] && CounterX < (220+20*(98%8))*blocks[98] && CounterY<(400-20*(98/8))*blocks[98] && CounterY>(380-20*(98/8))*blocks[98];
	wire B99 = CounterX > (200+20*(99%8))*blocks[99] && CounterX < (220+20*(99%8))*blocks[99] && CounterY<(400-20*(99/8))*blocks[99] && CounterY>(380-20*(99/8))*blocks[99];
	wire B100 = CounterX > (200+20*(100%8))*blocks[100] && CounterX < (220+20*(100%8))*blocks[100] && CounterY<(400-20*(100/8))*blocks[100] && CounterY>(380-20*(100/8))*blocks[100];
	wire B101 = CounterX > (200+20*(101%8))*blocks[101] && CounterX < (220+20*(101%8))*blocks[101] && CounterY<(400-20*(101/8))*blocks[101] && CounterY>(380-20*(101/8))*blocks[101];
	wire B102 = CounterX > (200+20*(102%8))*blocks[102] && CounterX < (220+20*(102%8))*blocks[102] && CounterY<(400-20*(102/8))*blocks[102] && CounterY>(380-20*(102/8))*blocks[102];
	wire B103 = CounterX > (200+20*(103%8))*blocks[103] && CounterX < (220+20*(103%8))*blocks[103] && CounterY<(400-20*(103/8))*blocks[103] && CounterY>(380-20*(103/8))*blocks[103];
	wire B104 = CounterX > (200+20*(104%8))*blocks[104] && CounterX < (220+20*(104%8))*blocks[104] && CounterY<(400-20*(104/8))*blocks[104] && CounterY>(380-20*(104/8))*blocks[104];
	wire B105 = CounterX > (200+20*(105%8))*blocks[105] && CounterX < (220+20*(105%8))*blocks[105] && CounterY<(400-20*(105/8))*blocks[105] && CounterY>(380-20*(105/8))*blocks[105];
	wire B106 = CounterX > (200+20*(106%8))*blocks[106] && CounterX < (220+20*(106%8))*blocks[106] && CounterY<(400-20*(106/8))*blocks[106] && CounterY>(380-20*(106/8))*blocks[106];
	wire B107 = CounterX > (200+20*(107%8))*blocks[107] && CounterX < (220+20*(107%8))*blocks[107] && CounterY<(400-20*(107/8))*blocks[107] && CounterY>(380-20*(107/8))*blocks[107];
	wire B108 = CounterX > (200+20*(108%8))*blocks[108] && CounterX < (220+20*(108%8))*blocks[108] && CounterY<(400-20*(108/8))*blocks[108] && CounterY>(380-20*(108/8))*blocks[108];
	wire B109 = CounterX > (200+20*(109%8))*blocks[109] && CounterX < (220+20*(109%8))*blocks[109] && CounterY<(400-20*(109/8))*blocks[109] && CounterY>(380-20*(109/8))*blocks[109];
	wire B110 = CounterX > (200+20*(110%8))*blocks[110] && CounterX < (220+20*(110%8))*blocks[110] && CounterY<(400-20*(110/8))*blocks[110] && CounterY>(380-20*(110/8))*blocks[110];
	wire B111 = CounterX > (200+20*(111%8))*blocks[111] && CounterX < (220+20*(111%8))*blocks[111] && CounterY<(400-20*(111/8))*blocks[111] && CounterY>(380-20*(111/8))*blocks[111];
	wire B112 = CounterX > (200+20*(112%8))*blocks[112] && CounterX < (220+20*(112%8))*blocks[112] && CounterY<(400-20*(112/8))*blocks[112] && CounterY>(380-20*(112/8))*blocks[112];
	wire B113 = CounterX > (200+20*(113%8))*blocks[113] && CounterX < (220+20*(113%8))*blocks[113] && CounterY<(400-20*(113/8))*blocks[113] && CounterY>(380-20*(113/8))*blocks[113];
	wire B114 = CounterX > (200+20*(114%8))*blocks[114] && CounterX < (220+20*(114%8))*blocks[114] && CounterY<(400-20*(114/8))*blocks[114] && CounterY>(380-20*(114/8))*blocks[114];
	wire B115 = CounterX > (200+20*(115%8))*blocks[115] && CounterX < (220+20*(115%8))*blocks[115] && CounterY<(400-20*(115/8))*blocks[115] && CounterY>(380-20*(115/8))*blocks[115];
	wire B116 = CounterX > (200+20*(116%8))*blocks[116] && CounterX < (220+20*(116%8))*blocks[116] && CounterY<(400-20*(116/8))*blocks[116] && CounterY>(380-20*(116/8))*blocks[116];
	wire B117 = CounterX > (200+20*(117%8))*blocks[117] && CounterX < (220+20*(117%8))*blocks[117] && CounterY<(400-20*(117/8))*blocks[117] && CounterY>(380-20*(117/8))*blocks[117];
	wire B118 = CounterX > (200+20*(118%8))*blocks[118] && CounterX < (220+20*(118%8))*blocks[118] && CounterY<(400-20*(118/8))*blocks[118] && CounterY>(380-20*(118/8))*blocks[118];
	wire B119 = CounterX > (200+20*(119%8))*blocks[119] && CounterX < (220+20*(119%8))*blocks[119] && CounterY<(400-20*(119/8))*blocks[119] && CounterY>(380-20*(119/8))*blocks[119];
	wire B120 = CounterX > (200+20*(120%8))*blocks[120] && CounterX < (220+20*(120%8))*blocks[120] && CounterY<(400-20*(120/8))*blocks[120] && CounterY>(380-20*(120/8))*blocks[120];
	wire B121 = CounterX > (200+20*(121%8))*blocks[121] && CounterX < (220+20*(121%8))*blocks[121] && CounterY<(400-20*(121/8))*blocks[121] && CounterY>(380-20*(121/8))*blocks[121];
	wire B122 = CounterX > (200+20*(122%8))*blocks[122] && CounterX < (220+20*(122%8))*blocks[122] && CounterY<(400-20*(122/8))*blocks[122] && CounterY>(380-20*(122/8))*blocks[122];
	wire B123 = CounterX > (200+20*(123%8))*blocks[123] && CounterX < (220+20*(123%8))*blocks[123] && CounterY<(400-20*(123/8))*blocks[123] && CounterY>(380-20*(123/8))*blocks[123];
	wire B124 = CounterX > (200+20*(124%8))*blocks[124] && CounterX < (220+20*(124%8))*blocks[124] && CounterY<(400-20*(124/8))*blocks[124] && CounterY>(380-20*(124/8))*blocks[124];
	wire B125 = CounterX > (200+20*(125%8))*blocks[125] && CounterX < (220+20*(125%8))*blocks[125] && CounterY<(400-20*(125/8))*blocks[125] && CounterY>(380-20*(125/8))*blocks[125];
	wire B126 = CounterX > (200+20*(126%8))*blocks[126] && CounterX < (220+20*(126%8))*blocks[126] && CounterY<(400-20*(126/8))*blocks[126] && CounterY>(380-20*(126/8))*blocks[126];
	wire B127 = CounterX > (200+20*(127%8))*blocks[127] && CounterX < (220+20*(127%8))*blocks[127] && CounterY<(400-20*(127/8))*blocks[127] && CounterY>(380-20*(127/8))*blocks[127];
	wire B128 = CounterX > (200+20*(128%8))*blocks[128] && CounterX < (220+20*(128%8))*blocks[128] && CounterY<(400-20*(128/8))*blocks[128] && CounterY>(380-20*(128/8))*blocks[128];
	wire B129 = CounterX > (200+20*(129%8))*blocks[129] && CounterX < (220+20*(129%8))*blocks[129] && CounterY<(400-20*(129/8))*blocks[129] && CounterY>(380-20*(129/8))*blocks[129];
	wire B130 = CounterX > (200+20*(130%8))*blocks[130] && CounterX < (220+20*(130%8))*blocks[130] && CounterY<(400-20*(130/8))*blocks[130] && CounterY>(380-20*(130/8))*blocks[130];
	wire B131 = CounterX > (200+20*(131%8))*blocks[131] && CounterX < (220+20*(131%8))*blocks[131] && CounterY<(400-20*(131/8))*blocks[131] && CounterY>(380-20*(131/8))*blocks[131];
	wire B132 = CounterX > (200+20*(132%8))*blocks[132] && CounterX < (220+20*(132%8))*blocks[132] && CounterY<(400-20*(132/8))*blocks[132] && CounterY>(380-20*(132/8))*blocks[132];
	wire B133 = CounterX > (200+20*(133%8))*blocks[133] && CounterX < (220+20*(133%8))*blocks[133] && CounterY<(400-20*(133/8))*blocks[133] && CounterY>(380-20*(133/8))*blocks[133];
	wire B134 = CounterX > (200+20*(134%8))*blocks[134] && CounterX < (220+20*(134%8))*blocks[134] && CounterY<(400-20*(134/8))*blocks[134] && CounterY>(380-20*(134/8))*blocks[134];
	wire B135 = CounterX > (200+20*(135%8))*blocks[135] && CounterX < (220+20*(135%8))*blocks[135] && CounterY<(400-20*(135/8))*blocks[135] && CounterY>(380-20*(135/8))*blocks[135];
	wire B136 = CounterX > (200+20*(136%8))*blocks[136] && CounterX < (220+20*(136%8))*blocks[136] && CounterY<(400-20*(136/8))*blocks[136] && CounterY>(380-20*(136/8))*blocks[136];
	wire B137 = CounterX > (200+20*(137%8))*blocks[137] && CounterX < (220+20*(137%8))*blocks[137] && CounterY<(400-20*(137/8))*blocks[137] && CounterY>(380-20*(137/8))*blocks[137];
	wire B138 = CounterX > (200+20*(138%8))*blocks[138] && CounterX < (220+20*(138%8))*blocks[138] && CounterY<(400-20*(138/8))*blocks[138] && CounterY>(380-20*(138/8))*blocks[138];
	wire B139 = CounterX > (200+20*(139%8))*blocks[139] && CounterX < (220+20*(139%8))*blocks[139] && CounterY<(400-20*(139/8))*blocks[139] && CounterY>(380-20*(139/8))*blocks[139];
	wire B140 = CounterX > (200+20*(140%8))*blocks[140] && CounterX < (220+20*(140%8))*blocks[140] && CounterY<(400-20*(140/8))*blocks[140] && CounterY>(380-20*(140/8))*blocks[140];
	wire B141 = CounterX > (200+20*(141%8))*blocks[141] && CounterX < (220+20*(141%8))*blocks[141] && CounterY<(400-20*(141/8))*blocks[141] && CounterY>(380-20*(141/8))*blocks[141];
	wire B142 = CounterX > (200+20*(142%8))*blocks[142] && CounterX < (220+20*(142%8))*blocks[142] && CounterY<(400-20*(142/8))*blocks[142] && CounterY>(380-20*(142/8))*blocks[142];
	wire B143 = CounterX > (200+20*(143%8))*blocks[143] && CounterX < (220+20*(143%8))*blocks[143] && CounterY<(400-20*(143/8))*blocks[143] && CounterY>(380-20*(143/8))*blocks[143];
	wire B144 = CounterX > (200+20*(144%8))*blocks[144] && CounterX < (220+20*(144%8))*blocks[144] && CounterY<(400-20*(144/8))*blocks[144] && CounterY>(380-20*(144/8))*blocks[144];
	wire B145 = CounterX > (200+20*(145%8))*blocks[145] && CounterX < (220+20*(145%8))*blocks[145] && CounterY<(400-20*(145/8))*blocks[145] && CounterY>(380-20*(145/8))*blocks[145];
	wire B146 = CounterX > (200+20*(146%8))*blocks[146] && CounterX < (220+20*(146%8))*blocks[146] && CounterY<(400-20*(146/8))*blocks[146] && CounterY>(380-20*(146/8))*blocks[146];
	wire B147 = CounterX > (200+20*(147%8))*blocks[147] && CounterX < (220+20*(147%8))*blocks[147] && CounterY<(400-20*(147/8))*blocks[147] && CounterY>(380-20*(147/8))*blocks[147];
	wire B148 = CounterX > (200+20*(148%8))*blocks[148] && CounterX < (220+20*(148%8))*blocks[148] && CounterY<(400-20*(148/8))*blocks[148] && CounterY>(380-20*(148/8))*blocks[148];
	wire B149 = CounterX > (200+20*(149%8))*blocks[149] && CounterX < (220+20*(149%8))*blocks[149] && CounterY<(400-20*(149/8))*blocks[149] && CounterY>(380-20*(149/8))*blocks[149];
	wire B150 = CounterX > (200+20*(150%8))*blocks[150] && CounterX < (220+20*(150%8))*blocks[150] && CounterY<(400-20*(150/8))*blocks[150] && CounterY>(380-20*(150/8))*blocks[150];
	wire B151 = CounterX > (200+20*(151%8))*blocks[151] && CounterX < (220+20*(151%8))*blocks[151] && CounterY<(400-20*(151/8))*blocks[151] && CounterY>(380-20*(151/8))*blocks[151];
	wire B152 = CounterX > (200+20*(152%8))*blocks[152] && CounterX < (220+20*(152%8))*blocks[152] && CounterY<(400-20*(152/8))*blocks[152] && CounterY>(380-20*(152/8))*blocks[152];
	wire B153 = CounterX > (200+20*(153%8))*blocks[153] && CounterX < (220+20*(153%8))*blocks[153] && CounterY<(400-20*(153/8))*blocks[153] && CounterY>(380-20*(153/8))*blocks[153];
	wire B154 = CounterX > (200+20*(154%8))*blocks[154] && CounterX < (220+20*(154%8))*blocks[154] && CounterY<(400-20*(154/8))*blocks[154] && CounterY>(380-20*(154/8))*blocks[154];
	wire B155 = CounterX > (200+20*(155%8))*blocks[155] && CounterX < (220+20*(155%8))*blocks[155] && CounterY<(400-20*(155/8))*blocks[155] && CounterY>(380-20*(155/8))*blocks[155];
	wire B156 = CounterX > (200+20*(156%8))*blocks[156] && CounterX < (220+20*(156%8))*blocks[156] && CounterY<(400-20*(156/8))*blocks[156] && CounterY>(380-20*(156/8))*blocks[156];
	wire B157 = CounterX > (200+20*(157%8))*blocks[157] && CounterX < (220+20*(157%8))*blocks[157] && CounterY<(400-20*(157/8))*blocks[157] && CounterY>(380-20*(157/8))*blocks[157];
	wire B158 = CounterX > (200+20*(158%8))*blocks[158] && CounterX < (220+20*(158%8))*blocks[158] && CounterY<(400-20*(158/8))*blocks[158] && CounterY>(380-20*(158/8))*blocks[158];
	wire B159 = CounterX > (200+20*(159%8))*blocks[159] && CounterX < (220+20*(159%8))*blocks[159] && CounterY<(400-20*(159/8))*blocks[159] && CounterY>(380-20*(159/8))*blocks[159];


	
	always @(posedge clk)
	begin
		vga_r <= R & inDisplayArea;
		vga_g <= G & inDisplayArea;
		vga_b <= B | B1 | B2 | B3 | B4 | B5 | B6 | B7 | B8 | B9 
		| B10 
		| B11
		| B12 
		| B13 
		| B14 
		| B15 
		| B16 
		| B17 
		| B18 
		| B19 
		| B20 
		| B21 
		| B22 
		| B23 
		| B24 
		| B25 
		| B26 
		| B27 
		| B28 
		| B29 
		| B30 
		| B31 
		| B32 
		| B33 
		| B34 
		| B35 
		| B36 
		| B37 
		| B38 
		| B39 
		| B40 
		| B41 
		| B42 
		| B43 
		| B44 
		| B45 
		| B46 
		| B47 
		| B48 
		| B49 
		| B50 
		| B51 
		| B52 
		| B53 
		| B54 
		| B55 
		| B56 
		| B57 
		| B58 
		| B59 
		| B60 
		| B61 
		| B62 
		| B63 
		| B64 
		| B65 
		| B66 
		| B67 
		| B68 
		| B69 
		| B70 
		| B71 
		| B72 
		| B73 
		| B74 
		| B75 
		| B76 
		| B77 
		| B78 
		| B79 
		| B80 
		| B81 
		| B82 
		| B83 
		| B84 
		| B85 
		| B86 
		| B87 
		| B88 
		| B89 
		| B80 
		| B81 
		| B82 
		| B83 
		| B84 
		| B85 
		| B86 
		| B87 
		| B88 
		| B89 
		| B90 
		| B91 
		| B92 
		| B93 
		| B94 
		| B95 
		| B96 
		| B97 
		| B98 
		| B99 
		| B100
		| B101
		| B102
		| B103
		| B104
		| B105
		| B106
		| B107
		| B108
		| B109
		| B110
		| B111
		| B112
		| B113
		| B114
		| B115
		| B116
		| B117
		| B118
		| B119
		| B120
		| B121
		| B122
		| B123
		| B124
		| B125
		| B126
		| B127
		| B128
		| B129
		| B130
		| B131
		| B132
		| B133
		| B134
		| B135
		| B136
		| B137
		| B138
		| B139
		| B140
		| B141
		| B142
		| B143
		| B144
		| B145
		| B146
		| B147
		| B148
		| B149
		| B150
		| B151
		| B152
		| B153
		| B154
		| B155
		| B156
		| B157
		| B158
		| B159 
 		& inDisplayArea;
	end
	
	/////////////////////////////////////////////////////////////////
	//////////////  	  VGA control ends here 	 ///////////////////
	/////////////////////////////////////////////////////////////////
	
	/////////////////////////////////////////////////////////////////
	//////////////  	  LD control starts here 	 ///////////////////
	/////////////////////////////////////////////////////////////////
	`define QI 			2'b00
	`define QGAME_1 	2'b01
	`define QGAME_2 	2'b10
	`define QDONE 		2'b11
	
	reg [3:0] p2_score;
	reg [3:0] p1_score;
	reg [1:0] state;
	wire LD0, LD1, LD2, LD3, LD4, LD5, LD6, LD7;
	
	assign LD0 = (p1_score == 4'b1010);
	assign LD1 = (p2_score == 4'b1010);
	
	assign LD2 = start;
	assign LD4 = reset;
	
	assign LD3 = (state == `QI);
	assign LD5 = (state == `QGAME_1);	
	assign LD6 = (state == `QGAME_2);
	assign LD7 = (state == `QDONE);
	
	/////////////////////////////////////////////////////////////////
	//////////////  	  LD control ends here 	 	////////////////////
	/////////////////////////////////////////////////////////////////
	
	/////////////////////////////////////////////////////////////////
	//////////////  	  SSD control starts here 	 ///////////////////
	/////////////////////////////////////////////////////////////////
	reg 	[3:0]	SSD;
	wire 	[3:0]	SSD0, SSD1, SSD2, SSD3;
	wire 	[1:0] ssdscan_clk;
	
	assign SSD3 = score[15:12];
	assign SSD2 = score[11:8];
	assign SSD1 = score[7:4];
	assign SSD0 = score[3:0];
	
	// need a scan clk for the seven segment display 
	// 191Hz (50MHz / 2^18) works well
	assign ssdscan_clk = DIV_CLK[19:18];	
	assign An0	= !(~(ssdscan_clk[1]) && ~(ssdscan_clk[0]));  // when ssdscan_clk = 00
	assign An1	= !(~(ssdscan_clk[1]) &&  (ssdscan_clk[0]));  // when ssdscan_clk = 01
	assign An2	= !( (ssdscan_clk[1]) && ~(ssdscan_clk[0]));  // when ssdscan_clk = 10
	assign An3	= !( (ssdscan_clk[1]) &&  (ssdscan_clk[0]));  // when ssdscan_clk = 11
	
	always @ (ssdscan_clk, SSD0, SSD1, SSD2, SSD3)
	begin : SSD_SCAN_OUT
		case (ssdscan_clk) 
			2'b00:
					SSD = SSD0;
			2'b01:
					SSD = SSD1;
			2'b10:
					SSD = SSD2;
			2'b11:
					SSD = SSD3;
		endcase 
	end	

	// and finally convert SSD_num to ssd
	reg [6:0]  SSD_CATHODES;
	assign {Ca, Cb, Cc, Cd, Ce, Cf, Cg, Dp} = {SSD_CATHODES, 1'b1};
	// Following is Hex-to-SSD conversion
	always @ (SSD) 
	begin : HEX_TO_SSD
		case (SSD)		
			4'b1111: SSD_CATHODES = 7'b1111111 ; //Nothing 
			4'b0000: SSD_CATHODES = 7'b0000001 ; //0
			4'b0001: SSD_CATHODES = 7'b1001111 ; //1
			4'b0010: SSD_CATHODES = 7'b0010010 ; //2
			4'b0011: SSD_CATHODES = 7'b0000110 ; //3
			4'b0100: SSD_CATHODES = 7'b1001100 ; //4
			4'b0101: SSD_CATHODES = 7'b0100100 ; //5
			4'b0110: SSD_CATHODES = 7'b0100000 ; //6
			4'b0111: SSD_CATHODES = 7'b0001111 ; //7
			4'b1000: SSD_CATHODES = 7'b0000000 ; //8
			4'b1001: SSD_CATHODES = 7'b0000100 ; //9
			4'b1010: SSD_CATHODES = 7'b0001000 ; //10 or A
			default: SSD_CATHODES = 7'bXXXXXXX ; // default is not needed as we covered all cases
		endcase
	end
	
	/////////////////////////////////////////////////////////////////
	//////////////  	  SSD control ends here 	 ///////////////////
	/////////////////////////////////////////////////////////////////
endmodule
