`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    19:33:35 12/02/2013 
// Design Name: 
// Module Name:    tetris 
// Project Name:  EE201 Final Project 
// Target Devices: Diligent Spartan-6
// Tool versions: 
// Description: 
//
// Dependencies: Food
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//		COLLISION: The implementation of the full row clearing may cause problems in the top row. 
//
//
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 100 ps

module tetris( Reset, Clk, Start, Ack, Left, Right, Down, Rotate,
	q_I, q_Gen, q_Rot, q_Col, q_Lose, blocks, score, orientation, location, next_block
    );

input Reset, Clk;
input Start, Ack;	
input Left, Right, Down;
input Rotate;
	
output q_I, q_Gen;
output q_Rot, q_Col, q_Lose;


output reg [159:0] blocks;
output reg [7:0] score;
reg [7:0] state;

// Current Block Information
output reg [7:0] location;
integer i;
reg [2:0] block_type;
output reg [1:0] orientation;

output reg [2:0] next_block;

// Number of Loops for Rotate and Move
reg [24:0] loop;
reg [2:0] random_count;

wire [19:0] full_rows;

//assign full_rows[0] = blocks[0] && blocks[1] && blocks[2] && blocks[3] && blocks[4] && blocks[5] && blocks[6] && blocks[7] && blocks[8];
// for(integer i = 0; i<20; i=i+1)
	// begin
	// assign full_rows[i] = blocks[0 + 8*i] && blocks[1+ 8*i] && blocks[2+ 8*i] && blocks[3+ 8*i] && blocks[4+ 8*i] && blocks[5+ 8*i] && blocks[6+ 8*i] && blocks[7+ 8*i] && blocks[8+ 8*i];
	// end
// Check if space is avaliable for a rotate or move down Wire
// Square
wire square_l, square_r, square_d;
assign square_l = !blocks[location-2] && !blocks[location -10] && ((location-1)%8);
assign square_r = !blocks[location+1] && !blocks[location-7] && ((location+1)%8);
assign square_d = !blocks[location-16] && !blocks[location-17] && (location > 15) ;
//Bar
wire bar0_l, bar0_r, bar0_d, bar0_rot, bar1_l, bar1_r, bar1_d, bar1_rot;  // Two orientations  
assign bar0_l = !blocks[location-3] && ((location-2)%8);
assign bar0_r = !blocks[location+2] && ((location+2)%8);
assign bar0_d = !blocks[location-7] && !blocks[location-8] && !blocks[location-9] && !blocks[location-10] && location > 7; 
assign bar0_rot = (location/8 != 19) && !blocks[location+8] && !blocks[location-8] && !blocks[location-16] && (location >15);
assign bar1_l = !blocks[location-1] && !blocks[location-9] && !blocks[location-17] && !blocks[location+7] && location%8;
assign bar1_r = !blocks[location+1] && !blocks[location+9] && !blocks[location-7] && !blocks[location -15] && (location+1)%8;
assign bar1_d = !blocks[location-24] && (location > 23);
assign bar1_rot = !blocks[location +1] && !blocks[location-1] && !blocks[location-2] && (location+1)%8 && location%8;

wire s0_l, s0_r, s0_d, s0_rot;
assign s0_l = !blocks[location-1] && !blocks[location-10] && ((location-1)%8);
assign s0_r = !blocks[location-2] && !blocks[location-7] && ((location+2)%8);
assign s0_d = !blocks[location-7] && !blocks[location-16] && !blocks[location-17] && (location>16); 
assign s0_rot = (location/8 != 19) && !blocks[location+8] && !blocks[location-7];

wire s1_l, s1_r, s1_d, s1_rot;
assign s1_l = !blocks[location-1] && !blocks[location+7] && !blocks[location-8] && ((location)%8);
assign s1_r = !blocks[location+9] && !blocks[location+2] && !blocks[location-6] && ((location+2)%8);
assign s1_d = !blocks[location-15] && !blocks[location-8] && (location>16); 
assign s1_rot = !blocks[location-8] && !blocks[location-9];

wire z0_l, z0_r, z0_d, z0_rot;
assign z0_l = !blocks[location-9] && !blocks[location-2] && ((location-1)%8);
assign z0_r = !blocks[location+1] && !blocks[location-6] && ((location+2)%8);
assign z0_d = !blocks[location-9] && !blocks[location-16] && !blocks[location-15] && (location>16); 
assign z0_rot = (location/8 != 19) && !blocks[location+1] && !blocks[location+9];

wire z1_l, z1_r, z1_d, z1_rot;
assign z1_l = !blocks[location-1] && !blocks[location+8] && !blocks[location-9] && ((location)%8);
assign z1_r = !blocks[location+10] && !blocks[location+2] && !blocks[location-7] && ((location+2)%8);
assign z1_d = !blocks[location-16] && !blocks[location-7] && (location>16); 
assign z1_rot = !blocks[location-1] && !blocks[location-7];

//for Row clear condition
wire above_row, location_row, below_row, double_below_row;
assign above_row = blocks[(location/8 +1)*8] && blocks[(location/8+1)*8 + 1]
					&& blocks[(location/8+1)*8 + 2]&& blocks[(location/8+1)*8 + 3]
					&& blocks[(location/8+1)*8 + 4]&& blocks[(location/8+1)*8 + 5]
					&& blocks[(location/8+1)*8 + 6]&& blocks[(location/8+1)*8 + 7]; 
assign location_row = blocks[(location/8)*8] && blocks[(location/8)*8 + 1]
					&& blocks[(location/8)*8 + 2]&& blocks[(location/8)*8 + 3]
					&& blocks[(location/8)*8 + 4]&& blocks[(location/8)*8 + 5]
					&& blocks[(location/8)*8 + 6]&& blocks[(location/8)*8 + 7]; 
assign below_row = blocks[(location/8-1)*8] && blocks[(location/8-1)*8 + 1]
					&& blocks[(location/8-1)*8 + 2]&& blocks[(location/8-1)*8 + 3]
					&& blocks[(location/8-1)*8 + 4]&& blocks[(location/8-1)*8 + 5]
					&& blocks[(location/8-1)*8 + 6]&& blocks[(location/8-1)*8 + 7]; 
assign double_below_row = blocks[(location/8-2)*8] && blocks[(location/8-2)*8 + 1]
					&& blocks[(location/8-2)*8 + 2]&& blocks[(location/8-2)*8 + 3]
					&& blocks[(location/8-2)*8 + 4]&& blocks[(location/8-2)*8 + 5]
					&& blocks[(location/8-2)*8 + 6]&& blocks[(location/8-2)*8 + 7]; 


assign { q_Lose, q_Col, q_Rot, q_Gen, q_I} = state[4:0] ;
	

localparam
	INITIAL = 8'b0000_0001,
	GENERATE_PIECE = 8'b0000_0010,
	ROTATE_PIECE = 8'b0000_0100,
	COLLISION = 8'b0000_1000,
	LOSE = 8'b0001_0000,
	CLEAR_ROW = 8'b0010_0000,
	UNKNOWN = 8'bxxxx_xxxx;
	
//temp
localparam
	empty_row = 8'b0000_0000,
	full_row = 8'b1111_1111,
	loop_max =  25'd1, //25'b11111_11111_11111_11111_11111, //25'd1,
	bottom = 8'b1110_1101;

//pieces	
localparam
	SQUARE = 3'b000,
	BAR = 3'b001,
	S = 3'b010,
	Z = 3'b011,
	L = 3'b100,
	J = 3'b101,
	T = 3'b110;

	
initial begin
	random_count = $random;
end
	
always @ (posedge Clk )
	begin: RANDOM_NUMBER_GENERATOR
		if(random_count >= 0'b110)
			random_count <= 0;
		else
			random_count <= random_count+ 1'b1;
	end
	
	
	
always @ (posedge Clk, posedge Reset)
	begin
		if(Reset)
			begin 
			state <= INITIAL;
			loop <= 25'd0;
			for(i=0; i<160; i = i+1)
				begin
				blocks[i] <= 0;
				end
			score <= 0;
			location <= 0;
			end
		else
			begin
			(* full_case, parallel_case *)
			case(state)
				INITIAL : 
					begin
					if(Start)
						state <= GENERATE_PIECE;
					else
						state <= INITIAL;
					
					loop <= 25'd0;
					for(i=0; i<160; i = i+1)
					begin
					blocks[i] <= 0;
					end
					score <= 0;
					location <= 0;
					block_type <= random_count %2; 
					next_block <= random_count %2;
					orientation <= 2'b00;
					
					end
				GENERATE_PIECE :
					begin
					(* full_case, parallel_case *)
					case(next_block)
					SQUARE :
						begin
						if(blocks[154] || blocks[153] || blocks[146] || blocks[145]	)
							state <= LOSE;
						else
							state <= ROTATE_PIECE;
						end
					BAR :
						begin
						if(blocks[152] || blocks[153] || blocks[154] || blocks[155])
							state <= LOSE;
						else 
							state <= ROTATE_PIECE;
						end
					S:
						begin
						if(blocks[154] || blocks[155] || blocks[146] || blocks[145] )
							state <= LOSE;
						else
							state <= ROTATE_PIECE;
						end
					Z :
						begin
						if( blocks[154] || blocks[153] || blocks[146] || blocks[147] )
							state <= LOSE;
						else
							state <= ROTATE_PIECE;
						end
					L:
						begin
						if( blocks[154] || blocks[155] || blocks[153] || blocks[145])
							state <= LOSE;
						else
							state <= ROTATE_PIECE;
						end
					J :
						begin
						if( blocks[154] || blocks[153] || blocks[155] || blocks[147])
							state <= LOSE;
						else
							state <= ROTATE_PIECE;
						end
					T :
						begin
						if(blocks[154] || blocks[153] || blocks[155] || blocks[146])
							state <= LOSE;
						else
							state <= ROTATE_PIECE;
						end
					endcase
					
					
					//State Actions
					block_type <= next_block;
					next_block <= random_count %2; //change for all blocks
					orientation <= 2'b00;
					location <= 8'd154;
					loop <= 25'b0;
					(* full_case, parallel_case *)
					case( next_block)
					SQUARE:
						begin
						blocks [154] <= 1;
						blocks[153] <= 1;
						blocks[146]<= 1;
						blocks[145] <= 1;
						end
					BAR:
						begin 
						blocks[152] <= 1;
						blocks[153] <= 1;
						blocks[154] <= 1;
						blocks[155] <= 1;
						end
					S:			
						begin
						blocks[154] <= 
						1;
						blocks[155] <= 1;
						blocks[146] <= 1;
						blocks[145] <= 1;
						end
					Z:
						begin
						blocks[154] <= 1;
						blocks[153] <= 1;
						blocks[146] <= 1;
						blocks[147] <= 1;						
						end					
					L:		
						begin
						blocks[154] <= 1;
						blocks[155] <= 1;
						blocks[153] <= 1;
						blocks[145] <= 1;
						end					
					J:	
						begin
						blocks[154] <= 1;
						blocks[153] <= 1;
						blocks[155] <= 1;
						blocks[147] <= 1;
						end					
					T:	
						begin
						blocks[154] <= 1;
						blocks[153] <= 1;
						blocks[155] <= 1;
						blocks[146] <= 1;
						end
					endcase
					end
				ROTATE_PIECE :
					begin
					if( loop < loop_max)
						state <= ROTATE_PIECE;
					else if(loop == loop_max)
						state <= COLLISION;
						
					loop<= loop+ 1'b1;
					
					if(block_type == SQUARE)
						begin					
						if(Left && square_l )
							begin
							blocks[location] <= 0;
							blocks[location-8] <= 0;
							blocks[location-10] <= 1;
							blocks[location -2] <= 1;
							location <= location - 1'b1;
							
							end
						else if( Right && square_r)
							begin
							blocks[location-1] <= 0;
							blocks[location-9] <=0;							
							blocks[location +1] <= 1;
							blocks[location - 7] <= 1;
							location <= location + 1'b1;
						 
							end
						else if( Down && square_d)
							begin
							blocks[location] <= 0;
							blocks[location-1] <= 0;							
							blocks[location-16] <= 1;
							blocks[location-17] <= 1;
							location <= location - 4'd8;
							loop<= 25'd0;
							end
						end						
					else if(block_type == BAR)
						begin
						if(Left && !orientation[0] && bar0_l)
							begin
							blocks[location +1] <= 0;
							blocks[location -3] <= 1;
							location <= location - 1'b1;
							end							
						else if(Right && !orientation && bar0_r)
							begin
							blocks[location +2] <= 1;
							blocks[location -2] <= 0;
							location <= location+1'b1;
							end
						else if(Down && !orientation && bar0_d)
							begin
							blocks[location] <= 0;
							blocks[location+1] <= 0;
							blocks[location-1] <= 0;
							blocks[location-2] <= 0;
							blocks[location-7] <= 1;
							blocks[location-8] <= 1;
							blocks[location-9] <= 1;
							blocks[location-10] <= 1;
							location <= location -4'd8;
							loop<= 25'd0;
							end
						else if(Rotate && !orientation && bar0_rot)
							begin
							blocks[location+8] <= 1;
							blocks[location-8] <= 1;
							blocks[location-16] <= 1;
							blocks[location-2] <= 0;
							blocks[location-1] <= 0;
							blocks[location +1] <= 0;
							orientation <= 2'b01;
							end
						else if(Left && orientation[0] && bar1_l)
							begin
							blocks[location +8] <= 0; 
							blocks[location] <= 0; 
							blocks[location -8] <= 0; 
							blocks[location -16] <= 0;
							blocks[location +7] <= 1; 
							blocks[location-1] <= 1; 
							blocks[location -9] <= 1; 
							blocks[location -17] <= 1;
							location <= location - 1'b1;
							end
						else if(Right && orientation[0] && bar1_r)
							begin
							blocks[location +8] <= 0; 
							blocks[location] <= 0; 
							blocks[location -8] <= 0; 
							blocks[location -16] <= 0;
							blocks[location +9] <= 1; 
							blocks[location+1] <= 1; 
							blocks[location -7] <= 1; 
							blocks[location -15] <= 1;
							location <= location +1'b1;
							end
						else if(Down && orientation[0]  && bar1_d)
							begin
							blocks[location +8] <= 0;
							blocks[location -24] <= 1;
							location <= location -4'd8;
							loop<= 25'd0;
							end
						else if(Rotate && orientation[0] && bar1_rot)
							begin
							blocks[location+8] <= 0;
							blocks[location-8] <= 0;
							blocks[location-16] <= 0;
							blocks[location+1] <= 1;
							blocks[location-1] <= 1;
							blocks[location-2] <= 1;
							orientation <= 2'b00;
							end
						end
					else if( block_type == S)
						begin
						if(!orientation[0])
							begin
							end
						else if(orientation[0])
							begin
							end
						end
					else if( block_type == Z)
						begin
						if(!orientation[0])
							begin
							end
						else if(orientation[0])
							begin
							end
						end
					else if( block_type == L)
						begin
						if(!orientation[0])
							begin
							end
						else if(orientation[0])
							begin
							end
						else if(orientation == 2'b10)
							begin
							end
						else if(orientation == 2'b11)
							begin
							end
						end
					else if( block_type == J)
						begin
						if(!orientation[0])
							begin
							end
						else if(orientation[0])
							begin
							end
						else if(orientation == 2'b10)
							begin
							end
						else if(orientation == 2'b11)
							begin
							end
						end
					else if( block_type == T)
						begin
						if(!orientation[0])
							begin
							end
						else if(orientation[0])
							begin
							end
						else if(orientation == 2'b10)
							begin
							end
						else if(orientation == 2'b11)
							begin
							end
						end					
					end
				COLLISION :
					begin
					if( (block_type == SQUARE && !square_d && (location_row + below_row) ==2 )   
						|| (block_type == BAR && !(orientation[0] ? bar1_d : bar0_d) && 
							(orientation[0] ? (above_row + location_row + below_row + double_below_row) >1 : location_row)))
						state <= CLEAR_ROW;
					else if( (block_type == SQUARE && !square_d)
							|| block_type == BAR && !(orientation[0] ? bar1_d : bar0_d))
						state <= GENERATE_PIECE;
					else 
						state <= ROTATE_PIECE;
					
					// Start of RTL
					if(block_type == SQUARE)
						begin
						if(square_d)
							begin
							blocks[location] <= 0;
							blocks[location-1] <= 0;							
							blocks[location-16] <= 1;
							blocks[location-17] <= 1;
							location <= location - 4'd8;
							loop<= 25'd0;
							end
						end
					else if(block_type == BAR)
						begin
						if( !orientation[0])
							begin
							if(bar0_d)
								begin
								blocks[location] <= 0;
								blocks[location+1] <= 0;
								blocks[location-1] <= 0;
								blocks[location-2] <= 0;
								blocks[location-7] <= 1;
								blocks[location-8] <= 1;
								blocks[location-9] <= 1;
								blocks[location-10] <= 1;
								location <= location -4'd8;
								loop <= 25'd0;
								end
							end	
						else if(orientation[0])
							begin
							if(bar1_d)
								begin
								blocks[location+8] <= 0;
								blocks[location-24] <= 1;
								location <= location - 4'd8;
								loop <= 25'd0;
								end					
							end
						end	// end of RTL
					end // end of the Collision State
				CLEAR_ROW:
					begin
					if( (block_type == SQUARE && !square_d && (location_row + below_row) ==2 )   
						|| (block_type == BAR && !(orientation[0] ? bar1_d : bar0_d) && 
							(orientation[0] ? (above_row + location_row + below_row + double_below_row) >1 : location_row)))
						state <= CLEAR_ROW;
					else
						state <= GENERATE_PIECE;
					
					
					
					end
				LOSE: 
					begin
					if(Ack)
						state<= INITIAL;
					else
						state<= LOSE;							
					end
				default : state <= UNKNOWN;
				endcase
			end
	end
endmodule
