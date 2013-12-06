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
reg [2:0] block_type;
output reg [1:0] orientation;

output reg [2:0] next_block;
integer i;

// Number of Loops for Rotate and Move
reg [24:0] loop;
reg [2:0] random_count;

wire [19:0] full_rows;
reg full_row_present;

assign full_rows[0] = blocks[0] && blocks[1] && blocks[2] && blocks[3] && blocks[4] && blocks[5] && blocks[6] && blocks[7];
assign full_rows[1] = blocks[8] && blocks[9] && blocks[10] && blocks[11] && blocks[12] && blocks[13] && blocks[14] && blocks[15];
assign full_rows[2] = blocks[16] && blocks[17] && blocks[18] && blocks[19] && blocks[20] && blocks[21] && blocks[22] && blocks[23]; 
assign full_rows[3] = blocks[24] && blocks[25] && blocks[26] && blocks[27] && blocks[28]&& blocks[29]&& blocks[30]&& blocks[31];
assign full_rows[4] = blocks[32] && blocks[33] && blocks[34] && blocks[35]&& blocks[36]&& blocks[37]&& blocks[38] && blocks[39];
assign full_rows[5] = blocks[40] && blocks[41] && blocks[42] && blocks[43] && blocks[44] && blocks[45] && blocks[46] && blocks[47] ;
assign full_rows[6] = blocks[48] && blocks[49] && blocks[50] && blocks[51] && blocks[52] && blocks[53] && blocks[54] && blocks[55] ;
assign full_rows[7] = blocks[56] && blocks[57] && blocks[58] && blocks[59] && blocks[60] && blocks[61] && blocks[62] && blocks[63];
assign full_rows[8] = blocks[64] && blocks[65] && blocks[66] && blocks[67] && blocks[68] && blocks[69] && blocks[70] && blocks[71] ;
assign full_rows[9] = blocks[72] && blocks[73] && blocks[74] && blocks[75] && blocks[76] && blocks[77] && blocks[78] && blocks[79] ;
assign full_rows[10] = blocks[80] && blocks[81] && blocks[82] && blocks[83] && blocks[84] && blocks[84] && blocks[85] && blocks[86] ;
assign full_rows[11] = blocks[88] && blocks[89] && blocks[90] && blocks[91] && blocks[92] && blocks[93] && blocks[94] && blocks[95] ;
assign full_rows[12] = blocks[96] && blocks[97] && blocks[98] && blocks[99] && blocks[100] && blocks[101] && blocks[102] && blocks[103] ;
assign full_rows[13] = blocks[104] && blocks[105] && blocks[106] && blocks[107] && blocks[108] && blocks[109] && blocks[110] && blocks[111] ;
assign full_rows[14] = blocks[112] && blocks[113] && blocks[114] && blocks[115] && blocks[116] && blocks[117] && blocks[118] && blocks[119] ;
assign full_rows[15] = blocks[120] && blocks[121] && blocks[122] && blocks[123] && blocks[124] && blocks[125] && blocks[126] && blocks[127] ;
assign full_rows[16] = blocks[128] && blocks[129] && blocks[130] && blocks[131] && blocks[132] && blocks[133] && blocks[134] && blocks[135] ;
assign full_rows[17] = blocks[136] && blocks[137] && blocks[138] && blocks[139] && blocks[140] && blocks[141] && blocks[142] && blocks[143] ;
assign full_rows[18] = blocks[144] && blocks[145] && blocks[146] && blocks[147] && blocks[148] && blocks[149] && blocks[150] && blocks[151] ;
assign full_rows[19] = blocks[152] && blocks[153] && blocks[154] && blocks[155] && blocks[156] && blocks[157] && blocks[158] && blocks[159];


	
//Check if space is avaliable for a rotate or move down Wire
//Square
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
assign s0_d = !blocks[location-7] && !blocks[location-16] && !blocks[location-17] && (location>15); 
assign s0_rot = (location/8 != 19) && !blocks[location+8] && !blocks[location-7];

wire s1_l, s1_r, s1_d, s1_rot;
assign s1_l = !blocks[location-1] && !blocks[location+7] && !blocks[location-8] && ((location)%8);
assign s1_r = !blocks[location+9] && !blocks[location+2] && !blocks[location-6] && ((location+2)%8);
assign s1_d = !blocks[location-15] && !blocks[location-8] && (location>15); 
assign s1_rot = !blocks[location-8] && !blocks[location-9];

wire z0_l, z0_r, z0_d, z0_rot;
assign z0_l = !blocks[location-9] && !blocks[location-2] && ((location-1)%8);
assign z0_r = !blocks[location+1] && !blocks[location-6] && ((location+2)%8);
assign z0_d = !blocks[location-9] && !blocks[location-16] && !blocks[location-15] && (location>15); 
assign z0_rot = (location/8 != 19) && !blocks[location+1] && !blocks[location+9];

wire z1_l, z1_r, z1_d, z1_rot;
assign z1_l = !blocks[location-1] && !blocks[location+8] && !blocks[location-9] && ((location)%8);
assign z1_r = !blocks[location+10] && !blocks[location+2] && !blocks[location-7] && ((location+2)%8);
assign z1_d = !blocks[location-16] && !blocks[location-7] && (location>15); 
assign z1_rot = !blocks[location-1] && !blocks[location-7];

wire l0_l, l0_r, l0_d, l0_rot;
assign l0_l = !blocks[location-2] && !blocks[location - 10] && (location-1)%8;
assign l0_r = !blocks[location+2] && (location+2)%8;
assign l0_d = !blocks[location-8] && !blocks[location-7] && !blocks[location-18] && location> 15;
assign l0_rot = !blocks[location-8] && !blocks[location-7] && !blocks[location +8] && (location/8 != 19) ;

wire l1_l, l1_r, l1_d, l1_rot;
assign l1_l = !blocks[location-1] && !blocks[location +7] && !blocks[location -9] && location%8;
assign l1_r = !blocks[location+1] && !blocks[location +9] !blocks[location - 6] && (location +2) %8;
assign l1_d = !blocks[location-16] && !blocks[location-15] && location > 15;
assign l1_rot = !blocks[location-1] && !blocks[location+1] && !blocks[location +9] ;

wire l2_l, l2_r, l2_d, l2_rot;
assign l2_l = !blocks[location-2] && !blocks[location+8] && (location -1)%8;
assign l2_r = !blocks[location+2] && !blocks[location+10] && (location+2)%8;
assign l2_d = !blocks[location -8] && !blocks[location-7] && !blocks[location-9] && location> 7;
assign l2_rot = !blocks[location+8] && !blocks[location+7] && !blocks[location-8] && location >7;

wire l3_l, l3_r, l3_d, l3_rot;
assign l3_l = !blocks[location-1] && !blocks[location-9} && !blocks[location +6] && (location -1)%8;
assign l3_r = !blocks[location +1] && !blocks[location -9] && !blocks[location -7] && (location+1)%8 ;
assign l3_d = !blocks[location -1] && !blocks[location-16] && location >15;
assign l3_rot = !blocks[location+1] && !blocks[location-1] && !blocks[location-9] && (location+1)%8;

wire j0_l, j0_r, j0_d, j0_rot;
assign j0_l = !blocks[location-2] && !blocks[location-8] && (location-1)%8;
assign j0_r = !blocks[location+2] && !blocks[location-6] && (location +2)%8;
assign j0_d = !blocks[location-8] && !blocks[location -9] && !blocks[location-6] && (location >15);
assign j0_rot = !blocks[location+8] && !blocks[location-8] && !blocks[location+9];

// wire j1_l, j1_r, j1_d, j1_rot;
// assign j1_l = ;
// assign j1_r = ;
// assign j1_d = ;
// assign j1_rot = ;

// wire j2_l, j2_r, j2_d, j2_rot;
// assign j2_l = ;
// assign j2_r = ;
// assign j2_d = ;
// assign j2_rot = ;

// wire j3_l, j3_r, j3_d, j3_rot;
// assign j3_l = ;
// assign j3_r = ;
// assign j3_d = ;
// assign j3_rot = ;

// wire t0_l, t0_r, t0_d, t0_rot;
// assign t0_l = ;
// assign t0_r = ;
// assign t0_d = ;
// assign t0_rot = ;

// wire t1_l, t1_r, t1_d, t1_rot;
// assign t1_l = ;
// assign t1_r = ;
// assign t1_d = ;
// assign t1_rot = ;

// wire t2_l, t2_r, t2_d, t2_rot;
// assign t2_l = ;
// assign t2_r = ;
// assign t2_d = ;
// assign t2_rot = ;

// wire t3_l, t3_r, t3_d, t3_rot;
// assign t3_l = ;
// assign t3_r = ;
// assign t3_d = ;
// assign t3_rot = ;

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
			for(  i=0; i<160; i = i+1)
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
					for( i=0; i<160; i = i+1)
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
						blocks[154] <= 1;
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
					full_row_present <= full_row[0] || full_row[1] || full_row[2] || full_row[3] || full_row[4] || full_row[5] || full_row[6] || full_row[7] || full_row[8] || full_row[9] || full_row[10]|| full_row[11]|| full_row[12]|| full_row[13]|| full_row[14]|| full_row[15]|| full_row[16]|| full_row[17]|| full_row[18]|| full_row[19];
					
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
							if( Left && s0_l)
								begin
								location <= location -1'b1;
								blocks[location-1] <= 1;
								blocks[location-10] <= 1;
								blocks[location-8] <= 0;
								blocks[location+1] <= 0;
								end
							else if( Right && s0_r)
								begin
								location <= location +1'b1;
								blocks[location +2] <= 1;
								blocks[location - 7] <= 1;
								blocks[location] <= 0;
								blocks[location -9] <= 0;
								end
							else if(Rotate && s0_rot)
								begin
								orientation <= 2'b01;
								blocks[location +8] <= 1;
								blocks[location -7] <= 1;
								blocks[location -9] <= 0;
								blocks[location -8] <= 0;							
								end
							else if( Down && s0_d)
								begin
								location <= location - 4'd8;
								loop <= 25'd0;
								blocks[location-16] <= 1;
								blocks[location-7] <= 1;
								blocks[location-17] <= 1;
								blocks[location] <= 0;
								blocks[location+1] <= 0;
								blocks[location-9] <= 0;								
								end							
							end
						else if(orientation[0])
							begin
							if( Left && s1_l)
								begin
								location <= location -1'b1;
								blocks[location-1] <= 1;	
								blocks[location+7] <= 1;	
								blocks[location-8] <= 1;	
								blocks[location+1] <= 0;	
								blocks[location-7] <= 0;	
								blocks[location-8] <= 0;	
								end
							else if( Right && s1_r)
								begin
								location <= location +1'b1;
								end
							else if(Rotate && s1_rot)
								begin
								orientation <= 2'b00;
								end
							else if( Down && s1_d)
								begin
								location <= location - 4'd8;
								end
							end
						end
					else if( block_type == Z)
						begin
						if(!orientation[0])
							begin
							if( Left && z0_l)
								begin
								location <= location -1'b1;
								end
							else if( Right && z0_r)
								begin
								location <= location +1'b1;
								end
							else if(Rotate && z0_rot)
								begin
								orientation <= 2'b01;
								end
							else if( Down && z0_d)
								begin
								location <= location - 4'd8;
								end			
							end
						else if(orientation[0])
							begin
							if( Left && z1_l)
								begin
								location <= location -1'b1;
								end
							else if( Right && z1_r)
								begin
								location <= location +1'b1;
								end
							else if(Rotate && z1_rot)
								begin
								orientation <= 2'b00;
								end
							else if( Down && z1_d)
								begin
								location <= location - 4'd8;
								end		
							end
						end
					else if( block_type == L)
						begin
						if(!orientation[0])
							begin
							if( Left && l0_l)
								begin
								location <= location -1'b1;
								end
							else if( Right && l0_r)
								begin
								location <= location +1'b1;
								end
							else if(Rotate && l0_rot)
								begin
								orientation <= 2'b01;
								end
							else if( Down && l0_d)
								begin
								location <= location - 4'd8;
								end			
							end
						else if(orientation[0])
							begin
							if( Left && l1_l)
								begin
								location <= location -1'b1;
								end
							else if( Right && l1_r)
								begin
								location <= location +1'b1;
								end
							else if(Rotate && l1_rot)
								begin
								orientation <= 2'b10;
								end
							else if( Down && l1_d)
								begin
								location <= location - 4'd8;
								end		
							end
						else if(orientation == 2'b10)
							begin
							if( Left && l2_l)
								begin
								location <= location -1'b1;
								end
							else if( Right && l2_r)
								begin
								location <= location +1'b1;
								end
							else if(Rotate && l2_rot)
								begin
								orientation <= 2'b11;
								end
							else if( Down && l2_d)
								begin
								location <= location - 4'd8;
								end			
							end
						else if(orientation == 2'b11)
							begin
							if( Left && l3_l)
								begin
								location <= location -1'b1;
								end
							else if( Right && l3_r)
								begin
								location <= location +1'b1;
								end
							else if(Rotate && l3_rot)
								begin
								orientation <= 2'b00;
								end
							else if( Down && l3_d)
								begin
								location <= location - 4'd8;
								end		
							end
						end
					else if( block_type == J)
						begin
						if(!orientation[0])
							begin
							if( Left && j0_l)
								begin
								location <= location -1'b1;
								end
							else if( Right && j0_r)
								begin
								location <= location +1'b1;
								end
							else if(Rotate && j0_rot)
								begin
								orientation <= 2'b01;
								end
							else if( Down && j0_d)
								begin
								location <= location - 4'd8;
								end		
							end
						else if(orientation[0])
							begin
							if( Left && j1_l)
								begin
								location <= location -1'b1;
								end
							else if( Right && j1_r)
								begin
								location <= location +1'b1;
								end
							else if(Rotate && j1_rot)
								begin
								orientation <= 2'b10;
								end
							else if( Down && j1_d)
								begin
								location <= location - 4'd8;
								end		
							end
						else if(orientation == 2'b10)
							begin
							if( Left && j2_l)
								begin
								location <= location -1'b1;
								end
							else if( Right && j2_r)
								begin
								location <= location +1'b1;
								end
							else if(Rotate && j2_rot)
								begin
								orientation <= 2'b11;
								end
							else if( Down && j2_d)
								begin
								location <= location - 4'd8;
								end		
							end
						else if(orientation == 2'b11)
							begin
							if( Left && j3_l)
								begin
								location <= location -1'b1;
								end
							else if( Right && j3_r)
								begin
								location <= location +1'b1;
								end
							else if(Rotate && j3_rot)
								begin
								orientation <= 2'b00;
								end
							else if( Down && j3_d)
								begin
								location <= location - 4'd8;
								end		
							end
						end
					else if( block_type == T)
						begin
						if(!orientation[0])
							begin
							if( Left && t0_l)
								begin
								location <= location -1'b1;
								end
							else if( Right && t0_r)
								begin
								location <= location +1'b1;
								end
							else if(Rotate && t0_rot)
								begin
								orientation <= 2'b01;
								end
							else if( Down && t0_d)
								begin
								location <= location - 4'd8;
								end		
							end
						else if(orientation[0])
							begin
							if( Left && t1_l)
								begin
								location <= location -1'b1;
								end
							else if( Right && t1_r)
								begin
								location <= location +1'b1;
								end
							else if(Rotate && t1_rot)
								begin
								orientation <= 2'b10;
								end
							else if( Down && t1_d)
								begin
								location <= location - 4'd8;
								end		
							end
						else if(orientation == 2'b10)
							begin
							if( Left && t2_l)
								begin
								location <= location -1'b1;
								end
							else if( Right && t2_r)
								begin
								location <= location +1'b1;
								end
							else if(Rotate && t2_rot)
								begin
								orientation <= 2'b11;
								end
							else if( Down && t2_d)
								begin
								location <= location - 4'd8;
								end		
							end
						else if(orientation == 2'b11)
							begin
							if( Left && t3_l)
								begin
								location <= location -1'b1;
								end
							else if( Right && t3_r)
								begin
								location <= location +1'b1;
								end
							else if(Rotate && t3_rot)
								begin
								orientation <= 2'b00;
								end
							else if( Down && t3_d)
								begin
								location <= location - 4'd8;
								end		
							end
						end					
					end
				
				COLLISION :
					begin
					if( (block_type == SQUARE && !square_d && full_row_present)   
						|| (block_type == BAR && !(orientation[0] ? bar1_d : bar0_d) &&	full_row_present))
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
					if( full_row_present)
						state <= CLEAR_ROW;
					else
						state <= GENERATE_PIECE;
			
					full_row_present <= full_row[0] || full_row[1] || full_row[2] || full_row[3] || full_row[4] || full_row[5] || full_row[6] || full_row[7] || full_row[8] || full_row[9] || full_row[10]|| full_row[11]|| full_row[12]|| full_row[13]|| full_row[14]|| full_row[15]|| full_row[16]|| full_row[17]|| full_row[18]|| full_row[19];
					
					if( full_row_present)
						begin
						if(full_row[0])
							blocks[7:0] <= blocks[15:8];
						if(full_row[1])
							blocks[15:8] <= blocks[23:16];
						if(full_row[2])
							blocks[23:16] <= blocks[31:24];
						if(full_row[3])
							blocks[31:24] <= blocks[39:32];
						if(full_row[4])
							blocks[39:32] <= blocks[47:40];
						if(full_row[5])
							blocks[47:40] <= blocks[55:48];
						if(full_row[6])
							blocks[55:48] <= blocks[63:56];
						if(full_row[7])
							blocks[63:56] <= blocks[71:64];
						if(full_row[8])
							blocks[71:64] <= blocks[79:72];
						if(full_row[9])
							blocks[79:72] <= blocks[87:80];
						if(full_row[10])
							blocks[87:80] <= blocks[95:88];
						if(full_row[11])
							blocks[95:88] <= blocks[103:96];
						if(full_row[12])
							blocks[103:96] <= blocks[111:104];
						if(full_row[13])
							blocks[111:104] <= blocks[119:112];
						if(full_row[14])
							blocks[119:112] <= blocks[127:120];
						if(full_row[15])
							blocks[127:120] <=blocks[135:128];
						if(full_row[16])
							blocks[135:128] <= blocks[143:136];
						if(full_row[17])
							blocks[143:136]<= blocks[151:144];
						if(full_row[18])
							blocks[151:144] <= blocks[159:152];
						blocks[159:152] <= empty_row;
					end
						
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
