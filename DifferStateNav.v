`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    14:28:09 12/01/2018 
// Design Name: 
// Module Name:    DifferStateNav 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module DifferStateNav(clk,rst,x,y,z,vx,vy,vz,step,step_6,state_d0,state_d1,state_d2,
           state_d3,state_d4,state_d5,state_en,start
    );    
	 
input clk;
input rst;
input [63:0] x;
input [63:0] y;
input [63:0] z;
input [63:0] vx;
input [63:0] vy;
input [63:0] vz;
input [63:0] step; // VC: step=h
input [63:0] step_6; // VC: h/6
//input in_en;
input start;
output reg [63:0] state_d0;
output reg [63:0] state_d1;
output reg [63:0] state_d2;
output reg [63:0] state_d3;
output reg [63:0] state_d4;
output reg [63:0] state_d5;
output reg state_en;

 reg [63:0] state_d0_reg;
 reg [63:0] state_d1_reg;
 reg [63:0] state_d2_reg;
 reg [63:0] state_d3_reg;
 reg [63:0] state_d4_reg;
 reg [63:0] state_d5_reg;
 
parameter GME=64'b1100001011110110101010000110011001011011100100010001011000000000;//-3.986004415000000000e+14;
parameter s0=6'b000000,
          s1=6'b000001,
			 s2=6'b000011,
			 s3=6'b000010,
			 s4=6'b000110,
			 s5=6'b000111,
			 s6=6'b000101,
			 s7=6'b000100,
			 s8=6'b001100,
			 s9=6'b001101,
			s10=6'b001111,
			s11=6'b001110,
			s12=6'b001010,
			s13=6'b001011,
			s14=6'b001001,
			s15=6'b001000,
			s16=6'b011000,
			s17=6'b011001,
			s18=6'b011011,
			s19=6'b011010,
			s20=6'b011110,
			s21=6'b011111,
			s22=6'b011101,//only for NS2¡¢CS2
			s23=6'b011100,
			s24=6'b010100,
			s25=6'b010101,
			s26=6'b010111,
			s27=6'b010110,
			s28=6'b010010,
			s29=6'b010011,
			s30=6'b010001,
			s31=6'b010000,
			s32=6'b110000,
			s33=6'b110001,
			s34=6'b110011,
			s35=6'b110010,
			s36=6'b110110,
			s37=6'b110111,
			s38=6'b110101,
			s39=6'b110100,
			s40=6'b111100,
			s41=6'b111101,
			s42=6'b111111,
			s43=6'b111110,
			s44=6'b111010,
			s45=6'b111011,
			s46=6'b111001,
			s47=6'b111000,
			s48=6'b101000,
			s49=6'b101001,
			s50=6'b101011;

reg [5:0] CS,NS;
reg [3:0] cc1;
reg [63:0] x2,y2,z2,r2,r;
reg [63:0] DifferStateNav_d0;
reg [63:0] DifferStateNav_d1;
reg [63:0] DifferStateNav_d2;
reg [63:0] DifferStateNav_d3;
reg [63:0] DifferStateNav_d4;
reg [63:0] DifferStateNav_d5;
reg DifferStateNav_en;


//************common*******************//
reg [63:0] s_a_data_reg;
reg [63:0] s_b_data_reg;
reg [63:0] s_a_data_reg2;
reg [63:0] s_b_data_reg2;
//*************************************//

//************mult1********************//
wire s1_tvalid_wire;
wire [63:0] s1_a_data_wire;
wire [63:0] s1_b_data_wire;
wire s1_result_tvalid_wire;
wire [63:0] s1_result_tdata_wire;
reg s1_tvalid_reg;
reg s1_tvalid_reg2;
//*************************************//

//**************add1*******************//
wire s2_tvalid_wire;
wire [63:0] s2_a_tdata_wire;
wire [63:0] s2_b_tdata_wire;
wire s2_result_tvalid_wire;
wire [63:0] s2_result_tdata_wire;
reg s2_tvalid_reg;
reg s2_tvalid_reg2;
//*************************************//

//**********sqare1*********************//
wire s3_tvalid_wire;
wire [63:0] s3_a_tdata_wire;
wire s3_result_tvalid_wire;
wire [63:0] s3_result_tdata_wire;
reg s3_tvalid_reg;
//*************************************//

//*************div1********************//
wire s4_tvalid_wire;
wire [63:0] s4_a_tdata_wire;
wire [63:0] s4_b_tdata_wire;
wire s4_result_tvalid_wire;
wire [63:0] s4_result_tdata_wire;
reg s4_tvalid_reg;
//*************************************//

//*************Propagation4************//
reg [63:0] x_reg,y_reg,z_reg,vx_reg,vy_reg,vz_reg;
reg in_en;
reg ST; //DifferStateNav:ST=1; Propagation4:ST=0;
reg [63:0] differ_x0,differ_x1,differ_x2,differ_x3,differ_x4,differ_x5;
reg [63:0] temp0,temp1,temp2,temp3,temp4,temp5;
reg [63:0] k2_0,k2_1,k2_2,k2_3,k2_4,k2_5;
reg [63:0] k3_0,k3_1,k3_2,k3_3,k3_4,k3_5;
reg [63:0] k4_0,k4_1,k4_2,k4_3,k4_4,k4_5;
reg [3:0] cc2;
reg [5:0] CS2,NS2;

wire [63:0] step_2;
assign step_2[63]=step[63];
assign step_2[62:52]=step[62:52]-1; //->h/2
assign step_2[51:0]=step[51:0];
//*************************************//

always @ (posedge clk or negedge rst)
begin
     if(!rst)
	  begin
	       CS<=s0;
	  end
	  
	  else
	  begin
	       CS<=NS;
	  end
end

always @ (*)
begin
     case(CS)
	  s0:
	  begin
	       if(in_en==1) NS=s1;
			 else         NS=s0;
	  end
	  
	  s1: NS=s2;
	  
	  s2:
	  begin
	       if(cc1==6) NS=s3;
			 else       NS=s2;
	  end 
	  
	  s3: NS=s4;
	  
	  s4:
	  begin
	       if(cc1==6) NS=s5;
			 else       NS=s4;
	  end
	  
	  s5: NS=s6;
	  
	  s6:
	  begin
	       if(s2_result_tvalid_wire) NS=s7;
			 else                      NS=s6;
	  end
	  
	  s7: NS=s8;
	  s8: NS=s9;
	  s9:
	  begin
	       if(s2_result_tvalid_wire) NS=s10;
			 else                      NS=s9;
	  end
	  
	  s10: NS=s11;
	  s11: NS=s12;
	  
	  s12:
	  begin
	       if(s3_result_tvalid_wire) NS=s13;
			 else                      NS=s12;
	  end
	  
	  s13: NS=s14;
	  s14: NS=s15;
	  
	  s15:
	  begin
	       if(s1_result_tvalid_wire) NS=s16;
			 else                      NS=s15;
	  end
	  
	  s16: NS=s17;
	  
	  s17:
	  begin
	       if(cc1==3) NS=s18;
			 else       NS=s17;
	  end
	  
	  s18:
	  begin
	       if(s4_result_tvalid_wire) NS=s19;
			 else                      NS=s18;
	  end
	  
	  s19:
	  begin
	       if(cc1==3) NS=s20;
			 else       NS=s19;
	  end
	  
	  s20: NS=s21;
	  s21: NS=s0;
	  
	  default:
	  begin
	       NS=s0;
	  end
	  endcase
end

always @ (posedge clk or negedge rst)
begin
     if(!rst)
	  begin
	       cc1<=0;
			 DifferStateNav_en<=0;
			 s1_tvalid_reg<=0;
			 s2_tvalid_reg<=0;
			 s3_tvalid_reg<=0;
			 s4_tvalid_reg<=0;
	  end
	  
	  else
	  begin
	       case(NS)
			 s0:
			 begin
			      cc1<=0;
					s1_tvalid_reg<=0;
			 end
			 
			 s1:
			 begin
			      DifferStateNav_d0<=vx_reg; //output directly
               DifferStateNav_d1<=vy_reg; //output directly
               DifferStateNav_d2<=vz_reg; //output directly
               DifferStateNav_d3<=x_reg;
               DifferStateNav_d4<=y_reg;
               DifferStateNav_d5<=z_reg;
			 end
			 
			 s2:
			 begin
			      cc1<=cc1+1;
					s1_tvalid_reg<=1;
					if(cc1==0) //x^2
					begin
					     s_a_data_reg<=DifferStateNav_d3;
						  s_b_data_reg<=DifferStateNav_d3;
					end
					if(cc1==1) //y^2
					begin
					     s_a_data_reg<=DifferStateNav_d4;
						  s_b_data_reg<=DifferStateNav_d4;
					end
					if(cc1==2) //z^2
					begin
					     s_a_data_reg<=DifferStateNav_d5;
						  s_b_data_reg<=DifferStateNav_d5;
					end
					if(cc1==3) //G*x
					begin
					     s_a_data_reg<=GME;
						  s_b_data_reg<=DifferStateNav_d3;
					end
					if(cc1==4) //G*y
					begin
					     s_a_data_reg<=GME;
						  s_b_data_reg<=DifferStateNav_d4;
					end
					if(cc1==5) //G*z
					begin
					     s_a_data_reg<=GME;
						  s_b_data_reg<=DifferStateNav_d5;
					end
			 end
			 
			 s3:
			 begin
			      cc1<=0;
					s1_tvalid_reg<=0;
			 end
			 
			 s4:
			 begin
			      if(s1_result_tvalid_wire) 
					begin
					     cc1<=cc1+1;
						  x2<=y2;
						  y2<=z2;
						  z2<=DifferStateNav_d3; 
						  DifferStateNav_d3<=DifferStateNav_d4; //get G*x
						  DifferStateNav_d4<=DifferStateNav_d5; //get G*y
						  DifferStateNav_d5<=s1_result_tdata_wire; //get G*z
					end
					s2_tvalid_reg<=0;
			 end
			 
			 s5:
			 begin
			      s2_tvalid_reg<=1;
					s_a_data_reg<=x2;
					s_b_data_reg<=y2;
					cc1<=0;
			 end
			 
			 s6:
			 begin
			      s2_tvalid_reg<=0;
			 end
			 
			 s7:
			 begin
			      r2<=s2_result_tdata_wire;
			 end
			 
			 s8:
			 begin
			      s2_tvalid_reg<=1;
					s_a_data_reg<=r2;
					s_b_data_reg<=z2;
			 end 
			 
			 s9:
			 begin
			      s2_tvalid_reg<=0;
			 end
			 
			 s10:
			 begin
			      r2<=s2_result_tdata_wire; //get r2
					s3_tvalid_reg<=0;
			 end
			 
			 s11:
			 begin
			      s3_tvalid_reg<=1;
					s_a_data_reg<=r2;
			 end
			 
			 s12:
			 begin
			      s3_tvalid_reg<=0;
			 end
			 
			 s13:
			 begin
			      r<=s3_result_tdata_wire; //get r
					s1_tvalid_reg<=0;
			 end
			 
			 s14:
			 begin
			      s1_tvalid_reg<=1;
					s_a_data_reg<=r;
					s_b_data_reg<=r2;
			 end
			 
			 s15:
			 begin
			      s1_tvalid_reg<=0;
			 end
			 
			 s16:
			 begin
			      r2<=s1_result_tdata_wire; //get r^3
					s4_tvalid_reg<=0;
					cc1<=0;
			 end
			 
			 s17:
			 begin
			      cc1<=cc1+1;
			      s4_tvalid_reg<=1;
					
					if(cc1==0)
					begin
					     s_a_data_reg<=DifferStateNav_d3;
						  s_b_data_reg<=r2;
					end
					
					if(cc1==1)
					begin
					     s_a_data_reg<=DifferStateNav_d4;
						  s_b_data_reg<=r2;
					end
					
					if(cc1==2)
					begin
					     s_a_data_reg<=DifferStateNav_d5;
						  s_b_data_reg<=r2;
					end
			 end
			 
			 s18:
			 begin
			      cc1<=0;
					s4_tvalid_reg<=0;
			 end
			 
			 s19:
			 begin
			      cc1<=cc1+1;
			      DifferStateNav_d5<=s4_result_tdata_wire;
					DifferStateNav_d4<=DifferStateNav_d5;
					DifferStateNav_d3<=DifferStateNav_d4;
					DifferStateNav_en<=0;
			 end
			 
			 s20:
			 begin
			      cc1<=0;
					DifferStateNav_en<=1;
			 end
			 
			 s21:
			 begin
			      DifferStateNav_en<=0;
			 end
			 
			 default:
			 begin
			      cc1<=0;
			      DifferStateNav_en<=0;
			      s1_tvalid_reg<=0;
			      s2_tvalid_reg<=0;
			      s3_tvalid_reg<=0;
			      s4_tvalid_reg<=0;
			 end
			 endcase
	  end
end

//////////////////////////////////////////////////////////////////////
//**************************Propagation4****************************//
//////////////////////////////////////////////////////////////////////
always @ (posedge clk or negedge rst)
begin
     if(!rst)
	  begin
	       CS2<=s0;
	  end
	  
	  else
	  begin
	       CS2<=NS2;
	  end
end

always @(*)
begin
     case(CS2)
	  s0:
	  begin
	       if(start) NS2=s1;
			 else      NS2=s0;
	  end
	  
	  s1: NS2=s2;
	  s2: 
	  begin
	       if(DifferStateNav_en) NS2=s3;
			 else                  NS2=s2;
	  end
	  
	  s3: NS2=s4;
	  
	  s4:
	  begin
	       if(cc2==6) NS2=s5;
			 else       NS2=s4;
	  end
	  
	  s5:
	  begin
	       if(s1_result_tvalid_wire) NS2=s6;
			 else                      NS2=s5;
	  end
	  
	  s6:
	  begin
	       if(cc2==6) NS2=s7;
			 else       NS2=s6;
	  end
	  
	  s7: NS2=s8;
	  
	  s8:
	  begin
	       if(cc2==6) NS2=s9;
			 else       NS2=s8;
	  end
	  
	  s9:
	  begin
	       if(s2_result_tvalid_wire) NS2=s10;
			 else                      NS2=s9;
	  end
	  
	  s10:
	  begin
	       if(cc2==6) NS2=s11;
			 else       NS2=s10;
	  end
	  
	  s11: NS2=s12;
	  
	  s12:
	  begin
	       if(DifferStateNav_en) NS2=s13;
			 else                  NS2=s12;
	  end
	  
	  s13: NS2=s14;
	  
	  s14:
	  begin
	       if(cc2==6) NS2=s15;
			 else       NS2=s14;
	  end
	  
	  s15:
	  begin
	       if(s1_result_tvalid_wire) NS2=s16;
			 else                      NS2=s15;
	  end
	  
	  s16:
	  begin
	       if(cc2==6) NS2=s17;
			 else       NS2=s16;
	  end
	  
	  s17: NS2=s18;
	  
	  s18:
	  begin
	       if(cc2==6) NS2=s19;
			 else       NS2=s18;
	  end
	  
	  s19:
	  begin
	       if(s2_result_tvalid_wire) NS2=s20;
			 else                      NS2=s19;
	  end
	  
	  s20:
	  begin
	       if(cc2==6) NS2=s21;
			 else       NS2=s20;
	  end
	  
	  s21: NS2=s22;
	  
	  s22:
	  begin
	       if(DifferStateNav_en) NS2=s23;
			 else                  NS2=s22;
	  end
	  
	  s23: NS2=s24;
	  
	  s24:
	  begin
	       if(cc2==6) NS2=s25;
			 else       NS2=s24;
	  end
	  
	  s25:
	  begin
	       if(s1_result_tvalid_wire) NS2=s26;
			 else                      NS2=s25;
	  end
	  
	  s26:
	  begin
	       if(cc2==6) NS2=s27;
			 else       NS2=s26;
	  end
	  
	  s27: NS2=s28;
	  
	  s28:
	  begin
	       if(cc2==6) NS2=s29;
			 else       NS2=s28;
	  end
	  
	  s29:
	  begin
	       if(s2_result_tvalid_wire) NS2=s30;
			 else                      NS2=s29;
	  end
	  
	  s30:
	  begin
	       if(cc2==6) NS2=s31;
			 else       NS2=s30;
	  end
	  
	  s31: NS2=s32;
	  
	  s32:
	  begin
	       if(DifferStateNav_en) NS2=s33;
			 else                  NS2=s32;
	  end
	  
	  s33: NS2=s34;
	  
	  s34: NS2=s35;
	  
	  s35:
	  begin
	       if(cc2==12) NS2=s36;
			 else        NS2=s35;
	  end
	  
	  s36:
	  begin
	       if(s2_result_tvalid_wire) NS2=s37;
			 else                      NS2=s36;
	  end
	  
	  s37:
	  begin
	       if(cc2==12) NS2=s38;
			 else        NS2=s37;
	  end
	  
	  s38: NS2=s39;
	  
	  s39:
	  begin
	       if(cc2==6) NS2=s40;
			 else       NS2=s39;
	  end
	  
	  s40:
	  begin
	       if(s2_result_tvalid_wire) NS2=s41;
			 else                      NS2=s40;
	  end
	  
	  s41:
	  begin
	       if(cc2==6) NS2=s42;
			 else       NS2=s41;
	  end
	  
	  s42: NS2=s43;
	  
	  s43:
	  begin
	       if(cc2==6) NS2=s44;
			 else       NS2=s43;
	  end
	  
	  s44:
	  begin
	       if(s1_result_tvalid_wire) NS2=s45;
			 else                      NS2=s44;
	  end
	  
	  s45:
	  begin
	       if(cc2==6) NS2=s46;
			 else       NS2=s45;
	  end
	  
	  s46: NS2=s47;
	  
	  s47:
	  begin
	       if(cc2==6) NS2=s48;
			 else       NS2=s47;
	  end
	  
	  s48:
	  begin
	       if(s2_result_tvalid_wire) NS2=s49;
			 else                      NS2=s48;
	  end
	  
	  s49:
	  begin
	       if(cc2==6) NS2=s50;
			 else       NS2=s49;
	  end
	  
	  s50: NS2=s0;
	  
	  default: NS2=s0;
	  endcase
end

always @ (posedge clk or negedge rst)
begin
     if(!rst)
	  begin
	       in_en<=0;
			 ST<=0;
			 s1_tvalid_reg2<=0;
			 s2_tvalid_reg2<=0;
			 cc2<=0;
			 state_en<=0;
			 state_d0_reg<=0;
			 state_d1_reg<=0;
			 state_d2_reg<=0;
			 state_d3_reg<=0;
			 state_d4_reg<=0;
			 state_d5_reg<=0;
	  end
	  
	  else 
	  begin
	       case(NS2)
			 s0:
			 begin
			      in_en<=0;
					state_en<=0;
					ST<=0;
					s1_tvalid_reg2<=0;
					s2_tvalid_reg2<=0;
			      cc2<=0;
			 end
			 
			 s1: //initial state[6]
			 begin
			      x_reg<=x;
					y_reg<=y;
					z_reg<=z;
					vx_reg<=vx;
					vy_reg<=vy;
					vz_reg<=vz;
					in_en<=1;
					
					state_d0_reg<=x;
					state_d1_reg<=y;
					state_d2_reg<=z;
					state_d3_reg<=vx;
					state_d4_reg<=vy;
					state_d5_reg<=vz;
					ST<=1;
			 end
			 
			 s2:
			 begin
			      in_en<=0;
			 end
			 
			 s3:
			 begin
			      differ_x0<=DifferStateNav_d0;
					differ_x1<=DifferStateNav_d1;
					differ_x2<=DifferStateNav_d2;
					differ_x3<=DifferStateNav_d3;
					differ_x4<=DifferStateNav_d4;
					differ_x5<=DifferStateNav_d5;
					ST<=0;
					s1_tvalid_reg2<=0;
					cc2<=0;
			 end
			 
			 s4:
			 begin
			      s1_tvalid_reg2<=1;
					cc2<=cc2+1;
					s_b_data_reg2<=step_2;
					
					if(cc2==0) s_a_data_reg2<=differ_x0;
					if(cc2==1) s_a_data_reg2<=differ_x1;
					if(cc2==2) s_a_data_reg2<=differ_x2;
					if(cc2==3) s_a_data_reg2<=differ_x3;
					if(cc2==4) s_a_data_reg2<=differ_x4;
					if(cc2==5) s_a_data_reg2<=differ_x5;

			 end
			 
			 s5:
			 begin
			      s1_tvalid_reg2<=0;
					cc2<=0;
			 end
			 
			 s6:
			 begin
			      cc2<=cc2+1;
					
					temp5<=s1_result_tdata_wire;  //get differ_x5 * h/2
					temp4<=temp5;  //get differ_x4 * h/2
					temp3<=temp4;  //get differ_x3 * h/2
					temp2<=temp3;  //get differ_x2 * h/2
					temp1<=temp2;  //get differ_x1 * h/2
					temp0<=temp1;  //get differ_x0 * h/2
			 end
			 
			 s7:
			 begin
			      cc2<=0;
					s2_tvalid_reg2<=0;
			 end
			 
			 s8:
			 begin
			      cc2<=cc2+1;
					s2_tvalid_reg2<=1;
					
					if(cc2==0) 
					begin
					     s_a_data_reg2<=state_d0_reg;
						  s_b_data_reg2<=temp0;
					end
					if(cc2==1) 
					begin
					     s_a_data_reg2<=state_d1_reg;
						  s_b_data_reg2<=temp1;
					end
					if(cc2==2) 
					begin
					     s_a_data_reg2<=state_d2_reg;
						  s_b_data_reg2<=temp2;
					end
					if(cc2==3) 
					begin
					     s_a_data_reg2<=state_d3_reg;
						  s_b_data_reg2<=temp3;
					end
					if(cc2==4) 
					begin
					     s_a_data_reg2<=state_d4_reg;
						  s_b_data_reg2<=temp4;
					end
					if(cc2==5) 
					begin
					     s_a_data_reg2<=state_d5_reg;
						  s_b_data_reg2<=temp5;
					end
			 end
			 
			 s9:
			 begin
			      cc2<=0;
					s2_tvalid_reg2<=0;
			 end
			 
			 s10:
			 begin //VC: -> temp1[j]=state[j]+differ_x[j]*h/2
			      cc2<=cc2+1;
			      temp5<=s2_result_tdata_wire;
					temp4<=temp5;
					temp3<=temp4;
					temp2<=temp3;
					temp1<=temp2;
					temp0<=temp1;
			 end
			 
			 s11:
			 begin
			      cc2<=0;
					ST<=1;
					
					x_reg<=temp0;
					y_reg<=temp1;
					z_reg<=temp2;
					vx_reg<=temp3;
					vy_reg<=temp4;
					vz_reg<=temp5;
					in_en<=1;
			 end
			 
			 s12:
			 begin
			      in_en<=0;
			 end
			 
			 s13:
			 begin
			      k2_0<=DifferStateNav_d0;
					k2_1<=DifferStateNav_d1;
					k2_2<=DifferStateNav_d2;
					k2_3<=DifferStateNav_d3;
					k2_4<=DifferStateNav_d4;
					k2_5<=DifferStateNav_d5;
					
					ST<=0;
					s1_tvalid_reg2<=0;
			 end
			 
			 s14:
			 begin
			      cc2<=cc2+1;
			      s1_tvalid_reg2<=1;
					s_b_data_reg2<=step_2;
					
					if(cc2==0) s_a_data_reg2<=k2_0;
					if(cc2==1) s_a_data_reg2<=k2_1;
					if(cc2==2) s_a_data_reg2<=k2_2;
					if(cc2==3) s_a_data_reg2<=k2_3;
					if(cc2==4) s_a_data_reg2<=k2_4;
					if(cc2==5) s_a_data_reg2<=k2_5;
			 end
			 
			 s15:
			 begin
			      s1_tvalid_reg2<=0;
					cc2<=0;
			 end
			 
			 s16:
			 begin
			      cc2<=cc2+1;
					
					temp5<=s1_result_tdata_wire;  //VC:-> get k2[5] * h/2
					temp4<=temp5;  //VC:-> get k2[4] * h/2
					temp3<=temp4;  //VC:-> get k2[3] * h/2
					temp2<=temp3;  //VC:-> get k2[2] * h/2
					temp1<=temp2;  //VC:-> get k2[1] * h/2
					temp0<=temp1;  //VC:-> get k2[0] * h/2
			 end
			 
			 s17:
			 begin
			      cc2<=0;
					s2_tvalid_reg2<=0;
			 end
			 
			 s18:
			 begin
			      cc2<=cc2+1;
					s2_tvalid_reg2<=1;
					
					if(cc2==0) 
					begin
					     s_a_data_reg2<=state_d0_reg;
						  s_b_data_reg2<=temp0;
					end
					if(cc2==1) 
					begin
					     s_a_data_reg2<=state_d1_reg;
						  s_b_data_reg2<=temp1;
					end
					if(cc2==2) 
					begin
					     s_a_data_reg2<=state_d2_reg;
						  s_b_data_reg2<=temp2;
					end
					if(cc2==3) 
					begin
					     s_a_data_reg2<=state_d3_reg;
						  s_b_data_reg2<=temp3;
					end
					if(cc2==4) 
					begin
					     s_a_data_reg2<=state_d4_reg;
						  s_b_data_reg2<=temp4;
					end
					if(cc2==5) 
					begin
					     s_a_data_reg2<=state_d5_reg;
						  s_b_data_reg2<=temp5;
					end
			 end
			 
			 s19:
			 begin
			      cc2<=0;
					s2_tvalid_reg2<=0;
			 end
			 
			 s20:
			 begin //VC: -> temp1[j]=state[j]+k2[j]*h/2;
			      cc2<=cc2+1;
			      temp5<=s2_result_tdata_wire;
					temp4<=temp5;
					temp3<=temp4;
					temp2<=temp3;
					temp1<=temp2;
					temp0<=temp1;
			 end
			 
			 s21:
			 begin
			      cc2<=0;
					ST<=1;
					
					x_reg<=temp0;
					y_reg<=temp1;
					z_reg<=temp2;
					vx_reg<=temp3;
					vy_reg<=temp4;
					vz_reg<=temp5;
					in_en<=1;
			 end
			 
			 s22:
			 begin
			      in_en<=0;
			 end
			 
			 s23:
			 begin
			      k3_0<=DifferStateNav_d0;
					k3_1<=DifferStateNav_d1;
					k3_2<=DifferStateNav_d2;
					k3_3<=DifferStateNav_d3;
					k3_4<=DifferStateNav_d4;
					k3_5<=DifferStateNav_d5;
					
					ST<=0;
					s1_tvalid_reg2<=0;
			 end
			 
			 s24:
			 begin
			      cc2<=cc2+1;
			      s1_tvalid_reg2<=1;
					s_b_data_reg2<=step;
					
					if(cc2==0) s_a_data_reg2<=k3_0;
					if(cc2==1) s_a_data_reg2<=k3_1;
					if(cc2==2) s_a_data_reg2<=k3_2;
					if(cc2==3) s_a_data_reg2<=k3_3;
					if(cc2==4) s_a_data_reg2<=k3_4;
					if(cc2==5) s_a_data_reg2<=k3_5;
			 end
			 
			 s25:
			 begin
			      s1_tvalid_reg2<=0;
					cc2<=0;
			 end
			 
			 s26:
			 begin
			      cc2<=cc2+1;
					
					temp5<=s1_result_tdata_wire;  //VC:-> get k3[5] * h
					temp4<=temp5;  //VC:-> get k3[4] * h
					temp3<=temp4;  //VC:-> get k3[3] * h
					temp2<=temp3;  //VC:-> get k3[2] * h
					temp1<=temp2;  //VC:-> get k3[1] * h
					temp0<=temp1;  //VC:-> get k3[0] * h
			 end
			 
			 s27:
			 begin
			      cc2<=0;
					s2_tvalid_reg2<=0;
			 end
			 
			 s28:
			 begin
			      cc2<=cc2+1;
					s2_tvalid_reg2<=1;
					
					if(cc2==0) 
					begin
					     s_a_data_reg2<=state_d0_reg;
						  s_b_data_reg2<=temp0;
					end
					if(cc2==1) 
					begin
					     s_a_data_reg2<=state_d1_reg;
						  s_b_data_reg2<=temp1;
					end
					if(cc2==2) 
					begin
					     s_a_data_reg2<=state_d2_reg;
						  s_b_data_reg2<=temp2;
					end
					if(cc2==3) 
					begin
					     s_a_data_reg2<=state_d3_reg;
						  s_b_data_reg2<=temp3;
					end
					if(cc2==4) 
					begin
					     s_a_data_reg2<=state_d4_reg;
						  s_b_data_reg2<=temp4;
					end
					if(cc2==5) 
					begin
					     s_a_data_reg2<=state_d5_reg;
						  s_b_data_reg2<=temp5;
					end
			 end
			 
			 s29:
			 begin
			      cc2<=0;
					s2_tvalid_reg2<=0;
			 end
			 
			 s30:
			 begin //VC: -> temp1[j]=state[j]+k3[j]*h;
			      cc2<=cc2+1;
			      temp5<=s2_result_tdata_wire;
					temp4<=temp5;
					temp3<=temp4;
					temp2<=temp3;
					temp1<=temp2;
					temp0<=temp1;
			 end
			 
			 s31:
			 begin
			      cc2<=0;
					ST<=1;
					
					x_reg<=temp0;
					y_reg<=temp1;
					z_reg<=temp2;
					vx_reg<=temp3;
					vy_reg<=temp4;
					vz_reg<=temp5;
					in_en<=1;
			 end
			 
			 s32:
			 begin
			      in_en<=0;
			 end
			 
			 s33:
			 begin
			      k4_0<=DifferStateNav_d0;
					k4_1<=DifferStateNav_d1;
					k4_2<=DifferStateNav_d2;
					k4_3<=DifferStateNav_d3;
					k4_4<=DifferStateNav_d4;
					k4_5<=DifferStateNav_d5;
					
					ST<=0;
			 end
			 
			 s34:
			 begin
			      k2_0[62:52]<=k2_0[62:52]+1; //VC: 2*k2[0]
					k2_1[62:52]<=k2_1[62:52]+1; //VC: 2*k2[1]
					k2_2[62:52]<=k2_2[62:52]+1; //VC: 2*k2[2]
					k2_3[62:52]<=k2_3[62:52]+1; //VC: 2*k2[3]
					k2_4[62:52]<=k2_4[62:52]+1; //VC: 2*k2[4]
					k2_5[62:52]<=k2_5[62:52]+1; //VC: 2*k2[5]
					
					k3_0[62:52]<=k3_0[62:52]+1; //VC: 2*k3[0]
					k3_1[62:52]<=k3_1[62:52]+1; //VC: 2*k3[1]
					k3_2[62:52]<=k3_2[62:52]+1; //VC: 2*k3[2]
					k3_3[62:52]<=k3_3[62:52]+1; //VC: 2*k3[3]
					k3_4[62:52]<=k3_4[62:52]+1; //VC: 2*k3[4]
					k3_5[62:52]<=k3_5[62:52]+1; //VC: 2*k3[5]
					
					s2_tvalid_reg2<=0;
			 end
			 
			 s35:
			 begin
			      cc2<=cc2+1;
					s2_tvalid_reg2<=1;
					
					if(cc2==0) 
					begin
					     s_a_data_reg2<=differ_x0;
						  s_b_data_reg2<=k2_0;
					end
					if(cc2==1) 
					begin
					     s_a_data_reg2<=differ_x1;
						  s_b_data_reg2<=k2_1;
					end
					if(cc2==2) 
					begin
					     s_a_data_reg2<=differ_x2;
						  s_b_data_reg2<=k2_2;
					end
					if(cc2==3) 
					begin
					     s_a_data_reg2<=differ_x3;
						  s_b_data_reg2<=k2_3;
					end
					if(cc2==4) 
					begin
					     s_a_data_reg2<=differ_x4;
						  s_b_data_reg2<=k2_4;
					end
					if(cc2==5) 
					begin
					     s_a_data_reg2<=differ_x5;
						  s_b_data_reg2<=k2_5;
					end
					
					if(cc2==6) 
					begin
					     s_a_data_reg2<=k3_0;
						  s_b_data_reg2<=k4_0;
					end
					if(cc2==7) 
					begin
					     s_a_data_reg2<=k3_1;
						  s_b_data_reg2<=k4_1;
					end
					if(cc2==8) 
					begin
					     s_a_data_reg2<=k3_2;
						  s_b_data_reg2<=k4_2;
					end
					if(cc2==9) 
					begin
					     s_a_data_reg2<=k3_3;
						  s_b_data_reg2<=k4_3;
					end
					if(cc2==10) 
					begin
					     s_a_data_reg2<=k3_4;
						  s_b_data_reg2<=k4_4;
					end
					if(cc2==11) 
					begin
					     s_a_data_reg2<=k3_5;
						  s_b_data_reg2<=k4_5;
					end
			 end 
			 
			 s36:
			 begin
			      s2_tvalid_reg2<=0;
					cc2<=0;
			 end
			 
			 s37:
			 begin
			      cc2<=cc2+1;
					
			      k4_5<=s2_result_tdata_wire;
					k4_4<=k4_5;
					k4_3<=k4_4;
					k4_2<=k4_3;
					k4_1<=k4_2;
					k4_0<=k4_1; //VC: differ_x[j]+2*k2[j]
					
					k2_5<=k4_0;
					k2_4<=k2_5;
					k2_3<=k2_4;
					k2_2<=k2_3;
					k2_1<=k2_2;
					k2_0<=k2_1; //VC: 2*k3[j]+k4[j]
			 end
			 
			 s38:
			 begin
			      cc2<=0;
			 end
			 
			 s39:
			 begin
			      cc2<=cc2+1;
					s2_tvalid_reg2<=1;
					
					if(cc2==0)
					begin
					     s_a_data_reg2<=k2_0;
						  s_b_data_reg2<=k4_0;
					end
					if(cc2==1)
					begin
					     s_a_data_reg2<=k2_1;
						  s_b_data_reg2<=k4_1;
					end
					if(cc2==2)
					begin
					     s_a_data_reg2<=k2_2;
						  s_b_data_reg2<=k4_2;
					end
					if(cc2==3)
					begin
					     s_a_data_reg2<=k2_3;
						  s_b_data_reg2<=k4_3;
					end
					if(cc2==4)
					begin
					     s_a_data_reg2<=k2_4;
						  s_b_data_reg2<=k4_4;
					end
					if(cc2==5)
					begin
					     s_a_data_reg2<=k2_5;
						  s_b_data_reg2<=k4_5;
					end
			 end
			 
			 s40:
			 begin
			      cc2<=0;
					s2_tvalid_reg2<=0;
			 end
			 
			 s41:
			 begin //VC: differ_x[j]+2*k2[j]+2*k3[j]+k4[j]
			      cc2<=cc2+1;
			      temp5<=s2_result_tdata_wire;
					temp4<=temp5;
					temp3<=temp4;
					temp2<=temp3;
					temp1<=temp2;
					temp0<=temp1;
			 end
			 
			 s42:
			 begin
			      cc2<=0;
					s1_tvalid_reg2<=0;
			 end
			 
			 s43:
			 begin
			      cc2<=cc2+1;
					s1_tvalid_reg2<=1;
					
					s_a_data_reg2<=step_6;
					
					if(cc2==0) s_b_data_reg2<=temp0;
					if(cc2==1) s_b_data_reg2<=temp1;
					if(cc2==2) s_b_data_reg2<=temp2;
					if(cc2==3) s_b_data_reg2<=temp3;
					if(cc2==4) s_b_data_reg2<=temp4;
					if(cc2==5) s_b_data_reg2<=temp5;
			 end
			 
			 s44:
			 begin
			      cc2<=0;
					s1_tvalid_reg2<=0;
			 end
			 
			 s45:
			 begin //VC: h*(differ_x[j]+2*k2[j]+2*k3[j]+k4[j])/6
			      cc2<=cc2+1;
			      temp5<=s1_result_tdata_wire;
					temp4<=temp5;
					temp3<=temp4;
					temp2<=temp3;
					temp1<=temp2;
					temp0<=temp1;
			 end
			 
			 s46:
			 begin
			      cc2<=0;
					s2_tvalid_reg2<=0;
			 end
			 
			 s47:
			 begin
			      cc2<=cc2+1;
					s2_tvalid_reg2<=1;
					
					if(cc2==0)
					begin
					     s_a_data_reg2<=temp0;
						  s_b_data_reg2<=state_d0_reg;
					end
					if(cc2==1)
					begin
					     s_a_data_reg2<=temp1;
						  s_b_data_reg2<=state_d1_reg;
					end
					if(cc2==2)
					begin
					     s_a_data_reg2<=temp2;
						  s_b_data_reg2<=state_d2_reg;
					end
					if(cc2==3)
					begin
					     s_a_data_reg2<=temp3;
						  s_b_data_reg2<=state_d3_reg;
					end
					if(cc2==4)
					begin
					     s_a_data_reg2<=temp4;
						  s_b_data_reg2<=state_d4_reg;
					end
					if(cc2==5)
					begin
					     s_a_data_reg2<=temp5;
						  s_b_data_reg2<=state_d5_reg;
					end
			 end
			 
			 s48:
			 begin
			      cc2<=0;
					s2_tvalid_reg2<=0;
			 end
			 
			 s49:
			 begin //VC: state[j]=state[j]+h*(differ_x[j]+2*k2[j]+2*k3[j]+k4[j])/6;	
			      cc2<=cc2+1;
					
					state_d5<=s2_result_tdata_wire;
					state_d4<=state_d5;
					state_d3<=state_d4;
					state_d2<=state_d3;
					state_d1<=state_d2;
					state_d0<=state_d1;
					
					state_en<=0;
			 end
			 
			 s50:
			 begin
			      cc2<=0;
					state_en<=1;
			 end
			 
			 default:
			 begin
			      in_en<=0;
					state_en<=0;
					ST<=0;
					s1_tvalid_reg2<=0;
					s2_tvalid_reg2<=0;
			      cc2<=0;
			 end
			 
			 endcase
	  end
end 
//////////////////////////////////////////////////////////////////////
//**************************The end*********************************//
//////////////////////////////////////////////////////////////////////


assign s1_tvalid_wire=ST? s1_tvalid_reg:s1_tvalid_reg2;
assign s1_a_data_wire=ST? s_a_data_reg:s_a_data_reg2;
assign s1_b_data_wire=ST? s_b_data_reg:s_b_data_reg2;
Diff_mult1 Diff_mult1 (
  .aclk(clk), // input aclk
  .s_axis_a_tvalid(s1_tvalid_wire), // input s_axis_a_tvalid
  .s_axis_a_tdata(s1_a_data_wire), // input [63 : 0] s_axis_a_tdata
  .s_axis_b_tvalid(s1_tvalid_wire), // input s_axis_b_tvalid
  .s_axis_b_tdata(s1_b_data_wire), // input [63 : 0] s_axis_b_tdata
  .m_axis_result_tvalid(s1_result_tvalid_wire), // output m_axis_result_tvalid
  .m_axis_result_tdata(s1_result_tdata_wire) // output [63 : 0] m_axis_result_tdata
);

assign s2_tvalid_wire=ST? s2_tvalid_reg:s2_tvalid_reg2;
assign s2_a_tdata_wire=ST? s_a_data_reg:s_a_data_reg2;
assign s2_b_tdata_wire=ST? s_b_data_reg:s_b_data_reg2;
Diff_add1 Diff_add1 (
  .aclk(clk), // input aclk
  .s_axis_a_tvalid(s2_tvalid_wire), // input s_axis_a_tvalid
  .s_axis_a_tdata(s2_a_tdata_wire), // input [63 : 0] s_axis_a_tdata
  .s_axis_b_tvalid(s2_tvalid_wire), // input s_axis_b_tvalid
  .s_axis_b_tdata(s2_b_tdata_wire), // input [63 : 0] s_axis_b_tdata
  .m_axis_result_tvalid(s2_result_tvalid_wire), // output m_axis_result_tvalid
  .m_axis_result_tdata(s2_result_tdata_wire) // output [63 : 0] m_axis_result_tdata
);

assign s3_tvalid_wire=s3_tvalid_reg;
assign s3_a_tdata_wire=s_a_data_reg;
Diff_square1 Diff_square1 (
  .aclk(clk), // input aclk
  .s_axis_a_tvalid(s3_tvalid_wire), // input s_axis_a_tvalid
  .s_axis_a_tdata(s3_a_tdata_wire), // input [63 : 0] s_axis_a_tdata
  .m_axis_result_tvalid(s3_result_tvalid_wire), // output m_axis_result_tvalid
  .m_axis_result_tdata(s3_result_tdata_wire) // output [63 : 0] m_axis_result_tdata
);

assign s4_tvalid_wire=s4_tvalid_reg;
assign s4_a_tdata_wire=s_a_data_reg;
assign s4_b_tdata_wire=s_b_data_reg;
Diff_div1 Diff_div1 (
  .aclk(clk), // input aclk
  .s_axis_a_tvalid(s4_tvalid_wire), // input s_axis_a_tvalid
  .s_axis_a_tdata(s4_a_tdata_wire), // input [63 : 0] s_axis_a_tdata
  .s_axis_b_tvalid(s4_tvalid_wire), // input s_axis_b_tvalid
  .s_axis_b_tdata(s4_b_tdata_wire), // input [63 : 0] s_axis_b_tdata
  .m_axis_result_tvalid(s4_result_tvalid_wire), // output m_axis_result_tvalid
  .m_axis_result_tdata(s4_result_tdata_wire) // output [63 : 0] m_axis_result_tdata
);

endmodule
