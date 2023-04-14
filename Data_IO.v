`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    14:25:32 12/01/2018 
// Design Name: 
// Module Name:    Data_IO 
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
module Data_IO(clk,rst,TX,RX,request_data,state_en,state_t,state_x,state_y,state_z,state_vx,state_vy,state_vz,
               start_time,EXP_duration,sat_x,sat_y,sat_z,sat_vx,sat_vy,sat_vz,
					p1_n1,p1_n2,p1_n3,p1_F0G,p1_F1G,p1_F2G,p1_Pepoch,p1_phase0,
					p2_n1,p2_n2,p2_n3,p2_F0G,p2_F1G,p2_F2G,p2_Pepoch,p2_phase0,
					p3_n1,p3_n2,p3_n3,p3_F0G,p3_F1G,p3_F2G,p3_Pepoch,p3_phase0,
					p4_n1,p4_n2,p4_n3,p4_F0G,p4_F1G,p4_F2G,p4_Pepoch,p4_phase0,
					profile1_data,profile1_data_en,profile2_data,profile2_data_en,profile3_data,profile3_data_en,profile4_data,profile4_data_en,
					EarthData,EarthData_en,double_speed,duo_pulsar,duo_pulsar_table,duo_pulsar_table_en,
					running,static_experiment,rate,time_out,ERROR1,EXP_duration_done,duo_pulsar_table_len			
    );
 
input clk;
input rst;
input RX;
input request_data; //keep least 2 clk
input [63:0] state_t,state_x,state_y,state_z,state_vx,state_vy,state_vz; //send the state of aircraft to PC
input state_en;
input time_out; //The experiment has been completed: 1 yes, 0 no. (keep least 2 clk)
input ERROR1; //EarthEphCal error
input EXP_duration_done;

output TX;
output [63:0] start_time;
output [63:0] EXP_duration;
output [63:0] sat_x,sat_y,sat_z,sat_vx,sat_vy,sat_vz;
output [63:0] p1_n1,p1_n2,p1_n3,p1_F0G,p1_F1G,p1_F2G,p1_Pepoch,p1_phase0;
output [63:0] p2_n1,p2_n2,p2_n3,p2_F0G,p2_F1G,p2_F2G,p2_Pepoch,p2_phase0;
output [63:0] p3_n1,p3_n2,p3_n3,p3_F0G,p3_F1G,p3_F2G,p3_Pepoch,p3_phase0;
output [63:0] p4_n1,p4_n2,p4_n3,p4_F0G,p4_F1G,p4_F2G,p4_Pepoch,p4_phase0;
output [15:0] profile1_data;
output profile1_data_en;
output [15:0] profile2_data;
output profile2_data_en;
output [15:0] profile3_data;
output profile3_data_en;
output [15:0] profile4_data;
output profile4_data_en;
output [63:0] EarthData; //the EarthEph data
output EarthData_en;
output double_speed;
output duo_pulsar;
output [31:0] duo_pulsar_table;
output duo_pulsar_table_en;
output running;
output static_experiment;
output [2:0] rate;
output reg [31:0] duo_pulsar_table_len;

wire [7:0] OData_wire;
reg [7:0] OData_reg;
reg [7:0] OData2_reg;
wire OData_en_wire;
reg OData_en_reg;
reg OData2_en_reg;
wire done_wire;
wire [7:0] IData_wire;
wire IData_en_wire;

reg [5:0] CS,NS;
reg [5:0] CS2,NS2;
reg [7:0] code0,code1,code2;
reg [7:0] cc,cc2;
reg [15:0] cc3;
reg [7:0] cc4,cc5;
reg double_speed; //0 is off, 1 is on
reg duo_pulsar; //0 is off, 1 is on
reg running; //0 is off, 1 is on
reg static_experiment; //0 is off, 1 is on
reg [2:0] rate; //chose the transfer rate data of aircraft state
reg [63:0] start_time; //unit: Julian day
reg [63:0] EXP_duration; //unit: s
reg [7:0] data0,data1,data2,data3,data4,data5,data6,data7;
reg [63:0] sat_x,sat_y,sat_z,sat_vx,sat_vy,sat_vz; //the intial state of spacecraft
reg [15:0] profile_len;
reg [15:0] profile1_data;
reg profile1_data_en;
reg [15:0] profile2_data;
reg profile2_data_en;
reg [15:0] profile3_data;
reg profile3_data_en;
reg [15:0] profile4_data;
reg profile4_data_en;
reg [63:0] p1_n1,p1_n2,p1_n3,p1_F0G,p1_F1G,p1_F2G,p1_Pepoch,p1_phase0;
reg [63:0] p2_n1,p2_n2,p2_n3,p2_F0G,p2_F1G,p2_F2G,p2_Pepoch,p2_phase0;
reg [63:0] p3_n1,p3_n2,p3_n3,p3_F0G,p3_F1G,p3_F2G,p3_Pepoch,p3_phase0;
reg [63:0] p4_n1,p4_n2,p4_n3,p4_F0G,p4_F1G,p4_F2G,p4_Pepoch,p4_phase0;
reg [63:0] EarthData; //the EarthEph data
reg EarthData_en;
reg [31:0] duo_pulsar_table;
reg duo_pulsar_table_en;
//reg [31:0] duo_pulsar_table_len;
reg code_busy; //0 is not busy, 1 is busy
reg data_busy; //0 is not busy, 1 is busy
reg [63:0] sat_station[6:0];

parameter s0=7'd0, s1=7'd1, s2=7'd2, s3=7'd3, s4=7'd4, s5=7'd5, s6=7'd6, s7=7'd7, s8=7'd8, s9=7'd9, s10=7'd10, 
          s11=7'd11, s12=7'd12, s13=7'd13, s14=7'd14, s15=7'd15, s16=7'd16, s17=7'd17, s18=7'd18, s19=7'd19, s20=7'd20,
			 s21=7'd21, s22=7'd22, s23=7'd23, s24=7'd24, s25=7'd25, s26=7'd26, s27=7'd27, s28=7'd28, s29=7'd29, s30=7'd30, 
			 s31=7'd31, s32=7'd32, s33=7'd33, s34=7'd34, s35=7'd35, s36=7'd36, s37=7'd37, s38=7'd38, s39=7'd39, s40=7'd40, 
			 s41=7'd41, s42=7'd42, s43=7'd43, s44=7'd44, s45=7'd45, s46=7'd46, s47=7'd47, s48=7'd48, s49=7'd49, s50=7'd50,
			 s3_1=7'd51, s41_1=7'd52, s1_a=7'd53;
//*******************************************************//
always @ (posedge clk or negedge rst)
begin
     if(! rst) CS<=s0;
	  else      CS<=NS;
end

always @(*)
begin
     case(CS)
	  s0:
	  begin
	       if(IData_en_wire) NS=s1;
			 else              
			 begin
			      if(request_data || time_out || ERROR1) NS=s41;
					else             NS=s0;
			 end
	  end
	  
	  s1: NS=s2;
	  
	  s2:
	  begin
	       if(code2==8'h00 && code1==8'hFF) NS=s3;
			 else
			 begin
			      if(IData_en_wire) NS=s1;
					else              NS=s2;
			 end
	  end
	  
	  s3: 
	  begin
	       if(data_busy==0) NS=s3_1;
			 else             NS=s3;
	  end
	  
	  s3_1: NS=s4;
	  s4: NS=s5;
	  
	  s5:
	  begin
	       if(done_wire)
			 begin
			      if(cc==3) NS=s6;
					else      NS=s3_1;
			 end
			 else NS=s5;
	  end
	  
	  s6:
	  begin
	       case(code0)
			 8'h0B: NS=s7;
			 8'h0D: NS=s7;
			 8'h04: NS=s7;
			 8'h05: NS=s7;
			 8'h12: NS=s7;
			 8'h13: NS=s7;
			 8'h14: NS=s7;
			 8'h15: NS=s7;
			 8'h16: NS=s7;
			 8'h17: NS=s7;
			 8'h18: NS=s7;
			 8'h19: NS=s7;
			 8'h20: NS=s7;
			 8'h01: NS=s8;
			 8'h02: NS=s8;
			 8'h03: NS=s11;
			 8'h07: NS=s14;
			 8'h08: NS=s14;
			 8'h09: NS=s14;
			 8'h0A: NS=s14;
			 8'h0E: NS=s25;
			 8'h0F: NS=s25;
			 8'h10: NS=s25;
			 8'h11: NS=s25;
			 8'h06: NS=s28;
			 8'h0C: NS=s33;
			 
			 default: NS=s0;
			 endcase
	  end
	  
	  s7: NS=s0;
	  
	  s8:
	  begin
	       if(IData_en_wire) NS=s9;
			 else              NS=s8;
	  end
	  
	  s9:
	  begin
	       if(cc==8) NS=s10;
			 else      NS=s8;
	  end
	  
	  s10: NS=s0;
	  
	  s11:
	  begin
	       if(IData_en_wire) NS=s12;
			 else              NS=s11;
	  end
	  
	  s12:
	  begin
	       if(cc==8) NS=s13;
			 else      NS=s11;
	  end
	  
	  s13:
	  begin
	       if(cc2==6) NS=s0;
			 else       NS=s11;
	  end
	  
	  s14:
	  begin
	       if(IData_en_wire) NS=s15;
			 else              NS=s14;
	  end
	  
	  s15:
	  begin
	       if(cc==2) NS=s16;
			 else      NS=s14;
	  end
	  
	  s16: NS=s17;
	  
	  s17:
	  begin
	       if(cc==2) NS=s18;
			 else      NS=s17;
	  end
	  
	  s18: NS=s19;
	  
	  s19: 
	  begin
	       if(IData_en_wire) NS=s20;
			 else              NS=s19;
	  end
	  
	  s20:
	  begin
	       if(cc==2) NS=s21;
			 else      NS=s19;
	  end
	  
	  s21: NS=s22;
	  s22: NS=s23;
	  s23: NS=s24;
	  
	  s24:
	  begin
	       if(cc3==profile_len) NS=s0;
			 else                 NS=s19;
	  end
	  
	  s25:
	  begin
	       if(IData_en_wire) NS=s26;
			 else              NS=s25;
	  end
	  
	  s26:
	  begin
	       if(cc==8) NS=s27;
			 else      NS=s25;
	  end
	  
	  s27:
	  begin
	       if(cc2==8) NS=s0;
			 else       NS=s25;
	  end
	  
	  s28:
	  begin
	       if(IData_en_wire) NS=s29;
			 else              NS=s28;
	  end
	  
	  s29:
	  begin
	       if(cc==8) NS=s30;
			 else      NS=s28;
	  end
	  
	  s30: NS=s31;
	  
	  s31:
	  begin
	       if(cc==2) NS=s32;
			 else      NS=s31;
	  end
	  
	  s32:
	  begin
	       if(cc3==4096 && running==0)      NS=s0;
			 else if(cc3==2048 && running==1) NS=s0;
			 else                             NS=s28;
	  end
	  
	  s33:
	  begin
	       if(IData_en_wire) NS=s34;
			 else              NS=s33;
	  end
	  
	  s34:
	  begin
	       if(cc==4) NS=s35;
			 else      NS=s33;
	  end
	  
	  s35: NS=s36;
	  
	  s36:
	  begin
	       if(IData_en_wire) NS=s37;
			 else              NS=s36;
	  end
	  
	  s37:
	  begin
	       if(cc==4) NS=s38;
			 else      NS=s36;
	  end
	  
	  s38: NS=s39;
	  
	  s39:
	  begin
	       if(cc==2) NS=s40;
			 else      NS=s39;
	  end
	  
	  s40:
	  begin
	       if(cc3==duo_pulsar_table_len) NS=s0;
			 else                          NS=s36;
	  end
	  
	  s41: NS=s41_1; // 00 FF A1
	  
	  s41_1: 
	  begin
	       if(data_busy==0) NS=s42;
			 else             NS=s41_1;
	  end
	  
	  s42: NS=s43;
	  s43: NS=s44;
	  s44: NS=s45;
	  
	  s45:
	  begin
	       if(done_wire)
			 begin
			      if(cc==3) NS=s0;
					else      NS=s43;
			 end
			 
			 else NS=s45;
	  end
	  
	  default: NS=s0;
	  endcase
end

always @ (posedge clk or negedge rst)
begin
     if(! rst)
	  begin
	       code0<=0;
			 code1<=0;
			 code2<=0;
			 cc<=0;
			 OData_en_reg<=0;
			 double_speed<=0;
			 duo_pulsar<=0;
			 running<=0;
			 static_experiment<=0;
			 rate<=0;
			 start_time<=0;
			 EXP_duration<=0;
			 data0<=0;
			 data1<=0;
			 data2<=0;
			 data3<=0;
			 data4<=0;
			 data5<=0;
			 data6<=0;
			 data7<=0;
			 sat_x<=0;
			 sat_y<=0;
			 sat_z<=0;
			 sat_vx<=0;
			 sat_vy<=0;
			 sat_vz<=0;
			 cc2<=0;
			 cc3<=0;
			 profile_len<=0;
			 profile1_data<=0;
          profile1_data_en<=0;
			 profile2_data<=0;
          profile2_data_en<=0;
			 profile3_data<=0;
          profile3_data_en<=0;
			 profile4_data<=0;
          profile4_data_en<=0;
			 p1_n1<=0; p1_n2<=0; p1_n3<=0; p1_F0G<=0; p1_F1G<=0; p1_F2G<=0; p1_Pepoch<=0; p1_phase0<=0;
			 p2_n1<=0; p2_n2<=0; p2_n3<=0; p2_F0G<=0; p2_F1G<=0; p2_F2G<=0; p2_Pepoch<=0; p2_phase0<=0;
			 p3_n1<=0; p3_n2<=0; p3_n3<=0; p3_F0G<=0; p3_F1G<=0; p3_F2G<=0; p3_Pepoch<=0; p3_phase0<=0;
			 p4_n1<=0; p4_n2<=0; p4_n3<=0; p4_F0G<=0; p4_F1G<=0; p4_F2G<=0; p4_Pepoch<=0; p4_phase0<=0;
			 EarthData<=0;
			 EarthData_en<=0;
			 duo_pulsar_table_en<=0;
			 duo_pulsar_table<=0;
			 duo_pulsar_table_len<=0;
			 code_busy<=0;
	  end
	  
	  else
	  begin
	       case(NS)
			 s0:
			 begin
			      code0<=0;
			      code1<=0;
			      code2<=0;
					OData_en_reg<=0;
					data0<=0;
			      data1<=0;
			      data2<=0;
			      data3<=0;
			      data4<=0;
			      data5<=0;
			      data6<=0;
			      data7<=0;
					cc<=0;
					cc2<=0;
					cc3<=0;
					EarthData_en<=0;
					duo_pulsar_table_en<=0;
					code_busy<=0;
					profile1_data_en<=0;
					profile2_data_en<=0;
					profile3_data_en<=0;
					profile4_data_en<=0;
					
					if(EXP_duration_done) running<=0;
			 end
			 
			 s1:
			 begin
			      code2<=code1;
					code1<=code0;
					code0<=IData_wire;
			 end
			 
			 s2:
			 begin
			      cc<=0;
			 end
			 
			 s3:
			 begin
			      code_busy<=1;
			 end
			 
			 s3_1:
			 begin
			      if(cc==0) OData_reg<=code2;
					if(cc==1) OData_reg<=code1;
					if(cc==2) OData_reg<=code0;
					OData_en_reg<=0;
			 end
			 
			 s4:
			 begin
			      cc<=cc+1;
					OData_en_reg<=1;
			 end
			 
			 s5:
			 begin
			      OData_en_reg<=0;
			 end
			 
			 s6:
			 begin
			      cc<=0;
					cc2<=0;
					cc3<=0;
					code_busy<=0;
			 end
			 
			 s7:
			 begin
			      case(code0)
					8'h0B: double_speed<=1;
			      8'h0D: duo_pulsar<=1;
			      8'h04: running<=1;
			      8'h05: running<=0;
			      8'h12: double_speed<=0;
			      8'h13: duo_pulsar<=0;
			      8'h14: static_experiment<=1;
			      8'h15: static_experiment<=0;
			      8'h16: rate<=1; //send aircraft station data per 1 second
			      8'h17: rate<=2; //send aircraft station data per 100 millisecond
			      8'h18: rate<=3; //send aircraft station data per 10 millisecond
			      8'h19: rate<=4; //send aircraft station data per 1 millisecond
					8'h20: rate<=0; //do not send data
					endcase
			 end
			 
			 s8:
			 begin
			 end
			 
			 s9:
			 begin
			      cc<=cc+1;
			      data0<=IData_wire;
					data1<=data0;
					data2<=data1;
					data3<=data2;
					data4<=data3;
					data5<=data4;
					data6<=data5;
					data7<=data6;
			 end
			 
			 s10:
			 begin
			      if(code0==8'h01) start_time<={data7,data6,data5,data4,data3,data2,data1,data0};
					if(code0==8'h02) EXP_duration<={data7,data6,data5,data4,data3,data2,data1,data0};
			 end
			 
			 s11:
			 begin
			 end
			 
			 s12:
			 begin
			      cc<=cc+1;
			      data0<=IData_wire;
					data1<=data0;
					data2<=data1;
					data3<=data2;
					data4<=data3;
					data5<=data4;
					data6<=data5;
					data7<=data6;
			 end
			 
			 s13:
			 begin
			      cc<=0;
					cc2<=cc2+1;
					if(cc2==0) sat_x<={data7,data6,data5,data4,data3,data2,data1,data0};
					if(cc2==1) sat_y<={data7,data6,data5,data4,data3,data2,data1,data0};
					if(cc2==2) sat_z<={data7,data6,data5,data4,data3,data2,data1,data0};
					if(cc2==3) sat_vx<={data7,data6,data5,data4,data3,data2,data1,data0};
					if(cc2==4) sat_vy<={data7,data6,data5,data4,data3,data2,data1,data0};
					if(cc2==5) sat_vz<={data7,data6,data5,data4,data3,data2,data1,data0};
			 end
			 
			 s14:
			 begin
			 end
			 
			 s15:
			 begin
			      cc<=cc+1;
			      data0<=IData_wire;
					data1<=data0;
			 end
			 
			 s16:
			 begin
			      cc<=0;
			      profile_len<={data1,data0};
					profile1_data_en<=0;
					profile2_data_en<=0;
					profile3_data_en<=0;
					profile4_data_en<=0;
					if(code0==8'h07) profile1_data<={data1,data0};
					if(code0==8'h08) profile2_data<={data1,data0};
					if(code0==8'h09) profile3_data<={data1,data0};
					if(code0==8'h0A) profile4_data<={data1,data0};
			 end
			 
			 s17:
			 begin
			      cc<=cc+1;
			      cc3<=0;
					if(code0==8'h07) profile1_data_en<=1;
					if(code0==8'h08) profile2_data_en<=1;
					if(code0==8'h09) profile3_data_en<=1;
					if(code0==8'h0A) profile4_data_en<=1;
			 end
			 
			 s18:
			 begin 
			      cc<=0;
			      profile1_data_en<=0;
					profile2_data_en<=0;
					profile3_data_en<=0;
					profile4_data_en<=0;
			 end
			 
			 s19:
			 begin
			 end
			 
			 s20:
			 begin
			      cc<=cc+1;
					data0<=IData_wire;
					data1<=data0;
			 end
			 
			 s21:
			 begin
			      cc<=0;
					if(code0==8'h07) profile1_data<={data1,data0};
					if(code0==8'h08) profile2_data<={data1,data0};
					if(code0==8'h09) profile3_data<={data1,data0};
					if(code0==8'h0A) profile4_data<={data1,data0};
					profile1_data_en<=0;
					profile2_data_en<=0;
					profile3_data_en<=0;
					profile4_data_en<=0;
			 end
			 
			 s22:
			 begin
					if(code0==8'h07) profile1_data_en<=1;
					if(code0==8'h08) profile2_data_en<=1;
					if(code0==8'h09) profile3_data_en<=1;
					if(code0==8'h0A) profile4_data_en<=1;
			 end
			 
			 s23:
			 begin
			      cc3<=cc3+1;
			 end
			 
			 s24:
			 begin
			      profile1_data_en<=0;
					profile2_data_en<=0;
					profile3_data_en<=0;
					profile4_data_en<=0;
			 end
			 
			 s25:
			 begin
			 end
			 
			 s26:
			 begin
			      cc<=cc+1;
			      data0<=IData_wire;
					data1<=data0;
					data2<=data1;
					data3<=data2;
					data4<=data3;
					data5<=data4;
					data6<=data5;
					data7<=data6;
			 end
			 
			 s27:
			 begin
			      cc<=0;
			      cc2<=cc2+1;
					if(cc2==0)
					begin
					     if(code0==8'h0E) p1_n1<={data7,data6,data5,data4,data3,data2,data1,data0};
						  if(code0==8'h0F) p2_n1<={data7,data6,data5,data4,data3,data2,data1,data0};
						  if(code0==8'h10) p3_n1<={data7,data6,data5,data4,data3,data2,data1,data0};
						  if(code0==8'h11) p4_n1<={data7,data6,data5,data4,data3,data2,data1,data0};
					end
					if(cc2==1)
					begin
					     if(code0==8'h0E) p1_n2<={data7,data6,data5,data4,data3,data2,data1,data0};
						  if(code0==8'h0F) p2_n2<={data7,data6,data5,data4,data3,data2,data1,data0};
						  if(code0==8'h10) p3_n2<={data7,data6,data5,data4,data3,data2,data1,data0};
						  if(code0==8'h11) p4_n2<={data7,data6,data5,data4,data3,data2,data1,data0};
					end
					if(cc2==2)
					begin
					     if(code0==8'h0E) p1_n3<={data7,data6,data5,data4,data3,data2,data1,data0};
						  if(code0==8'h0F) p2_n3<={data7,data6,data5,data4,data3,data2,data1,data0};
						  if(code0==8'h10) p3_n3<={data7,data6,data5,data4,data3,data2,data1,data0};
						  if(code0==8'h11) p4_n3<={data7,data6,data5,data4,data3,data2,data1,data0};
					end
					if(cc2==3)
					begin
					     if(code0==8'h0E) p1_F0G<={data7,data6,data5,data4,data3,data2,data1,data0};
						  if(code0==8'h0F) p2_F0G<={data7,data6,data5,data4,data3,data2,data1,data0};
						  if(code0==8'h10) p3_F0G<={data7,data6,data5,data4,data3,data2,data1,data0};
						  if(code0==8'h11) p4_F0G<={data7,data6,data5,data4,data3,data2,data1,data0};
					end
					if(cc2==4)
					begin
					     if(code0==8'h0E) p1_F1G<={data7,data6,data5,data4,data3,data2,data1,data0};
						  if(code0==8'h0F) p2_F1G<={data7,data6,data5,data4,data3,data2,data1,data0};
						  if(code0==8'h10) p3_F1G<={data7,data6,data5,data4,data3,data2,data1,data0};
						  if(code0==8'h11) p4_F1G<={data7,data6,data5,data4,data3,data2,data1,data0};
					end
					if(cc2==5)
					begin
					     if(code0==8'h0E) p1_F2G<={data7,data6,data5,data4,data3,data2,data1,data0};
						  if(code0==8'h0F) p2_F2G<={data7,data6,data5,data4,data3,data2,data1,data0};
						  if(code0==8'h10) p3_F2G<={data7,data6,data5,data4,data3,data2,data1,data0};
						  if(code0==8'h11) p4_F2G<={data7,data6,data5,data4,data3,data2,data1,data0};
					end
					if(cc2==6)
					begin
					     if(code0==8'h0E) p1_Pepoch<={data7,data6,data5,data4,data3,data2,data1,data0};
						  if(code0==8'h0F) p2_Pepoch<={data7,data6,data5,data4,data3,data2,data1,data0};
						  if(code0==8'h10) p3_Pepoch<={data7,data6,data5,data4,data3,data2,data1,data0};
						  if(code0==8'h11) p4_Pepoch<={data7,data6,data5,data4,data3,data2,data1,data0};
					end
					if(cc2==7)
					begin
					     if(code0==8'h0E) p1_phase0<={data7,data6,data5,data4,data3,data2,data1,data0};
						  if(code0==8'h0F) p2_phase0<={data7,data6,data5,data4,data3,data2,data1,data0};
						  if(code0==8'h10) p3_phase0<={data7,data6,data5,data4,data3,data2,data1,data0};
						  if(code0==8'h11) p4_phase0<={data7,data6,data5,data4,data3,data2,data1,data0};
					end
			 end
			 
			 s28:
			 begin
			 end
			 
			 s29:
			 begin
			      cc<=cc+1;
			      data0<=IData_wire;
					data1<=data0;
					data2<=data1;
					data3<=data2;
					data4<=data3;
					data5<=data4;
					data6<=data5;
					data7<=data6;	
			 end
			 
			 s30:
			 begin
			      cc<=0;
					EarthData<={data7,data6,data5,data4,data3,data2,data1,data0};
					cc3<=cc3+1;
					EarthData_en<=0;
			 end
			 
			 s31:
			 begin
			      EarthData_en<=1;
					cc<=cc+1;
			 end
			 
			 s32:
			 begin
			      cc<=0;
					EarthData_en<=0;
			 end
			 
			 s33:
			 begin
			 end
			 
			 s34:
			 begin
			      cc<=cc+1;
					data0<=IData_wire;
					data1<=data0;
					data2<=data1;
					data3<=data2;
			 end
			 
			 s35:
			 begin
			      cc<=0;
					duo_pulsar_table_len<={data3,data2,data1,data0};
			 end
			 
			 s36:
			 begin
			 end
			 
			 s37:
			 begin
			      cc<=cc+1;
					data0<=IData_wire;
					data1<=data0;
					data2<=data1;
					data3<=data2;
			 end
			 
			 s38:
			 begin
			      cc<=0;
					duo_pulsar_table_en<=0;
					if( {data3,data2,data1,data0} < 1000000000 )
					begin
					     duo_pulsar_table[31:29]<=1; //pulsar1
						  duo_pulsar_table[28:0]<={data3[4:0],data2,data1,data0};
					end
					
					else if( {data3,data2,data1,data0}>=1000000000 &&{data3,data2,data1,data0}<2000000000 )
					begin
					     duo_pulsar_table[31:29]<=2; //pulsar2
						  duo_pulsar_table[28:0]<={data3,data2,data1,data0}-1000000000;
					end
					
					else if( {data3,data2,data1,data0}>=2000000000 &&{data3,data2,data1,data0}<3000000000 )
					begin
					     duo_pulsar_table[31:29]<=3; //pulsar3
						  duo_pulsar_table[28:0]<={data3,data2,data1,data0}-2000000000;
					end
					
					else if( {data3,data2,data1,data0}>=3000000000 &&{data3,data2,data1,data0}<4000000000 )
					begin
					     duo_pulsar_table[31:29]<=4; //pulsar4
						  duo_pulsar_table[28:0]<={data3,data2,data1,data0}-3000000000;
					end
			 end
			 
			 s39:
			 begin
			      duo_pulsar_table_en<=1;
					cc<=cc+1;
			 end
			 
			 s40:
			 begin
			      cc3<=cc3+1;
					cc<=0;
					duo_pulsar_table_en<=0;
			 end
			 
			 s41: // 00 FF A1
			 begin
			      OData_en_reg<=0;
					code_busy<=1;
					if(request_data) code0<=8'hA1;
					if(time_out) code0<=8'hA2;
					if(ERROR1) code0<=8'hA3;
			 end
			 
			 s41_1:
			 begin
			 end
			 
			 s42:
			 begin
			      code2<=8'h00;
					code1<=8'hFF;
			 end
			 
			 s43:
			 begin
			      if(cc==0) OData_reg<=code2;
					if(cc==1) OData_reg<=code1;
					if(cc==2) OData_reg<=code0;
					OData_en_reg<=1;
			 end
			 
			 s44:
			 begin
			      OData_en_reg<=1;
					cc<=cc+1;
			 end
			 
			 s45:
			 begin
			      OData_en_reg<=0;
			 end
			 
			 default:
			 begin
			 end
			 endcase
	  end
end
//*******************************************************//

//**********************send aircraft state to PC********//
reg [31:0] rate_cc;
always @ (posedge clk or negedge rst)
begin
     if(!rst) CS2<=s0;
	  else     CS2<=NS2;
end

always @ (*)
begin
     case(CS2)
	  s0:
	  begin
			 if(rate==0) NS2=s0;
			 else
			 begin
			      if(state_en) NS2=s1;
					else         NS2=s0;
			 end
	  end
	  
	  s1: NS2=s1_a;
	  
	  s1_a:
	  begin
	       if(rate==1 && rate_cc==100000) NS2=s2;
			 else if(rate==2 && rate_cc==10000) NS2=s2;
			 else if(rate==3 && rate_cc==1000) NS2=s2;
			 else NS2=s0;
	  end
	  
	  
	  
	  //s1: NS2=s2;
	  
	  s2:
	  begin
	       if(code_busy==0) NS2=s3_1;
			 else             NS2=s2;
	  end
	  
	  s3_1:
	  begin
	       if(code_busy==0) NS2=s3;
			 else             NS2=s3_1;
	  end
	  
	  s3: NS2=s4;
	  s4: NS2=s5;
	  s5: NS2=s6;
	  
	  s6:
	  begin
	       if(done_wire) 
			 begin
			      if(cc4==8)
					begin
					     if(cc5==6) NS2=s0;
						  else       NS2=s7;
					end
					else NS2=s3;
			 end
			 else NS2=s6;
	  end
	  
	  s7: NS2=s3;
	  
	  default: NS2=s0;
	  endcase
end

always @ (posedge clk or negedge rst)
begin
     if(!rst)
	  begin
	       cc4<=0;
			 cc5<=0;
			 data_busy<=0;
			 sat_station[0]<=0;
			 sat_station[1]<=0;
			 sat_station[2]<=0;
			 sat_station[3]<=0;
			 sat_station[4]<=0;
			 sat_station[5]<=0;
			 sat_station[6]<=0;
			 OData2_en_reg<=0;
			 OData2_reg<=0;
			 rate_cc<=0;
	  end
	  
	  else
	  begin
	       case(NS2)
			 s0:
			 begin
			      cc4<=0;
					cc5<=0;
					data_busy<=0;
					OData2_en_reg<=0;
			      OData2_reg<=0;
					//rate_cc<=0;
			 end
			 
			 s1:
			 begin
			      sat_station[0]<=state_t;
			      sat_station[1]<=state_x;
			      sat_station[2]<=state_y;
			      sat_station[3]<=state_z;
			      sat_station[4]<=state_vx;
			      sat_station[5]<=state_vy;
			      sat_station[6]<=state_vz;
			 end
			 
			 s1_a:
			 begin
			      rate_cc<=rate_cc+1;
			 end
			 
			 s2:
			 begin
			      rate_cc<=0;
			 end
			 
			 s3_1:
			 begin
			 end
			 
			 s3:
			 begin
			      data_busy<=1;
					if(cc5==0) OData2_reg<=sat_station[0][63:56];
					if(cc5==1) OData2_reg<=sat_station[1][63:56];
					if(cc5==2) OData2_reg<=sat_station[2][63:56];
					if(cc5==3) OData2_reg<=sat_station[3][63:56];
					if(cc5==4) OData2_reg<=sat_station[4][63:56];
					if(cc5==5) OData2_reg<=sat_station[5][63:56];
					if(cc5==6) OData2_reg<=sat_station[6][63:56];
			 end
			 
			 s4:
			 begin
			      if(cc5==0) sat_station[0]<=sat_station[0]<<8;
					if(cc5==1) sat_station[1]<=sat_station[1]<<8;
					if(cc5==2) sat_station[2]<=sat_station[2]<<8;
					if(cc5==3) sat_station[3]<=sat_station[3]<<8;
					if(cc5==4) sat_station[4]<=sat_station[4]<<8;
					if(cc5==5) sat_station[5]<=sat_station[5]<<8;
					if(cc5==6) sat_station[6]<=sat_station[6]<<8;
					OData2_en_reg<=1;
			 end
			 
			 s5:
			 begin
			      cc4<=cc4+1;
			 end
			 
			 s6:
			 begin
			      OData2_en_reg<=0;
			 end
			 
			 s7:
			 begin
			      cc5<=cc5+1;
					cc4<=0;
			 end
			 
			 default:
			 begin
			      cc4<=0;
					cc5<=0;
					data_busy<=0;
					OData2_en_reg<=0;
			      OData2_reg<=0;
			 end
			 endcase
	  end
end
//*******************************************************//

TX_232 TX_PC(
       .clk(clk),
		 .rst(rst),
		 .OData(OData_wire),
		 .OData_en(OData_en_wire),
		 .TX(TX),
		 .done(done_wire)
		 );
//assign OData_wire=OData_reg;
//assign OData_en_wire=OData_en_reg;
assign OData_wire=(data_busy==0)? OData_reg : OData2_reg;
assign OData_en_wire=(data_busy==0)? OData_en_reg : OData2_en_reg;
		 
RX_232 RX_PC(
       .clk(clk),
		 .rst(rst),
		 .RX(RX),
		 .IData(IData_wire),
		 .IData_en(IData_en_wire)
		 );

endmodule
