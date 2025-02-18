`timescale 1ns/1ps

module TB_Lidar_CCM();

logic clk_i;
logic rstn_i;
logic valid1_ACM_i;
logic valid2_ACM_i;
logic [3:0] channel_ID_i;
logic signed [15:0] distance_i;
logic signed [17:0] sina1_i, cosa1_i, sina2_i, cosa2_i;
logic signed [17:0] x_o, y_o, z_o;
logic ready_CCM_o;
logic valid_CCM_o;

int num_sim=0;
int length_sim=15;
int id=0;

Lidar_CCM DUT (
		.rstn_i(rstn_i),
		.clk_i (clk_i),
		.valid1_ACM_i(valid1_ACM_i),
		.valid2_ACM_i(valid2_ACM_i),
		.channel_ID_i(channel_ID_i),
		.distance_i (distance_i),
		.cosa1_i (cosa1_i),
		.sina1_i(sina1_i),
		.cosa2_i (cosa2_i),
		.sina2_i(sina2_i),
		.x_o(x_o),
		.y_o(y_o),
		.z_o(z_o),
		.ready_CCM_o (ready_CCM_o),
		.valid_CCM_o(valid_CCM_o)
);

initial begin
	clk_i='0;
	while(1) begin
		#10;
		clk_i=~clk_i;
	end
end

initial begin
	rstn_i='0;
	#5;
	rstn_i=1'b1;
	
end

initial begin

wait(rstn_i)

//distanza massimo valore è 131070
//simulo un firing sequence
@(posedge clk_i);
valid1_ACM_i=1'd1;
valid2_ACM_i=0;
cosa1_i=18'sd99999;
sina1_i=18'd23456;
cosa2_i='0;
sina2_i='0;
while(num_sim<=length_sim) begin
@(posedge clk_i);
channel_ID_i=id;
distance_i=$urandom_range(65535);
id++;
num_sim++;
end

@(posedge clk_i);
valid2_ACM_i=1'd1;
cosa2_i=-18'd25648;
sina2_i=-18'd65489;
id=0;
num_sim=0;

while(num_sim<=length_sim) begin
channel_ID_i=id;
distance_i=$urandom_range(65535);
id++;
num_sim++;
@(posedge clk_i);
end

@(posedge clk_i);
@(posedge clk_i);
$stop;
end
endmodule