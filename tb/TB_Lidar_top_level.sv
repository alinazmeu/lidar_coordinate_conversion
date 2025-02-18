`timescale 1ns/1ps

module TB_Lidar_top_level();

logic clk_i;
logic rstn_i;
logic [3:0] channel_ID_i;
logic [15:0] distance_i;
logic [15:0]azimuth_i;
logic valid_DDM_i;
logic ready_ACM_o;
logic valid1_ACM_o;
logic valid2_ACM_o;
logic valid_CCM_o;
logic signed [17:0] x_o, y_o, z_o;

Lidar_top_level DUT(
	.rstn_i(rstn_i),
	.clk_i (clk_i),
	.channel_ID_i(channel_ID_i),
	.distance_i(distance_i),
	.azimuth_i(azimuth_i),
	.valid_DDM_i(valid_DDM_i),
	.ready_ACM_o(ready_ACM_o),
	.valid1_ACM_o(valid1_ACM_o),
	.valid2_ACM_o(valid2_ACM_o),
	.x_o(x_o),
	.y_o(y_o),
	.z_o(z_o),
	.valid_CCM_o(valid_CCM_o)
	
);

int length_sim=15;
int id=0;

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
@(posedge clk_i);
valid_DDM_i=1'd1;
azimuth_i=$urandom_range(36000); //if ready ccm 

@(posedge clk_i);
valid_DDM_i='0;


wait(valid1_ACM_o)
while(id<=length_sim) begin
@(posedge clk_i);
channel_ID_i=id;
distance_i=$urandom_range(65535);
id++;
end

wait(valid2_ACM_o)
id=0;
while(id<=length_sim) begin
channel_ID_i=id;
distance_i=$urandom_range(65535);
id++;
@(posedge clk_i);
$display("x_o: %d,\t y_o: %d", x_o, y_o);
end


repeat(5)@(posedge clk_i);

$stop;
end
endmodule
