
`timescale 1ns/1ps

module TB_Lidar_top_level();

import load_stim_pkg::*;

logic clk_i;
logic rstn_i;
logic [3:0] channel_ID_i;
logic [15:0] distance_i;
logic [15:0]azimuth_i;
logic valid_DDM_i;
logic valid_fs1_CCM_o;
logic valid_fs2_CCM_o;
logic valid_fs1_ACM_o;
logic valid_fs2_ACM_o;
logic signed [17:0] x_o, y_o, z_o;

Lidar_top_level DUT(
	.rstn_i(rstn_i),
	.clk_i (clk_i),
	.channel_ID_i(channel_ID_i),
	.distance_i(distance_i),
	.azimuth_i(azimuth_i),
	.valid_DDM_i(valid_DDM_i),
	.valid_fs1_CCM_o(valid_fs1_CCM_o),
	.valid_fs2_CCM_o(valid_fs2_CCM_o),
	.valid_fs1_ACM_o(valid_fs1_ACM_o),
	.valid_fs2_ACM_o(valid_fs2_ACM_o),
	.x_o(x_o),
	.y_o(y_o),
	.z_o(z_o)
	
);

int length_sim;
int id=0;
int array_idx = 0;
int idx=0;

// Declare stimuli vectors
logic [15:0] azimuth[$];
logic [15:0] distance[$];

// Load stimuli from .txt files
initial begin
	load_azimuth("../tb/stimuli/azimuth.txt", azimuth);
	load_distance("../tb/stimuli/distance.txt", distance);
end

assign length_sim = $bits(distance)/16;

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
	while(idx<length_sim) begin
		wait(valid_fs1_ACM_o) 
		repeat(3) @(posedge clk_i);
		$display("azimuth%d", azimuth_i);
		for(int i=0; i<=15; i++) begin
			$display("ID:%d, Distance:%d, x:%d, y:%d, z:%d",i, distance[idx], x_o,y_o,z_o);
			@(posedge clk_i);
			idx++;
		end

		wait(valid_fs2_ACM_o)
		repeat(3) @(posedge clk_i);
		for(int i=0; i<=15; i++) begin
			$display("ID:%d, Distance:%d, x:%d, y:%d, z=%d",i, distance[idx], x_o,y_o,z_o);
			@(posedge clk_i);
			idx++;
		end
	end
end


initial begin

wait(rstn_i)
@(posedge clk_i); 
valid_DDM_i=1'd1; //ACM goes to COMPUTE1
azimuth_i = azimuth[array_idx];

@(posedge clk_i); //ACM in COMPUTE1
valid_DDM_i='0;

while(array_idx<length_sim) begin
	wait(valid_fs1_ACM_o) 
	@(posedge clk_i); //CCM in COMPUTE1
		while(id<=15) begin
			channel_ID_i=id;
			distance_i=distance[array_idx];
			id++;
			array_idx++;
			@(posedge clk_i); //id=15 CCM goes to VALID1 
		end
	id=0; 
	@(posedge clk_i); //CCM in VALID2
	wait(valid_fs2_ACM_o) //CCM goes to COMPUTE2
	@(posedge clk_i); // CCM in COMPUTE2

		while(id<=15) begin
			channel_ID_i=id;
			distance_i=distance[array_idx];
			@(posedge clk_i);
			id++;
			array_idx++; //id=15 CCM goes to VALID2
		end
	id=0;
	@(posedge clk_i); //CCM in VALID2
	@(posedge clk_i); //CCM  e ACM in IDLE 
	valid_DDM_i=1'd1; //ACM goes to COMPUTE1
	azimuth_i=azimuth[array_idx];
	@(posedge clk_i); //ACM in COMPUTE1
	valid_DDM_i='0;

end
repeat(5)@(posedge clk_i);


$stop;
end
endmodule
