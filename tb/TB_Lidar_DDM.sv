
`timescale 1ns/1ps

module TB_Lidar_DDM ();

import load_stim_pkg::*;

logic clk_i, rstn_i, ready_ACM_i, ready_CCM_i, valid_data_i, testmode_i;
logic valid_datapoint_DDM, valid_azimuth_DDM;
logic [15:0] azimuth_DDM, distance_DDM;
logic [7:0] data_i;
logic [3:0] id_DDM;


logic [18:0] [7:0] data = {8'hFF, 8'hEE, 8'h0, 8'h7, 8'h3, 8'h99, 8'h0, 8'h9, 8'h69, 8'h0, 8'h4, 8'h40, 8'h0,  8'h9, 8'h6F, 8'h0, 8'h5, 8'h2E, 8'h0 }; 



Lidar_DDM DUT(
.clk_i(clk_i),
.rstn_i(rstn_i),
.ready_ACM_i(ready_ACM_i),
.ready_CCM_i(ready_CCM_i),
.valid_data_i(valid_data_i),
.data_i(data_i),
.testmode_i(testmode_i),
.valid_datapoint_DDM_o(valid_data_point_DDM),
.valid_azimuth_DDM_o(valid_azimuth_DDM),
.azimuth_DDM_o(azimuth_DDM),
.distance_DDM_o(distance_DDM),
.id_DDM_o(id_DDM)
);

int length_sim;
int index=0;


// Declare stimuli vectors
logic [7:0] bytes[$];

assign length_sim = $bits(bytes)/8;

// Load stimuli from .txt files
initial begin
	load_bytes("../tb/stimuli/vlp_packet.txt", bytes);
end

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
testmode_i=1'b1;
ready_CCM_i=1'b0;
ready_ACM_i=1'b0;
valid_data_i=1'b0;
wait(rstn_i)
ready_ACM_i=1'b1;

while(index<=length_sim) begin
@(posedge clk_i);
valid_data_i=1'b1;
data_i=bytes[index];
index++;
end

$stop;
end

endmodule
