module Lidar_top_level(
input logic rstn_i, 
input logic clk_i, 
input logic [3:0] channel_ID_i,
input logic [15:0] distance_i,
input logic [15:0]azimuth_i,
input logic valid_DDM_i,
output logic valid1_ACM_o,
output logic valid_CCM_o,
output logic valid2_ACM_o,
output logic ready_ACM_o,
output logic signed [17:0] x_o, y_o, z_o
);

logic signed [17:0] cosa1, sina1, cosa2, sina2;
logic ready_CCM;

Lidar_ACM lidar_acm (
		.rstn_i(rstn_i),
		.clk_i (clk_i),
		.valid_DDM_i(valid_DDM_i),
		.valid_CCM_i(valid_CCM_o),
		.ready_CCM_i(ready_CCM),
		.azimuth_i(azimuth_i),
		.cosa1_o (cosa1),
		.sina1_o(sina1),
		.cosa2_o (cosa2),
		.sina2_o(sina2),
		.valid1_ACM_o(valid1_ACM_o),
		.valid2_ACM_o(valid2_ACM_o),
		.ready_ACM_o (ready_ACM_o)
	
);

Lidar_CCM lidar_ccm (
		.rstn_i(rstn_i),
		.clk_i (clk_i),
		.valid1_ACM_i(valid1_ACM_o),
		.valid2_ACM_i(valid2_ACM_o),
		.channel_ID_i(channel_ID_i),
		.distance_i (distance_i),
		.cosa1_i (cosa1),
		.sina1_i(sina1),
		.cosa2_i (cosa2),
		.sina2_i(sina2),
		.x_o(x_o),
		.y_o(y_o),
		.z_o(z_o),
		.ready_CCM_o (ready_CCM),
		.valid_CCM_o (valid_CCM_o)
);
endmodule