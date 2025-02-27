
module Lidar_top_level(
input logic rstn_i, 
input logic clk_i, 
input logic [3:0] channel_ID_i,
input logic [15:0] distance_i,
input logic [15:0]azimuth_i,
input logic valid_DDM_i,
output logic valid_fs1_CCM_o,
output logic valid_fs2_CCM_o,
output logic valid_fs1_ACM_o,
output logic valid_fs2_ACM_o,
output logic signed [17:0] x_o, y_o, z_o
);

logic signed [17:0] cosa1, sina1, cosa2, sina2;
logic ready_fs1_CCM, ready_fs2_CCM;
logic ready_fs2_ACM,ready_fs1_ACM;

Lidar_ACM lidar_acm (
		.rstn_i(rstn_i),
		.clk_i (clk_i),
		.valid_DDM_i(valid_DDM_i),
		.valid_fs1_CCM_i(valid_fs1_CCM_o),
		.valid_fs2_CCM_i(valid_fs2_CCM_o),
		.ready_fs1_CCM_i(ready_fs1_CCM),
		.ready_fs2_CCM_i(ready_fs2_CCM),
		.azimuth_i(azimuth_i),
		.cosa1_o (cosa1),
		.sina1_o(sina1),
		.cosa2_o (cosa2),
		.sina2_o(sina2),
		.valid_fs1_ACM_o(valid_fs1_ACM_o),
		.valid_fs2_ACM_o(valid_fs2_ACM_o),
		.ready_fs1_ACM_o (ready_fs1_ACM),
		.ready_fs2_ACM_o (ready_fs2_ACM)
	
);

Lidar_CCM lidar_ccm (
		.rstn_i(rstn_i),
		.clk_i (clk_i),
		.valid_fs1_ACM_i(valid_fs1_ACM_o),
		.valid_fs2_ACM_i(valid_fs2_ACM_o),
		.ready_fs1_ACM_i(ready_fs1_ACM),
		.ready_fs2_ACM_i(ready_fs2_ACM),
		.channel_ID_i(channel_ID_i),
		.distance_i (distance_i),
		.cosa1_i (cosa1),
		.sina1_i(sina1),
		.cosa2_i (cosa2),
		.sina2_i(sina2),
		.x_o(x_o),
		.y_o(y_o),
		.z_o(z_o),
		.ready_fs1_CCM_o (ready_fs1_CCM),
		.ready_fs2_CCM_o (ready_fs2_CCM),
		.valid_fs1_CCM_o (valid_fs1_CCM_o),
		.valid_fs2_CCM_o (valid_fs2_CCM_o)
);
endmodule
