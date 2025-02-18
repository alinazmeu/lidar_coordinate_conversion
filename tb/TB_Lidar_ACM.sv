`timescale 1ns/1ps

module TB_Lidar_ACM();

logic clk_i;
logic rstn_i;
logic valid_DDM_i;
logic ready_CCM_i;
logic [15:0]azimuth_i; //[0, 36000]
logic signed [17:0] cosa1_o, sina1_o, sina2_o, cosa2_o;
logic valid1_ACM_o;
logic valid2_ACM_o;
logic ready_ACM_o;
logic valid_CCM_i;


Lidar_ACM DUT (
		.rstn_i(rstn_i),
		.clk_i (clk_i),
		.valid_DDM_i(valid_DDM_i),
		.valid_CCM_i(valid_CCM_i),
		.ready_CCM_i(ready_CCM_i),
		.azimuth_i(azimuth_i),
		.cosa1_o (cosa1_o),
		.sina1_o(sina1_o),
		.cosa2_o (cosa2_o),
		.sina2_o(sina2_o),
		.valid1_ACM_o(valid1_ACM_o),
		.valid2_ACM_o(valid2_ACM_o),
		.ready_ACM_o (ready_ACM_o)
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
@(posedge clk_i);
valid_DDM_i=1'd1;
ready_CCM_i='0;
azimuth_i=11256;


 repeat(40)@(posedge clk_i);
ready_CCM_i='0;
$stop;
end
endmodule