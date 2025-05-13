
`timescale 1ns/1ps

module TB_Lidar_top_level();

import load_stim_pkg::*;
import write_out_pkg::*;

logic clk_i;
logic rstn_i;
logic testmode_i;
logic valid_data_i;
logic [7:0] data_i;
logic valid_datapoint_CCM_o;
logic ready_DDM_o;
logic signed [17:0] x_o, y_o, z_o;
logic [7:0] nr_packets;

	// Create interface instance
  AXI_STREAM_BUS_DV #(.DataWidth(8)) axi_if (.clk_i(clk_i));

  // Virtual interface to pass to class
virtual AXI_STREAM_BUS_DV #(.DataWidth(8)) v_axi_if = axi_if;

//instance od driver object in my tb
axi_stream_test::axi_stream_driver #(.DataWidth(8)) axi_tx;

Lidar_top_level DUT(
	.rstn_i(rstn_i),
	.clk_i (clk_i),
	.data_i(axi_if.tdata),
	.valid_data_i(axi_if.tvalid),
	.valid_datapoint_CCM_o(valid_datapoint_CCM_o),
	.x_o(x_o),
	.y_o(y_o),
	.z_o(z_o),
	.ready_DDM_o(axi_if.tready),
	.testmode_i(testmode_i),
	.nr_packets(nr_packets)
);

int length_sim;
int index=0;
int out_idx = 0;

// Declare stimuli vectors
logic [7:0] bytes[$];

assign length_sim = $bits(bytes)/8;

// Declare output vectors
logic signed [17:0] x[$], y[$], z[$];


// Load stimuli from .txt files
initial begin
	load_bytes("../tb/stimuli/frame.txt", bytes);
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
ready_upsizer_i=1'b1;
//valid_data_i=1'b0;
axi_tx=new(v_axi_if);
axi_tx.reset_tx();
wait(rstn_i)
@(posedge clk_i);
while(index<=length_sim) begin
axi_tx.send(bytes[index], 1'b0);
data_i.tdata=bytes[index];

if(valid_datapoint_CCM_o) begin
	x[out_idx] = x_o;
	y[out_idx] = y_o;
	z[out_idx] = z_o;
	//$display("x=%d", x[out_idx] );

	out_idx++;
end 
index++;
end

write_x_out(x, "x_out.txt");
write_y_out(y, "y_out.txt");
write_z_out(z, "z_out.txt");
$display("Number of packets: %d", nr_packets);

repeat(5) @(posedge clk_i);

$stop;
end

endmodule
