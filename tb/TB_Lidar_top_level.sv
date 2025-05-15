
`timescale 1ns/1ps

module TB_Lidar_top_level();

import load_stim_pkg::*;
import write_out_pkg::*;
import axi_stream_test::*;

logic clk_i;
logic rstn_i;

//Input AXI-S (from framing top)
logic valid_data_i;
logic [7:0] data_i;
logic ready_DDM_o;

//Output AXI-S (to upsizer)
logic valid_serial_o;
logic signed [15:0] data_o;
logic ready_upsizer_i;

//control logic signals 
logic [7:0] nr_packets;


/*****AXIS-S INJECTION ******/

  //AXI interface instance
  AXI_STREAM_BUS_DV #(.DataWidth(8)) axi_if (.clk_i(clk_i));

  // AXI Virtual interface instance 
  virtual AXI_STREAM_BUS_DV #(.DataWidth(8)) v_axi_if = axi_if;

//instance of stream_driver class from stream_test package
axi_stream_test::axi_stream_driver #(.DataWidth(8)) axi_tx;

//DEVICE UNDER TEST
Lidar_top_level DUT(
	.rstn_i(rstn_i),
	.clk_i (clk_i),
	.data_i(axi_if.tdata),
	.valid_data_i(axi_if.tvalid),
	.ready_DDM_o(axi_if.tready),
	.valid_serial_o(valid_serial_o),
	.ready_upsizer_i(ready_upsizer_i),
	.data_o(data_o),
	.nr_packets(nr_packets)
);

int length_sim;
int index=0;
int out_idx = 0;

// Declare stimuli vectors
logic [7:0] bytes[$];

assign length_sim = $bits(bytes)/8;

// Declare output vectors
logic signed [15:0] serial[$];

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
ready_upsizer_i=1'b1;

axi_tx=new(v_axi_if); //virtual interface is passed to the driver 
axi_tx.reset_tx();

wait(rstn_i)
@(posedge clk_i);
while(index<=length_sim) begin
axi_tx.send(bytes[index], 1'b0);

if(valid_serial_o) begin
	serial[out_idx] = data_o;
	out_idx++;
end

index++;
end

write_serial_out(serial, "serial_out_txt");
$display("Number of packets: %d", nr_packets);

repeat(5) @(posedge clk_i);

$stop;
end

/****PARALLEL OUTPUT****/


// Declare output vectors
/* logic signed [17:0] x[$], y[$], z[$]; */

//in while loop
/* if(valid_datapoint_CCM_o) begin
	x[out_idx] = x_o;
	y[out_idx] = y_o;
	z[out_idx] = z_o;
	//$display("x=%d", x[out_idx] );

	out_idx++;
end */  


//out of the loop
/* write_x_out(x, "x_out.txt");
write_y_out(y, "y_out.txt");
write_z_out(z, "z_out.txt"); */


endmodule
