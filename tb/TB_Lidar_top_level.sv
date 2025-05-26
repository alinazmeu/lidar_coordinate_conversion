
`timescale 1ns/1ps
`include "axi_stream/assign.svh"
`include "axi_stream/typedef.svh"

module TB_Lidar_top_level();

import load_stim_pkg::*;
import write_out_pkg::*;
import axi_stream_test::*;

logic clk_i, rstn_i;
logic [7:0] nr_packets;

localparam int unsigned IdWidth=0;
localparam int unsigned DestWidth=0;

// ---------------- axis streams for the framing module ----------------------//
  localparam int unsigned FramingDataWidth = 8;
  localparam int unsigned FramingIdWidth   = IdWidth;
  localparam int unsigned FramingDestWidth = DestWidth;
  localparam int unsigned FramingUserWidth = 1;

// AXI stream channels typedefs
  typedef logic [FramingDataWidth-1:0]   framing_tdata_t;
  typedef logic [FramingDataWidth/8-1:0] framing_tstrb_t;
  typedef logic [FramingDataWidth/8-1:0] framing_tkeep_t;
  typedef logic [FramingIdWidth-1:0]     framing_tid_t;
  typedef logic [FramingDestWidth-1:0]   framing_tdest_t;
  typedef logic [FramingUserWidth-1:0]   framing_tuser_t;

  `AXI_STREAM_TYPEDEF_ALL(s_framing, framing_tdata_t, framing_tstrb_t, framing_tkeep_t, framing_tid_t, framing_tdest_t, framing_tuser_t)


  // ---------------- axis streams for the hwa module ----------------------//
  localparam int unsigned HwaDataWidth = 16;
  localparam int unsigned HwaIdWidth   = IdWidth;
  localparam int unsigned HwaDestWidth = DestWidth;
  localparam int unsigned HwaUserWidth = 1;

// AXI stream channels typedefs
  typedef logic [HwaDataWidth-1:0]   hwa_tdata_t;
  typedef logic [HwaDataWidth/8-1:0] hwa_tstrb_t;
  typedef logic [HwaDataWidth/8-1:0] hwa_tkeep_t;
  typedef logic [HwaIdWidth-1:0]     hwa_tid_t;
  typedef logic [HwaDestWidth-1:0]   hwa_tdest_t;
  typedef logic [HwaUserWidth-1:0]   hwa_tuser_t;

  `AXI_STREAM_TYPEDEF_ALL(s_hwa, hwa_tdata_t, hwa_tstrb_t, hwa_tkeep_t, hwa_tid_t, hwa_tdest_t, hwa_tuser_t)


// AXI stream signals packed struct signals for in interface
  s_framing_req_t  s_framing_rx_req;
  s_framing_rsp_t  s_framing_rx_rsp;
// AXI stream packed struct signals for out interface 
  s_hwa_req_t s_hwa_rx_req;
  s_hwa_rsp_t s_hwa_rx_rsp;


/*****AXIS-S INJECTION ******/

  //AXI interface instance Input/Output port
  AXI_STREAM_BUS_DV #( .DataWidth(8)  ) axi_in_if  (  .clk_i(clk_i) );
  AXI_STREAM_BUS_DV #( .DataWidth(16) ) axi_out_if (  .clk_i(clk_i) );

  //instance of stream_driver class from stream_test package
  axi_stream_test::axi_stream_driver #( .DataWidth(8)  ) axi_in_rx;
  axi_stream_test::axi_stream_driver #( .DataWidth(16) ) axi_out_rx;

  // AXI Virtual interface instance 
  virtual AXI_STREAM_BUS_DV #( .DataWidth(8)  ) v_axi_in_if  =  axi_in_if;
  virtual AXI_STREAM_BUS_DV #( .DataWidth(16) ) v_axi_out_if =  axi_out_if; 


assign s_framing_rx_req.t.data=axi_in_if.tdata;
assign s_framing_rx_req.tvalid=axi_in_if.tvalid;
assign axi_in_if.tready=s_framing_rx_rsp.tready;   
assign s_framing_rx_req.t.keep=axi_in_if.tkeep;   
assign s_framing_rx_req.t.strb=axi_in_if.tstrb;
assign s_framing_rx_req.t.id=axi_in_if.tid;
assign s_framing_rx_req.t.dest=axi_in_if.tdest;  
assign s_framing_rx_req.t.user=axi_in_if.tuser;   
assign s_framing_rx_req.t.last=axi_in_if.tlast;                                                                   


assign axi_out_if.tdata=s_hwa_rx_req.t.data;
assign axi_out_if.tvalid=s_hwa_rx_req.tvalid;
assign s_hwa_rx_rsp.tready=axi_out_if.tready;

Hwa_top_level #(
	.axi_stream_in_req_t(s_framing_req_t),
	.axi_stream_in_rsp_t(s_framing_rsp_t),
	.axi_stream_out_req_t(s_hwa_req_t),
	.axi_stream_out_rsp_t(s_hwa_rsp_t)
) DUT (
	.rstn_i(rstn_i),
	.clk_i (clk_i),
	.nr_packets(nr_packets),
	.axis_in_req_i(s_framing_rx_req),
	.axis_in_rsp_o(s_framing_rx_rsp),
	.axis_out_req_o(s_hwa_rx_req),
	.axis_out_rsp_i(s_hwa_rx_rsp)
);


int length_sim;
int index=0;
int out_idx = 0;
// Declare stimuli vectors
logic [7:0] bytes[$];
// Declare output vectors for serial output
logic signed [15:0] serial[$];
// Declare output vectors for parallel output
logic signed [15:0] x[$], y[$], z[$]; 

assign length_sim = $bits(bytes)/8;

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
	#15;
	rstn_i=1'b1;
	
end

initial begin
	axi_out_rx=new(v_axi_out_if);
	axi_out_rx.reset_tx();
  axi_out_if.tready=1'b1;
	if(axi_out_if.tready) axi_out_rx.send(axi_out_if.tdata, 1'b0);
end


int f;
int index_f=0;
int index_x=0;
int index_y=0;
int index_z=0;
initial begin
  wait(rstn_i)
  @(posedge clk_i);
  while (index_f<=length_sim) begin
      if (f==0 && s_hwa_rx_req.tvalid) begin
        x[index_x]=s_hwa_rx_req.t.data;
        index_x++;
        index_f++;
        f++;
      end
      else if (f==1 && s_hwa_rx_req.tvalid ) begin
        y[index_y]=s_hwa_rx_req.t.data;
        index_y++;
        index_f++;
        f++;
      end
      else if (f==2 && s_hwa_rx_req.tvalid) begin
        z[index_z]=s_hwa_rx_req.t.data;
        index_z++;
        index_f++;
        f=0;
      end
      @(posedge clk_i);
  end
end

initial begin
axi_in_rx=new(v_axi_in_if); //virtual interface is passed to the driver 
axi_in_rx.reset_tx();

wait(rstn_i)
@(posedge clk_i);
while(index<=length_sim) begin
	axi_in_rx.send(bytes[index], 1'b0); //reading from the txt file
	if(axi_out_if.tvalid) begin
		serial[out_idx] = axi_out_if.tdata; 
		out_idx++;
	end
  /* if(index==120) begin       //to debug phy_error behaviour
    @(posedge clk_i);
    axi_in_if.tlast=1;
    axi_in_if.tvalid=1;
    axi_in_if.tdata=8'hAA;
    @(posedge clk_i);
  end */
	index++;
end

write_serial_out(serial, "serial_out_txt");
write_x_out(x, "x_out.txt");
write_y_out(y, "y_out.txt");
write_z_out(z, "z_out.txt"); 
$display("Number of packets: %d", nr_packets);

repeat(5) @(posedge clk_i);

$stop;
end


endmodule
