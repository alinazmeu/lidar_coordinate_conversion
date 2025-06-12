

module Hwa_top_level #(

    /// AXI Stream in request struct
  parameter type axi_stream_in_req_t        = logic  ,
  /// AXI Stream in response struct
  parameter type axi_stream_in_rsp_t        = logic  ,
     /// AXI Stream out request struct
  parameter type axi_stream_out_req_t       = logic  ,
  /// AXI Stream out response struct
  parameter type axi_stream_out_rsp_t       = logic

) (
	input logic 									            rstn_i         , 
	input logic 									            clk_i          , 
	 // AXIS TX/RX
  input  axi_stream_in_req_t                axis_in_req_i  ,
  output axi_stream_in_rsp_t                axis_in_rsp_o  ,
  output axi_stream_out_req_t               axis_out_req_o ,
  input  axi_stream_out_rsp_t               axis_out_rsp_i ,
  output logic [7:0]                        nr_packets     ,
  output logic [11:0]                       hwa_length_o   ,
  output logic                              error_rx_o     
);

/*   assign axis_out_req_o.t.keep   =  axis_in_req_i.t.keep;
  assign axis_out_req_o.t.strb   =  axis_in_req_i.t.strb;
  assign axis_out_req_o.t.user   =  axis_in_req_i.t.user;
  assign axis_out_req_o.t.id     =  axis_in_req_i.t.id;
  assign axis_out_req_o.t.dest   =  axis_in_req_i.t.dest; */
  assign axis_out_req_o.t.keep   =  '0;
  assign axis_out_req_o.t.strb   =  '0;
  assign axis_out_req_o.t.user   =  '0;
  assign axis_out_req_o.t.id     =  '0;
  assign axis_out_req_o.t.dest   =  '0;
  assign axis_out_req_o.t.last   =  '0;
 

 //packing axi_s_req/rsp for upsizer
 //unpacking axi_sreq/rsp from framing_top
  Lidar_top_level  lidar_top_level 
  (
    .clk_i                  (   clk_i                  ),
    .rstn_i                 (   rstn_i                 ),
    .data_i                 (   axis_in_req_i.t.data   ),          
    .valid_data_i           (   axis_in_req_i.tvalid   ),
    .tlast_data_i           (   axis_in_req_i.t.last   ),
    .ready_DDM_o            (   axis_in_rsp_o.tready   ),
    .data_o                 (   axis_out_req_o.t.data  ),
    .valid_serial_o         (   axis_out_req_o.tvalid  ),
    .ready_upsizer_i        (   axis_out_rsp_i.tready  ),
    .nr_packets             (   nr_packets             ),
    .hwa_length_o           (   hwa_length_o           ),
    .error_rx_o             (   error_rx_o             )
  );

endmodule
