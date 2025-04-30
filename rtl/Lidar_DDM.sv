module Lidar_DDM
  (
    input logic clk_i, 
    input logic rstn_i,

    //handshake with CCM e ACM
    input logic ready_CCM_i, //to be conncet to pop_i pin of fifo_distance and fifo_id
    input logic ready_fs1_ACM_i, //to be connect to azimuth forward block
    
  //data from ethernet phy to be pushed into fifo_in
    input logic [7:0] data_i, //to be connect to data_i pin of fifo_in
    input logic valid_data_i, //to be connect to push_i pin of fifo_in

    //2B azimuth from packet to be send to ACM module 
    output logic [15:0] azimuth_DDM_o,
    output logic valid_azimuth_DDM_o, //to notify ACM that a new azimuth from packet has been decoded 

    //2B distance from fifo_distance and 4 bit id from fifo_id to be popped by CCM
    output logic [7:0] distance_o, 
    output logic [3:0] id_o,
    output logic valid_datapoint_DDM_o, //function of fifo_(distance, id) not empty signal

    //fifo signals
    logic testmode_i 
    
  );


  logic [5:0] cnt_data_n, cnt_data_q; //to count 2B flag, 2B azimuth, 32*(2B distance+1B reflectivity) data points
  logic [3:0] cnt_id_n, cnt_id_q; //to count 16 data points in firing and to generate an id for each data point in firing 
  logic [3:0] cnt_block_n, cnt_block_q; // to count 12 data block in a packet

  localparam logic [7:0] FLAG_MSBYTE=8'hFF, FLAG_LSBYTE=8'hEE;

  logic [7:0] data_decoder;
  logic [7:0] reg_msbyte_n, reg_msbyte_q, reg_lsbyte_n, reg_lsbyte_q;

  //flags for data routing
logic flag_ok; //it does not drive any internal logic, it will be used in tb to check if flag value is the expected one 
logic firing1_ok;
logic firing2_ok;
logic ready_decoder; //to pop data from fifo_in, by default is high. It might be pull down if fifo_distance or fifo_id are full
logic valid_datapoint;

assign valid_datapoint_DDM_o=valid_datapoint;
  
  //FIFO_IN 
  logic flush_fifo_in, full_fifo_in, empty_fifo_in;
  logic [4:0]  usage_fifo_in; 
  
  fifo_v3 #(
    .DATA_WIDTH(8),
    .DEPTH(16)
  )
  fifo_in(
    .clk_i(clk_i),
    .rst_ni(rstn_i),
    .flush_i(flush_fifo_in),
    .testmode_i(testmode_i),
    .data_i(data_i),
    .push_i(valid_data_i),
    .pop_i(ready_decoder),
    .full_o(full_fifo_in),
    .empty_o(empty_fifo_in),
    .usage_o(usage_fifo_in),
    .data_o(data_decoder)
  );

  //FIFO_DISTANCE
  logic flush_fifo_distance, full_fifo_distance, empty_fifo_distance;
  logic [4:0]  usage_fifo_distance;

  fifo_v3 #(
    .DATA_WIDTH(8),
    .DEPTH(16)
  )
  fifo_distance(
    .clk_i(clk_i),
    .rst_ni(rstn_i),
    .flush_i(flush_fifo_distance), //controlled by internal logic, zero default
    .testmode_i(testmode_i),
    .data_i({reg_msbyte_q, reg_lsbyte_q}),
    .push_i(valid_datapoint),
    .pop_i(ready_CCM_i), 
    .full_o(full_fifo_distance),
    .empty_o(empty_fifo_distance),
    .usage_o(usage_fifo_distance),
    .data_o(distance_o)
  );

  //FIFO_ID
  logic flush_fifo_id, full_fifo_id, empty_fifo_id;
  logic [4:0]  usage_fifo_id;
  
  fifo_v3 #(
    .DATA_WIDTH(4),
    .DEPTH(16)
  )
  fifo_id(
    .clk_i(clk_i),
    .rst_ni(rstn_i),
    .flush_i(flush_fifo_id),
    .testmode_i(testmode_i),
    .data_i(cnt_id_q),
    .push_i(valid_datapoint),
    .pop_i(ready_CCM_i), 
    .full_o(full_fifo_id),
    .empty_o(empty_fifo_id),
    .usage_o(usage_fifo_id),
    .data_o(id_o)
  );


  typedef enum logic [2:0] {STATE_IDLE, STATE_FLAG, STATE_AZIMUTH, STATE_FIRING1, STATE_FIRING2} statetype;
statetype cs, ns;

  always_comb begin
    ready_decoder=1'b1; 

    flush_fifo_in=1'd0;
    flush_fifo_distance=1'd0;
    flush_fifo_id=1'd0;
    
    if(full_fifo_distance || full_fifo_id) ready_decoder=1'b0; 
    
  end
  
//FSM
always_comb begin
  ns=cs;
  case(cs)
    STATE_IDLE: begin
      if(~empty_fifo_in) begin
        ns=STATE_FLAG;
      end
    end
    STATE_FLAG: begin
      if (~empty_fifo_in && cnt_data_q==6'd1)
        ns=STATE_AZIMUTH;
    end
    STATE_AZIMUTH: begin
      if(valid_azimuth_DDM_o)
        ns=STATE_FIRING1;
    end
    STATE_FIRING1 begin
      if(firing1_ok) 
        ns=STATE_FIRING2;
    end
  STATE_FIRING2: begin
    if(firing2_ok && cnt_block_q<=4'd11) 
      ns=STATE_FLAG;
    else if(firing2_ok && cnt_block_q==4'd11)
      nd=STATE_IDLE;
  end
end

    //FLAG VERIFICATION  (EACH DATA BLOCK STARTS WITH 0xFFEE FLAG)
always_comb begin 
  reg_msbyte_n=reg_msbyte_q;
  flag_ok=1'b0;
    if(cs==STATE_FLAG) begin
      if(~empty_fifo_in && cnt_data_q==6'd0)
        reg_msbyte_n = data_decoder;
      else if(~empty_fifo_in && cnt_data_q==6'd1) begin
        if (reg_msbyte_q==FLAG_MSBYTE && data_decoder==FLAG_LSBYTE) 
          flag_ok=1'b1;
      end
  end
end

//AZIMUTH 2 BYTES DATA to ACM
always_comb begin
  reg_msbyte_n=reg_msbyte_q;
  valid_azimuth_DDM_o=1'b0;
  if(cs==STATE_AZIMUTH) begin
    if(~empty_fifo_in && cnt_data_q==6'd2) 
      reg_msbyte_n=data_decoder;
    else if (~empty_fifo_in && cnt_data_q==6'd3 && ) begin
      azimuth_DDM_o = {reg_msbyte_q, data_decoder};
      valid_azimuth_DDM_o=1'b1;
    end
  end
end

  //DISTANCE 2B to fifo_distance and ID 4 bit to fifo_id
  always_comb begin
    cnt_id_n=cnt_id_q;
    reg_msbyte_n=reg_msbyte_q;
    reg_lsbyte_n=reg_lsbyte_q;
    valid_datapoint=1'b0;
    if(cs==STATE_FIRING1 || cs==STATE_FIRING2) begin
      if (~empty_fifo_in && cnt_data_q==6'd0)
        reg_msbyte_n=data_decoder;
      else if(~empty_fifo_in && cnt_data_q==6'd1) begin
        cnt_id_n=cnt_id_q+4'd1;
        reg_lsbyte_n=data_decoder;
      end
      else if(~empty_fifo_in && cnt_data_q==6'd2)
        valid_datapoint=1'b1; 
    end
  end


    //COUNTER TO COUNT THE INPUT VALID BYTES {header 42 B, flag 2B + azimuth 2B, 32*[distance 2 B + reflectivity 1B]}
always_comb begin
  cnt_data_n=cnt_data_q;
  if(valid_azimuth_DDM_o)
      cnt_data_n=6'd0;
  else if (valid_datapoint) begin //TO COUNTS 2 BYTES OF DISTANCE AND 1 BYTE OF REFLECTIVITY FOR EACH CHANNEL IN DATA BLOCK
    cnt_data_n=6'd0;
  end
    else if(~empty_fifo_in) 
      cnt_data_n=cnt_data_q+6'd1;
end

  always_comb begin
    firing1_ok=1'b0;
    firing2_ok=1'b0;
    cnt_block_n=cnt_block_q;
    if(cs==STATE_FIRING1 && ~empty_fifo_in && cnt_id_q==4'd15) 
     firing1_ok=1'b1; 
    else if(cs==STATE_FIRING2 && ~empty_fifo_in && cnt_id_q==4'd15) begin
      firing2_ok=1'b1;
      cnt_block_n=cnt_block_q+4'd1;
    end
  end
    

always_ff @(posedge clk_i, negedge rstn_i) begin
  if(~rstn_i) 
    cs<=IDLE;
  else 
    cs<=ns;
end
    
always_ff@(posedge clk_i, negedge rstn_i) begin
  if(~rstn_i) begin
      cnt_data_q<=6'b0;
      cnt_id_q<=4'b0;
      cnt_block_q<=4'd0;
    reg_msbyte_q<=8'd0;
    reg_lsbyte_q<=8'd0;
  end
  else begin
    cnt_data_q<=cnt_data_n;
    cnt_id_q<=cnt_id_n;
    cnt_block_q<=cnt_block_n;
    reg_msbyte_q<=reg_msbyte_n;
    reg_lsbyte_q>=reg_lsbyte_n;
  end
end 


end module
