module Lidar_DDM
  (
    input logic clk_i, 
    input logic rstn_i,
  //data from the upstream fifo
    input logic [7:0] data_i,
    input logic valid_data_i,

  //azimuth forwarded to ACM module for each of 12's data block in data packet
    output logic [15:0] azimuth_ACM_i,
    output logic valid_azimuth_ACM_i,

//signals of fifo input?????
  );


  logic [5:0] cnt_data_n, cnt_data_q;
  logic [3:0] cnt_id_n, cnt_id_q; //ID (IDENTIFIER) IN RANGE [0, 15] FOR EACH 2 BYTES DISTANCE IN A FIRING
  logic [3:0] cnt_block_n, cnt_block_q;

  localparam logic [7:0] FLAG_MSBYTE=8'hFF, FLAG_LSBYTE=8'hEE;
  logic [7:0] flag_msbyte_n, flag_msbyte_q; 

  logic [7:0] azimuth_msbyte_n, azimuth_msbyte_q;
  
  logic [7:0] distance_msbyte_n, distance_msbyte_q, distance_lsb_n, distance_lsb_q;
  logic valid_channel_push_n, valid_channel_push_q;

//flags for data routing
logic flag_ok;
logic header_ok;
logic firing1_ok;
logic firing2_ok;

  typedef enum logic [2:0] {STATE_IDLE, STATE_HEADER, STATE_FLAG, STATE_AZIMUTH, STATE_FIRING1, STATE_FIRING2} statetype;
statetype cs, ns;

//FSM
always_comb begin
  ns=cs;
  case(cs)
    STATE_IDLE: begin
      if(valid_data_i) begin
        ns=STATE_HEADER;
      end
    end
    STATE_HEADER: begin
      if(header_ok)
        ns=STATE_FLAG
    end
    STATE_FLAG: begin
      if (flag_ok)
        ns=STATE_AZIMUTH;
    end
    STATE_AZIMUTH: begin
      if(valid_azimuth_o)
        ns=STATE_FIRING1;
    end
    STATE_FIRING1 begin
      if(firing1_ok) 
        ns=STATE_FIRING2;
    end
  STATE_FIRING2: begin
    if(firing2_ok && cnt_block_q==4'd11) 
      ns=IDLE;
  end
end

    //FLAG VERIFICATION  (EACH DATA BLOCK STARTS WITH 0xFFEE FLAG)
always_comb begin 
  flag_msbyte_n=flag_msbyte_q;
  flag_ok=1'b0;
    if(cs==STATE_FLAG) begin
      if(valid_data_i && cnt_data_q==6'd0)
        flag_msbyte_n = data_i;
      else if(valid_data_i && cnt_data_q==6'd1) begin
        if (flag_msbyte_q==FLAG_MSBYTE && data_i==FLAG_LSBYTE) 
          flag_ok=1'b1;
      end
  end
end

//AZIMUTH 2 BYTES DATA 
always_comb begin
  azimuth_msbyte_n=azimuth_msbyte_q;
  valid_azimuth_ACM_i=1'b0;
  if(cs==STATE_AZIMUTH) begin
    if(valid_data_i && cnt_data_q==6'd2) 
      azimuth_msbyte_n=data_i;
    else if (valid_data_i && cnt_data_q==6'd3) begin
      azimuth_ACM_i = {azimuth_msbyte_q, data_i}
      valid_azimuth_ACM_i=1'b1;
    end
  end
end




    //COUNTER TO COUNT THE INPUT VALID BYTES {header 42 B, flag 2B + azimuth 2B, 32*[distance 2 B + reflectivity 1B]}
always_comb begin
  cnt_data_n=cnt_data_q;
  header_ok=1'b0;
  if( cs==STATE_HEADER && valid_data_i && cnt_data_q==6'd41) begin
      cnt_data_n=6'd0; 
      header_ok=1'b1; //SLAG SINGNAL TO MOVE FROM STATE HEADER TO STATE FLAG
    end
    else if(cs==STATE_AZIMUTH && valid_azimuth_ACM_i)
      cnt_data_n=6'd0;
  else if (cs==STATE_FIRING1 && valid_data_i && cnt_data_q==6'd2) begin //TO COUNTS 2 BYTES OF DISTANCE AND 1 BYTE OF REFLECTIVITY FOR EACH CHANNEL IN DATA BLOCK
    cnt_data_n=6'd0;
  end
    else if(valid_data_i) 
      cnt_data_n=cnt_data_q+6'd1;
end

    //DISTANCE 2 BYTES AND ID 4 BIT FOR EACH LASER IN A FIRING SEQUENCE
  always_comb begin
    cnt_id_n=cnt_id_q;
    distance_msb_n=distance_msb_q;
    distance_lsb_n=distance_lsb_q;
    valid_channel_push_n=valid_channel_push_q;
    if(cs==STATE_FIRING1 || cs==STATE_FIRING2) begin
      if (valid_data_i && cnt_data_q==6'd0)
        distance_msb_n=data_i;
      else if(valid_data_i && cnt_data_q==6'd1) begin
        cnt_id_n=cnt_id_q+4'd1;
        distance_lsb_n=data_i;
        valid_channel_push_n=1'b1;
      end
    end
  end

  always_comb begin
    firing1_ok=1'b0;
    firing2_ok=1'b0;
    cnt_block_n=cnt_block_q;
    if(cs==STATE_FIRING1 && valid_data_i && cnt_id_q==4'd15) 
     firing1_ok=1'b1; 
    else if(cs==STATE_FIRING1 && valid_data_i && cnt_id_q==4'd15) begin
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
      flag_msbyte_q<=8'b0;
      azimuth_msbyte_q<=8'b0;
      distance_msbyte_q<=8'b0;
      valid_distance_push_q<=1'b0;
  end
  else begin
    cnt_data_q<=cnt_data_n;
    cnt_id_q<=cnt_id_n;
    cnt_block_q<=cnt_block_n;
    flag_msbyte_q<=flag_msbyte_n;
    azimuth_msbyte_q<=azimuth_msbyte_n;
    distance_msbyte_q<=distance_msbyte_n;
    valid_distance_push_q<=valid_distance_push_n;
  end
end 


end module
