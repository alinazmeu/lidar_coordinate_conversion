module Lidar_ACM (
input logic clk_i, 
input logic rstn_i,
input [7:0] data_i
input logic valid_data_i,
output logic [15:0] azimuth_o,
output logic valid_azimuth_o,

//signals of fifo input
);

//internal logic
localparam logic [7:0] FLAG_MSBYTE=8'hff, FLAG_LSBYTE=8'hee;
logic [7:0] flag_msbyte_d, flag_msbyte_q; //??va bene che sia un registro???
logic [7:0] azimuth_msbyte_d, azimuth_msbyte_q;
logic [7:0] distance_msbyte_d, distance_msbyte_q;

//flags for data routing
logic flag_ok;
logic header_ok;

typedef enum logic [2:0] {STATE_IDLE, STATE_HEADER, STATE_FLAG, STATE_AZIMUTH, STATE_DISTANCE} statetype;
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
    end
    STATE_AZIMUTH: begin
      if(valid_azimuth_o)
        ns=STATE_DISTANCE;
    end
  STATE_DISTANCE begin
  
  end

end

//ACQUISIZIONE E CONTROLLO FLAG
always_comb begin 
  flag_msbyte_d=flag_msbyte_q;
  if(cs==STATE_FLAG) begin
    if(valid_d_i && cnt_data_q==6'd0)
      flag_msbyte_d = data_i;
    else if(valid_data_i && cnt_data_q==6'd1)
      if (flag_msbyte_q==FLAG_MSBYTE && data_i==FLAG_LSBYTE) flag_ok=1'b1;
  end
end

//ACQUISIZIONE AZIMUTH
always_comb begin
azimuth_msbyte_d=azimuth_msbyte_q;
  if(valid_data_i && cnt_data_q==6'd2) 
    azimuth_msbyte_d=data_i;
  else if (valid_data_i && cnt_data_q==6'd3) begin
    azimuth_o [15:8] = azimuth_msbyte_q;
    azimuth_o [7:0] = data_i;
    valid_azimuth_o=1'b1;
  end
end

//ACQUISIZIONE DISTANZA

always_ff @(posedge clk_i, negedge rstn_i) begin
  if(~rstn_i) 
    cs<=IDLE;
  else 
    cs<=ns;
end

//COUNTER TO COUNT THE INPUT VALID BYTES
logic [5:0] cnt_data_d, cnt_data_q;

always_comb begin
  cnt_data_d=cnt_data_q;
    if(valid_data_i && cnt_data_q==6'd41 && cs==STATE_HEADER) begin
      cnt_data_d=6'b0; //ho finito di contare il header e resetto il contatore per riutilizzarlo
      header_ok=1'b1;
    end
    else if(valid_data_i) 
      cnt_data_d=cnt_data_q+6'b1;
end


always_ff@(posedge clk_i, negedge rstn_i) begin
  if(~rstn_i) begin
      cnt_data_q<=6'b0;
      flag_msbyte_q<=8'b0;
      azimuth_msbyte_q<=8'b0;
      distance_msbyte_q<=8'b0;
  end
  else begin
    cnt_data_q<=cnt_data_d;
    flag_msbyte_q<=flag_msbyte_d;
    azimuth_msbyte_q<=azimuth_msbyte_d;
    distance_msbyte_q<=distance_msbyte_d;
  end
end 


end module
