
module Lidar_CCM (
input logic rstn_i, 
input logic clk_i, 
input logic error_rx_i,
output logic [11:0] hwa_length_o,

//valid trigonometric functions from ACM 
input logic valid_fs1_ACM_i,
input logic valid_fs2_ACM_i,
input logic signed [17:0] sina1_i, cosa1_i, sina2_i, cosa2_i,

output logic valid_fs1_CCM_o, //this signal do not drive any logic
output logic valid_fs2_CCM_o,// assertion of valid_fs2_CCM is used in ACM to go in IDLE 

//valid ID, Distance to be popped from fifo_id, fifo_distance in DDM
input logic valid_channel_DDM_i,
input logic [3:0] id_DDM_i,
input logic [15:0] distance_DDM_i,
output logic ready_CCM_o, //drives fifo_id and fifo_distance pop signals 


output logic valid_datapoint_CCM_o,
output logic signed [15:0] x_o, y_o, z_o,
input logic ready_serial_i
);

/*formula matematica 3d:          Formula di conversione nel nostro caso  :
X=R*cosw*sina                  X=[(R*cosw*100_000*sina*100_000)*normalizer_xy]>>52     normalizer_xy = 450359 ~= ((1/10000000000)*(2**52))
Y=R*cosw*cosa                  Y=[(R*cosw*100_000*cosa*100_000)*normalizer_xy]>>52  
Z=R*sinw                       Z=[(R*sinw*100_000)*normalizer_z]>>>35                  normalizer_z = 343597 ~= ((1/10000000000)*(2**35))

Per normalizzare il valore di sin/cos, per evitare di usare un divisore, uso una costante moltiplicativa dalla quale 
rimane un refuso di 2^48 per x,y e 2^32 per z che correggo con uno shift a destra di 48 e 32 bit che equivale a dividere per 2^44 e 2^26
*/
logic signed [17:0] distance;
logic [11:0] hwa_length_q, hwa_length_d;

localparam logic signed [15:0] NORMALIZER_XY = 16'sd28147;
localparam logic signed [16:0] NORMALIZER_Z = 17'sd42949; 

//registri interni per il calcolo delle coordinate dalla formula di ricostruzione 3D
logic signed [35:0] x_trigonometric; //x_trigonometric=(cosw*100_000)*(sina*100_000)   36bit=18bit+18bit
logic signed [35:0] y_trigonometric; //y_trigonometric=(cosw*100_000)*(cosa*100_000)   36bit=18bit+18bit

logic signed [53:0] x_3Dformula; //x_3Dformula=R*x_trigonometric     54bit=18bit+36bit
logic signed [53:0] y_3Dformula; //y_3Dformula=R*y_trigonometric     54bit=18bit+36bit
logic signed [33:0] z_3Dformula; //z_3Dformula=R*(sinw*100_000)      34bit=18bit+16bit

logic signed [34:0] z_corrected; //z_corrected=z_3Dformula+LUT_Z[channel_ID]    35bit=34bit+1bit carryout

logic signed [70:0] x_normalized; //x_normalized=(x_3Dformula*normalizer_xy)>>>52   75bit=55bit+16bit
logic signed [70:0] y_normalized; //y_normalized=(y_corrected*normalizer_xy)>>>52   75bit=55bit+16bit
logic signed [51:0] z_normalized; //z_normalized=(z_corrected*normalizer_z)>>>35   55bit=35bit+17bit

//Istanzio due LUT per i valori di {cosw, sinw}, channel_ID 4 bit unsigned sar  l'indice di scorrimento
logic signed [15:0] [15:0] LUT_sinw; // valore massimo per il dimensionamento dei registri   -25881 => 16 bit signed in 2's complement 
logic signed [15:0] [17:0] LUT_cosw;
logic signed [15:0] [20:0] LUT_Z;


assign LUT_sinw = {16'sd25881 /*sin(elevation_channel_15)*/, -16'sd1745, 16'sd22495, -16'sd5233, 16'sd19080, -16'sd8715, 16'sd15643, -16'sd12186, 16'sd12186,
 -16'sd15643, 16'sd8715, -16'sd19080, 16'sd5233, -16'sd22495, 16'sd1745, -16'sd25881 /*sin(elevation_channel_0)*/};

assign LUT_cosw = {18'sd96592 /*cos(elevation_channel_15)*/, 18'sd99984, 18'sd97437, 18'sd99862, 18'sd98162, 18'sd99619, 18'sd98768, 
18'sd99254, 18'sd99254, 18'sd98768, 18'sd99619, 18'sd98162, 18'sd99862, 18'sd97437, 18'sd99984, 18'sd96592/*cos(elevation_channel_0)*/ };

assign LUT_Z = {-21'sd740_000, 21'sd90_000, -21'sd650_000, 21'sd180_000, -21'sd550_000, 21'sd270_000, -21'sd460_000, 21'sd370_000,
 -21'sd370_000, 21'sd460_000, -21'sd270_000, 21'sd550_000, -21'sd180_000, 21'sd650_000, -21'sd90_000, 21'sd740_000};

//implemento una fsm 
typedef enum logic[2:0] {IDLE, COMPUTE1, COMPUTE2, WAIT_FOR_ACM} statetype; //cs means current_state, ns means next_state
statetype cs, ns;

always_ff @(posedge clk_i, negedge rstn_i) begin
if(~rstn_i) cs<=IDLE;
else cs<=ns;
end

always_ff @(posedge clk_i, negedge rstn_i) begin
	if (~rstn_i) hwa_length_q<='0;
	else hwa_length_q <= hwa_length_d;
end

always_comb begin
	hwa_length_d=hwa_length_q;
	hwa_length_o=hwa_length_q;
	if(valid_datapoint_CCM_o) begin
		hwa_length_d=hwa_length_q+12'd6;
		if(error_rx_i ) begin
			hwa_length_o=hwa_length_q+12'd6;
			hwa_length_d = '0;
		end
	end
	if(hwa_length_q == 12'd2304) 
		hwa_length_d = '0;
end

//conta i 32 dapa point di un data block
logic[5:0] cnt_dp_d, cnt_dp_q;
logic clear_cnt_dp;

always_ff@(posedge clk_i, negedge rstn_i) begin
if(~rstn_i) cnt_dp_q<=6'd0;
else if (clear_cnt_dp) cnt_dp_q<=1'd0;
else cnt_dp_q<=cnt_dp_d;
end

always_comb begin
cnt_dp_d=cnt_dp_q;
if(cs==COMPUTE1 || cs==COMPUTE2) begin
	if(valid_channel_DDM_i && ready_serial_i) cnt_dp_d=cnt_dp_q+6'd1;
end
end

always_comb begin
ns=cs;
valid_fs1_CCM_o=1'd0;
valid_fs2_CCM_o=1'd0;
ready_CCM_o=1'd1;
clear_cnt_dp = 1'b0;

if(error_rx_i) begin
	ns  = IDLE;
	clear_cnt_dp    = 1'b1;
	valid_fs1_CCM_o = 1'b0;
	valid_fs2_CCM_o = 1'b0;
end

if(cnt_dp_q==6'd16 && cs==COMPUTE1) valid_fs1_CCM_o=1'd1;
if(cnt_dp_q==6'd32 && cs==COMPUTE2) valid_fs2_CCM_o=1'd1;
case(cs) 
	IDLE:     begin
		ready_CCM_o=1'd0;
		   if(valid_fs1_ACM_i) ns=COMPUTE1;
	 end
	COMPUTE1: begin
		if(!ready_serial_i) ready_CCM_o=1'b0;
		else begin
			if(cnt_dp_q==6'd16) begin
				if(valid_fs2_ACM_i==1'd0) ns=WAIT_FOR_ACM;
				else ns=COMPUTE2;
			end
		
		end
	end
	WAIT_FOR_ACM: begin
		valid_fs1_CCM_o=1'd1;
		ready_CCM_o=1'd0;
		if(valid_fs2_ACM_i) ns=COMPUTE2;
	end
	COMPUTE2: begin
		valid_fs1_CCM_o=1'd1;
		if(!ready_serial_i) ready_CCM_o=1'b0;
		else begin
		    if(cnt_dp_q==6'd32) begin
				ns=IDLE;
				valid_fs2_CCM_o=1'd1;
				clear_cnt_dp=1'd1;
			end
		end
	end
endcase
end

//for each valid data in input if serializer is ready (fifo are not full) CCM can pop data 
//as the x,y,z are processed with combinational logic. if a handshake occurs, in the next cycle there will be valid coordinates to be pushed in serializer fifos
logic valid_dp_in;

always_ff@(posedge clk_i, negedge rstn_i) begin 
if(~rstn_i) valid_datapoint_CCM_o<=1'd0;
else valid_datapoint_CCM_o<=valid_dp_in;
end

always_comb begin
valid_dp_in=1'd0;
if(cs==COMPUTE1 && ready_serial_i) begin
	if(cs != ns)
		valid_dp_in = 1'b0;
	else
		valid_dp_in = valid_channel_DDM_i;
end else if(cs==COMPUTE2 && ready_serial_i) begin
		valid_dp_in = valid_channel_DDM_i;
	end
end

//Il calcolo delle coordinate avviene in parallelo, implemento un always_comb per ogni coordinata
assign distance=signed'(distance_DDM_i<<1);

//CCORDINATA X
always_comb begin
x_trigonometric=36'd0;
x_3Dformula=54'd0;
x_normalized=71'd0;
if(valid_channel_DDM_i && ready_serial_i)begin
if (cs==COMPUTE1 || cs==COMPUTE2)  begin 
	if(cs==COMPUTE1) x_trigonometric = signed'(LUT_cosw[id_DDM_i])*sina1_i;
	else if(cs==COMPUTE2) x_trigonometric = signed'(LUT_cosw[id_DDM_i])*sina2_i;
	x_3Dformula  = x_trigonometric * distance;
	x_normalized = (x_3Dformula * NORMALIZER_XY)>>>48;
end
end
end //end comb

//Calcolo della coordinata Y 
always_comb begin
y_trigonometric=36'd0;
y_3Dformula=54'd0;
y_normalized=71'd0;
if (valid_channel_DDM_i && ready_serial_i) begin
if (cs==COMPUTE1 || cs==COMPUTE2  ) begin
	if(cs==COMPUTE1) y_trigonometric = signed'(LUT_cosw[id_DDM_i])*cosa1_i;
	else if(cs==COMPUTE2) y_trigonometric = signed'(LUT_cosw[id_DDM_i])*cosa2_i;
	y_3Dformula  = y_trigonometric* distance;
	y_normalized = (y_3Dformula * NORMALIZER_XY)>>>48;
	end
end //end comb
end

logic signed [50:0] z_normalized_no_offset;

//Calcolo della coordinata Z
always_comb begin
z_3Dformula=34'd0;
z_corrected=35'd0;
z_normalized_no_offset=51'd0;
z_normalized=52'd0;
if(valid_channel_DDM_i && ready_serial_i) begin
if (cs==COMPUTE1 || cs==COMPUTE2) begin
z_3Dformula  = signed'(LUT_sinw[id_DDM_i]) * distance;
z_corrected= signed'(LUT_Z[id_DDM_i])+z_3Dformula;
z_normalized_no_offset=(z_3Dformula*NORMALIZER_Z)>>>32;
z_normalized = (z_corrected * NORMALIZER_Z)>>>32;
end // end if
end //end comb
end

always_ff@(posedge clk_i, negedge rstn_i) begin
if(~rstn_i) begin
	z_o<=16'd0;
	y_o<=16'd0;
	x_o<=16'd0;
	end
else begin
	z_o <=z_normalized_no_offset[15:0];
	y_o <=y_normalized[15:0];
	x_o <=x_normalized[15:0];
	end
end
endmodule
