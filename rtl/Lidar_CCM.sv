module Lidar_CCM (
input logic rstn_i, 
input logic clk_i, 
input logic valid1_ACM_i,
input logic valid2_ACM_i,
input logic signed [17:0] sina1_i, cosa1_i, sina2_i, cosa2_i,
input logic [3:0] channel_ID_i,
input logic [15:0] distance_i,
output logic ready_CCM_o,
output logic valid_CCM_o,
output logic signed [17:0] x_o, y_o, z_o
);

/*formula matematica 3d:          Formula di conversione nel nostro caso �:
X=R*cosw*sina                  X=[(R*cosw*100_000*sina*100_000)*normalizer_xy]>>48     normalizer_xy = 28147 ~= ((1/10000000000)*(2**48))
Y=R*cosw*cosa                  Y=[(R*cosw*100_000*cosa*100_000)*normalizer_xy]>>48  
Z=R*sinw                       Z=[(R*sinw*100_000)*normalizer_z]>>>32                  normalizer_z = 42949 ~= ((1/10000000000)*(2**32))

Per normalizzare il valore di sin/cos, per evitare di usare un divisore, uso una costante moltiplicativa dalla quale 
rimane un refuso di 2^48 per x,y e 2^32 per z che correggo con uno shift a destra di 48 e 32 bit che equivale a dividere per 2^44 e 2^26
*/
logic signed [17:0] distance, distance_signed;

localparam logic signed [15:0] normalizer_xy = 16'sd28147;
localparam logic signed [16:0] normalizer_z = 17'sd42949; 

//registri interni per il calcolo delle coordinate dalla formula di ricostruzione 3D
logic signed [35:0] x_trigonometric; //x_trigonometric=(cosw*100_000)*(sina*100_000)   36bit=18bit+18bit
logic signed [35:0] y_trigonometric; //y_trigonometric=(cosw*100_000)*(cosa*100_000)   36bit=18bit+18bit

logic signed [53:0] x_3Dformula; //x_3Dformula=R*x_trigonometric     54bit=18bit+36bit
logic signed [53:0] y_3Dformula; //y_3Dformula=R*y_trigonometric     54bit=18bit+36bit
logic signed [33:0] z_3Dformula; //z_3Dformula=R*(sinw*100_000)      34bit=18bit+16bit

logic signed [54:0] y_corrected;
logic signed [69:0] x_normalized; //x_normalized=(x_3Dformula*normalizer_xy)>>>44   70bit=54bit+16bit
logic signed [70:0] y_normalized; //y_normalized=(y_corrected*normalizer_xy)>>>44   71bit=55bit+16bit
logic signed [50:0] z_normalized; //z_normalized=(z_3Dformula*normalizer_z)>>>26    51bit=34bit+11bit

logic signed [17:0] z_valid, y_valid, x_valid;


//Istanzio due LUT per i valori di {cosw, sinw}, channel_ID 4 bit unsigned sar� l'indice di scorrimento
logic signed [15:0] [15:0] LUT_sinw; // valore massimo per il dimensionamento dei registri � -25881 => 16 bit signed in 2's complement 
logic signed [15:0] [17:0] LUT_cosw;
logic signed [15:0] [37:0] LUT_y_correction;

assign LUT_sinw = {16'sd25881 /*sin(elevation_channel_15)*/, -16'sd1745, 16'sd22495, -16'sd5233, 16'sd19080, -16'sd8715, 16'sd15643, -16'sd12186, 16'sd12186,
 -16'sd15643, 16'sd8715, -16'sd19080, 16'sd5233, -16'sd22495, 16'sd1745, -16'sd25881 /*sin(elevation_channel_0)*/};

assign LUT_cosw = {18'sd96592 /*cos(elevation_channel_15)*/, 18'sd99984, 18'sd97437, 18'sd99862, 18'sd98162, 18'sd99619, 18'sd98768, 
18'sd99254, 18'sd99254, 18'sd98768, 18'sd99619, 18'sd98162, 18'sd99862, 18'sd97437, 18'sd99984, 18'sd96592/*cos(elevation_channel_0)*/ };

assign LUT_y_correction = {-38'sd74_000_000_000, 38'sd9_000_000_000, -38'sd65_000_000_000, 38'sd18_000_000_000, -38'sd55_000_000_000, 38'sd27_000_000_000, 
-38'sd46_000_000_000, 38'sd37_000_000_000, -38'sd37_000_000_000, 38'sd46_000_000_000, -38'sd27_000_000_000, 38'sd55_000_000_000, -38'sd18_000_000_000, 38'sd65_000_000_000, 
-38'sd9_000_000_000, 38'sd74_000_000_000};

//implemento una fsm 
typedef enum logic[1:0] {IDLE, COMPUTE1, COMPUTE2, WAIT} statetype; //cs means current_state, ns means next_state
statetype cs, ns;

always_ff @(posedge clk_i, negedge rstn_i) begin
if(~rstn_i) cs<=IDLE;
else cs<=ns;
end

always_comb begin
ns=cs;
ready_CCM_o=1'd0;
valid_CCM_o=1'd0;
case(cs) 
	IDLE:     begin
		   ready_CCM_o=1'd1;
		   if(valid1_ACM_i) ns=COMPUTE1;
	 end
	COMPUTE1: begin
		   if(channel_ID_i==4'd15 & valid2_ACM_i) ns=COMPUTE2; 
		   else if(channel_ID_i==4'd15 &  valid2_ACM_i==1'd0) ns=WAIT;
	end
	WAIT: begin
		if(valid2_ACM_i) ns=COMPUTE2;
	end
	COMPUTE2: begin
		    if(channel_ID_i==4'd15) begin
		 	ns=IDLE;
			valid_CCM_o=1'd1;
		end
	end
endcase
end


//Il calcolo delle coordinate avviene in parallelo, implemento un always_comb per ogni coordinata
always_comb begin
distance=distance_i<<1;
distance_signed=signed'(distance);
end

//CCORDINATA X
always_comb begin
x_trigonometric=36'd0;
x_3Dformula=54'd0;
x_normalized=70'd0;
x_valid=18'd0;
if (cs==COMPUTE1 || cs==COMPUTE2)  begin 
	if(cs==COMPUTE1) x_trigonometric = signed'(LUT_cosw[channel_ID_i])*sina1_i;
	else if(cs==COMPUTE2) x_trigonometric = signed'(LUT_cosw[channel_ID_i])*sina2_i;
	x_3Dformula  = x_trigonometric * distance_signed;
	x_normalized = (x_3Dformula * normalizer_xy)>>>48;
	x_valid=x_normalized[17:0];
end
end //end comb

//Calcolo della coordinata Y 
always_comb begin
y_trigonometric=36'd0;
y_3Dformula=54'd0;
y_corrected=55'd0;
y_normalized=71'd0;
y_valid=18'd0;
if (cs==COMPUTE1 || cs==COMPUTE2  ) begin
	if(cs==COMPUTE1) y_trigonometric = signed'(LUT_cosw[channel_ID_i])*cosa1_i;
	else if(cs==COMPUTE2) y_trigonometric = signed'(LUT_cosw[channel_ID_i])*cosa2_i;
	y_3Dformula  = y_trigonometric* distance_signed;
	y_corrected = y_3Dformula+signed'(LUT_y_correction[channel_ID_i]);
	y_normalized = (y_corrected * normalizer_xy)>>>48;
	y_valid=y_normalized[17:0];
	end
end //end comb

//Calcolo della coordinata Z
always_comb begin
z_3Dformula=34'd0;
z_normalized=51'd0;
z_valid=18'd0;
if (cs==COMPUTE1 || cs==COMPUTE2) begin
z_3Dformula  = signed'(LUT_sinw[channel_ID_i]) * distance_signed;
z_normalized = (z_3Dformula * normalizer_z)>>>32;
z_valid=z_normalized[17:0];
end // end if
end //end comb


always_ff@(posedge clk_i, negedge rstn_i) begin
if(~rstn_i) begin
	z_o<=18'd0;
	y_o<=18'd0;
	x_o<=18'd0;
	end
else begin
	z_o <=z_valid;
	y_o <=y_valid;
	x_o <=x_valid;
	end
end
endmodule