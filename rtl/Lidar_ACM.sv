module Lidar_ACM (
input logic clk_i, 
input logic rstn_i,
input logic valid_DDM_i,
input logic ready_CCM_i,
input logic valid_CCM_i,
input logic [15:0]azimuth_i, 
output logic signed [17:0] cosa1_o, sina1_o, sina2_o, cosa2_o,
output logic valid1_ACM_o,
output logic valid2_ACM_o,
output logic ready_ACM_o
);

typedef enum logic[2:0] {IDLE, START, COMPUTE1, RESULT1, COMPUTE2, RESULT2, WAIT_FOR_CCM} statetype; //cs means current_state, ns means next_state
statetype cs, ns;

always_ff @(posedge clk_i, negedge rstn_i) begin
if(~rstn_i) cs<=IDLE;
else cs<=ns;
end

//RISORSE PER AZIMUTH PRE-PROCESSING, CALCOLO E PRE-PROCESSING DRIFT AZIMUTH 
logic [15:0] azimuth_d, azimuth_q; //PER CAMPIONARE L'AZIMUTH IN INGRESSO
logic [15:0] drift_azimuth_d, drift_azimuth_q; //AZIMUTH SECONDO FIRING OTTENUTO COME BASE+OFFSET
localparam logic [4:0] DRIFT_OFFSET=5'd20; //RISOLUZIONE ANGOLARE, DIPENDE DA RPM, ASSUNZIONE RPM=600

logic [15:0] angle_translated; //ANGOLO TRASLATO AL PRIMO QUADRANTE
logic signed [14:0] angle_cordic; //ANGOLO DA ADEGUARE ALLA DINAMICA DEL CORDIC 

logic [1:0] quadrante_fs1_d, quadrante_fs1_q; //QUADRANTE INIZIALE AZIMUTH, PER LE COORDINATE DEL PRIMO FIRING 
logic [1:0] quadrante_fs2_d, quadrante_fs2_q; //-- DRIFT AZIMUTH, PER LE COORDINATE DEL SECONDO FIRING
localparam logic [15:0] QI=16'd9000;
localparam logic [15:0] QII=16'd18000;
localparam logic [15:0] QIII=16'd27000;
localparam logic [15:0] QIV=16'd36000;

 
//RISORSE CORDIC 
localparam logic signed[4:0] MULTIPLIER=5'sd10;
localparam logic signed [16:0] X_GAIN=17'sd60725;

logic signed [19:0] z_d, z_q; //Z0=angle_cordic*MULTIPLIER=> 20bit signed � il max valore, poi converge a zero
logic signed [18:0] x_d, x_q, y_d, y_q, y_shifted, x_shifted;
logic signed [17:0] cosa1_q, cosa1_d, sina1_q, sina1_d, cosa2_d, cosa2_q, sina2_d, sina2_q;

logic [4:0] shifter; //MAX=15, INDICE LUT E SHIFTER
logic [4:0] iter_d, iter_q; //NR ITERAZIONI ALGORITMO CORDIC FINO A 17 POI TORNO A ZERO



always_comb begin
iter_d=iter_q;
if(cs==COMPUTE1 || cs==COMPUTE2 ) iter_d=iter_q+5'd1;
end

logic signed [15:0] [19:0] LUT_rotation_angle; 
assign LUT_rotation_angle = {20'sd1/*LUT[15]*/, 20'sd3, 20'sd6, 20'sd13, 20'sd27, 20'sd55, 20'sd111, 20'sd223, 
20'sd447, 20'sd895, 20'sd1789, 20'sd3576, 20'sd7125, 20'sd14036, 20'sd26565, 20'sd45000 /*LUT[0]*/}; 

//FSM
always_comb begin
ns=cs;
valid1_ACM_o=1'd0;
valid2_ACM_o=1'd0;
ready_ACM_o=1'd0; 

case(cs) 
	IDLE: begin 
		ready_ACM_o=1'd1; //IN ATTESA DELL'AZIMUTH DA DDM 
		if(valid_DDM_i) ns=START; //NELLO STESSO PERIODO TROVO IL QUADRANTE DELL'AZIMUTH E CALCOLO IL DRIFT_AZIMUTH
	end
	START: begin
		ns=COMPUTE1;
	end
	COMPUTE1: begin
		//Si rimane nello stato per 17 cicli di clock, da iter_q=0 in cui inizializzo i regsitri a iter_q=16 in cui si arriva alla convergenza di x=cos, y=sin
		if (iter_q==5'd16) ns=RESULT1; 
	end
	RESULT1: begin //Impiega un periodo, si effettua la traslazione del risultato Cordic al quadrante di appartenenza, si fa il pre-processing  sul drift_azimuth
		valid1_ACM_o=1'd1; //handshake con CCM che inizia la conversione del primo firing con la coppia {sina1, cosa1}
		ns=COMPUTE2;
	end
	COMPUTE2: begin // Si rimane per 17 cicli come in compute1, cambia solo il contenuto con cui si inizializza il registro accumulatore z
		valid1_ACM_o=1'd1;
		if(iter_q==5'd16) ns=RESULT2; 
	end
	RESULT2: begin
		valid2_ACM_o=1'd1; //handshake con CCM per notificare che la coppia {sina2, cosa2} pu� essere usata nel calcolo delle coordinate del secondo firing
		valid1_ACM_o=1'd1; 
		ns=WAIT_FOR_CCM; //handshake da CCM, quando CCM ha calcolato l'ultimo data point del data_block lo notifica ed entrambe le fsm tornano in IDLE
	end
	WAIT_FOR_CCM: begin
		valid2_ACM_o=1'd1;
		valid1_ACM_o=1'd1;
		if(valid_CCM_i) ns=IDLE;
	end
endcase
end

// QUADRANTE AZIMUTH
always_comb begin 
quadrante_fs1_d=quadrante_fs1_q;
azimuth_d=azimuth_q;

if(valid_DDM_i & cs==IDLE) azimuth_d=azimuth_i; //VA BENE COS�? O LO DEVO METTERE COME SEGNALE DI ENABLE SINCRONO DEL FF ?
else if(cs==START) begin
	if(azimuth_q>QIII) quadrante_fs1_d=2'd3;
	else if(azimuth_q>QII) quadrante_fs1_d=2'd2;
	else if(azimuth_q>QI) quadrante_fs1_d=2'd1;
	else quadrante_fs1_d=2'd0;
	end
end

//CALCOLO DRIFT AZIMUTH E QUADRANTE
always_comb begin
quadrante_fs2_d=quadrante_fs2_q;
drift_azimuth_d=drift_azimuth_q;

if(cs==START) drift_azimuth_d=azimuth_q+DRIFT_OFFSET;
else if (cs==COMPUTE1 && iter_q==5'd0) begin
	if(drift_azimuth_q>QIV)
	drift_azimuth_d=drift_azimuth_q-QIV; //SE SUPERO 360� FACCIO UN GIRO COMPLETO SULLA CIRCONFERENZA 
	end
else if (cs==RESULT1) begin //inidividuo il quadranate
	if(drift_azimuth_q>QIII) quadrante_fs2_d=2'd3;
	else if(drift_azimuth_q>QII) quadrante_fs2_d=2'd2;
	else if(drift_azimuth_q>QI) quadrante_fs2_d=2'd1;
	else quadrante_fs2_d=2'd0;
	end
end

// modello cordic con 17 iterazioni
always_comb begin
shifter=4'd0;
angle_translated=16'd0;
angle_cordic=15'sd0;
y_shifted=19'sd0;
x_shifted=19'sd0;
z_d=z_q;
x_d=x_q;
y_d=y_q;

if(cs==COMPUTE1 && iter_q==5'd0) begin //INIZIALIZZO I REGISTRI DEL FIRING1
	x_d=X_GAIN;
	y_d=19'sd0;
	case(quadrante_fs1_q) 
		2'd0: begin //CASO QUADRANTE I [0, 9000]
			angle_translated=azimuth_q;
			end
		2'd1: begin //CASO QUADRANTE II [9001, 18000]
			angle_translated=QII-azimuth_q;
			end
		2'd2: begin//QUADRANTE III [18001, 27000]
			angle_translated=QIII-azimuth_q;
			end
		2'd3: begin //QUADRANTE IV [27001, 36000]
			angle_translated=QIV-azimuth_q;
			end
		endcase
	angle_cordic=angle_translated[14:0];
	z_d=angle_cordic*MULTIPLIER;
	end
else if(cs==COMPUTE2 && iter_q==5'd0) begin //INIZIALIZZO REGISTRI DEL FIRING 2
	x_d=X_GAIN;
	y_d=19'sd0;
	case(quadrante_fs2_q) 
		2'd0: begin
			angle_translated=drift_azimuth_q;
			end
		2'd1: begin
			angle_translated=QII-drift_azimuth_q;
			end
		2'd2: begin
			angle_translated=QIII-drift_azimuth_q;
			end
		2'd3: begin
			angle_translated=QIV-drift_azimuth_q;
			end
		endcase
	angle_cordic=angle_translated[14:0];
	z_d=angle_cordic*MULTIPLIER;
end
else if (cs==COMPUTE1 || cs==COMPUTE2) begin
	shifter=iter_q-5'd1; 
	y_shifted=y_q>>>shifter;
	x_shifted=x_q>>>shifter;
	if (z_q[19]==17'd1) begin
		x_d=x_q+y_shifted;
		y_d=y_q-x_shifted;
		z_d=z_q+LUT_rotation_angle[shifter];
	end else begin
		x_d=x_q-y_shifted;
		y_d=y_q+x_shifted;
		z_d=z_q-LUT_rotation_angle[shifter];
	end 
end
end

//per ri-trsalare il risultato del cordic al quadrante di partenza 
always_comb begin
cosa1_d=cosa1_q;
sina1_d=sina1_q;
cosa2_d=cosa2_q;
sina2_d=sina2_q;

if(cs==RESULT1) begin
	case(quadrante_fs1_q) 
		2'd0:  begin
			cosa1_d=x_q[17:0];
			sina1_d=y_q[17:0];
			end
		2'd1: begin
			cosa1_d=-x_q[17:0];
			sina1_d=y_q[17:0];
			end
		2'd2: begin
			cosa1_d=-y_q[17:0];
			sina1_d=-x_q[17:0];
			end
		2'd3:  begin
			cosa1_d=x_q[17:0];
			sina1_d=-y_q[17:0];
			end
		endcase
end
else if(cs==RESULT2) begin
	case(quadrante_fs2_q) //traslo il risultato del cordic 
		2'd0:  begin
			cosa2_d=x_q[17:0];
			sina2_d=y_q[17:0];
			end
		2'd1: begin
			cosa2_d=-x_q[17:0];
			sina2_d=y_q[17:0];
			end
		2'd2: begin
			cosa2_d=-y_q[17:0];
			sina2_d=-x_q[17:0];
			end
		2'd3:  begin
			cosa2_d=x_q[17:0];
			sina2_d=-y_q[17:0];
			end
		endcase
	end
end

always_ff@(posedge clk_i, negedge rstn_i) begin
if(~rstn_i) begin
 cosa1_q<='0;
 sina1_q<='0;
 cosa2_q<='0;
 sina2_q<='0;
 end
else begin
cosa1_q<=cosa1_d; //cosa1_q<=
sina1_q<=sina1_d;
cosa2_q<=cosa2_d;
sina2_q<=sina2_d;
end
end

always_ff@(posedge clk_i, negedge rstn_i) begin
	if(~rstn_i) iter_q<=5'd0;
	else if (cs==RESULT1 || cs==RESULT2) iter_q<=5'd0;
	else iter_q<=iter_d;
end


always_ff@(posedge clk_i, negedge rstn_i) begin
if(~rstn_i) begin
	z_q<=20'sd0;
	x_q<=19'sd0;
	y_q<=19'sd0;
	quadrante_fs1_q<=2'd0;
	quadrante_fs2_q<=2'd0;
	azimuth_q<=16'd0;
	drift_azimuth_q<=16'd0;
  end
else begin
	z_q<=z_d;
	x_q<=x_d;
	y_q<=y_d;
	quadrante_fs1_q<=quadrante_fs1_d;
	quadrante_fs2_q<=quadrante_fs2_d;
	drift_azimuth_q<=drift_azimuth_d;
	azimuth_q<=azimuth_d;
  end 
end


always_comb begin
cosa1_o=cosa1_q; //VA BENE?? L'USCITA SI HA NEL CICLO DI RESULT*
sina1_o=sina1_q;
cosa2_o=cosa2_q;
sina2_o=sina2_q;
end

endmodule