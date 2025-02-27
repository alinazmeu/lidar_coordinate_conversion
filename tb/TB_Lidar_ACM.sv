`timescale 1ns/1ps
module TB_Lidar_ACM();

logic clk_i;
logic rstn_i;
logic valid_DDM_i;
logic ready_fs1_CCM_i;
logic ready_fs2_CCM_i;
logic [15:0]azimuth_i; //[0, 36000]
logic valid_fs1_CCM_i;
logic valid_fs2_CCM_i;

logic signed [17:0] cosa1_o, sina1_o, sina2_o, cosa2_o;
logic valid_fs1_ACM_o;
logic valid_fs2_ACM_o;
logic ready_fs1_ACM_o;
logic ready_fs2_ACM_o;



Lidar_ACM DUT (
		.rstn_i(rstn_i),
		.clk_i (clk_i),
		.valid_DDM_i(valid_DDM_i),
		.ready_fs1_CCM_i(ready_fs1_CCM_i),
		.ready_fs2_CCM_i(ready_fs2_CCM_i),
		.azimuth_i(azimuth_i),
		.valid_fs1_CCM_i(valid_fs1_CCM_i),
		.valid_fs2_CCM_i(valid_fs2_CCM_i),
		.cosa1_o (cosa1_o),
		.sina1_o(sina1_o),
		.cosa2_o (cosa2_o),
		.sina2_o(sina2_o),
		.valid_fs1_ACM_o(valid1_fs1_ACM_o),
		.valid_fs2_ACM_o(valid_fs2_ACM_o),
		.ready_fs1_ACM_o (ready_fs1_ACM_o),
		.ready_fs2_ACM_o (ready_fs2_ACM_o)
);

int num_sim=0;
int length_sim=2;

logic [2:0] [15:0]azimuth_vector = {16'd95, 16'd54, 16'd14};
// 16'd354, 16'd394, 16'd434, 16'd474, 16'd514, 16'd554, 16'd594, 16'd634, 16'd674, 16'd714, 16'd754, 16'd794, 16'd834};

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
ready_fs1_CCM_i=1'd1;
ready_fs2_CCM_i=1'd1;
valid_fs1_CCM_i=1'd0;
valid_fs2_CCM_i=1'd0;
wait(rstn_i)
@(posedge clk_i); //arriva valid_DDM_i
repeat(19)@(posedge clk_i); //1clk start+18clk compute1 =>in 19 clock si acquisiscono le distanze dei primi 6 canali + 1 byte del 7 canale
@(posedge clk_i) //valid_fs1_ACM_i=1=> acquisisco le distanze dei primi 7 canali
@(posedge clk_i) //CCM cs is compute1=>ready_fs1_CCM=0  coordinate del primo canale e decodifico il terzo byte del 7 canale
ready_fs1_CCM_i='0;
repeat(6)@(posedge clk_i); //converto i sei canali rimanenti intanto ne ho estratti altri tre completi
repeat(3)@(posedge clk_i); //converto i tre canali precedenti intanto ne estraggo un altro 
@(posedge clk_i); //converto il canale precednete ed estraggo il 1  byte del 12 canale 
@(posedge clk_i); //converto il 12  canale
repeat(3)@(posedge clk_i); //converto il 13 canale 
repeat(9)@(posedge clk_i); //converto gli ultimi tre canali
@(posedge clk_i) //vado in valid1 estraggo il primo byte del primo canale seconod firing
valid_fs1_CCM_i=1'd1;
ready_fs1_CCM_i=1'd1;
@(posedge clk_i); // converto il primo canale 
ready_fs2_CCM_i='0;
repeat(42)@(posedge clk_i); // converto i successuvu 14 canali
repeat(2)@(posedge clk_i); //converto l ultimo canale
@(posedge clk_i); //sono in valid2
ready_fs2_CCM_i=1'd1;
valid_fs2_CCM_i=1'd1;

end


initial begin

wait(rstn_i)
//ad ogni clock si estrae un byte, iniziamo la simulazione dal primo azimuth del primo blocco. 
//Dal secondo blocco in poi si estrae un nuovo azimuth dopo (2B*32+32B)+2B+2B=100B 
while(num_sim<=length_sim) begin
@(posedge clk_i);
valid_DDM_i=1'd1;
azimuth_i=azimuth_vector[num_sim]; 
@(posedge clk_i);
valid_DDM_i=1'd0;
num_sim++;
repeat(99)@(posedge clk_i);
end
$stop;
end
endmodule
