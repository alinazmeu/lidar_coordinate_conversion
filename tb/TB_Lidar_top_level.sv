
`timescale 1ns/1ps

module TB_Lidar_Top_Level();

logic clk_i;
logic rstn_i;
logic [3:0] channel_ID_i;
logic [15:0] distance_i;
logic [15:0]azimuth_i;
logic valid_DDM_i;
logic valid_fs1_CCM_o;
logic valid_fs2_CCM_o;
logic valid_fs1_ACM_o;
logic valid_fs2_ACM_o;
logic signed [17:0] x_o, y_o, z_o;

Lidar_Top_Level DUT(
	.rstn_i(rstn_i),
	.clk_i (clk_i),
	.channel_ID_i(channel_ID_i),
	.distance_i(distance_i),
	.azimuth_i(azimuth_i),
	.valid_DDM_i(valid_DDM_i),
	.valid_fs1_CCM_o(valid_fs1_CCM_o),
	.valid_fs2_CCM_o(valid_fs2_CCM_o),
	.valid_fs1_ACM_o(valid_fs1_ACM_o),
	.valid_fs2_ACM_o(valid_fs2_ACM_o),
	.x_o(x_o),
	.y_o(y_o),
	.z_o(z_o)
	
);


int length_sim=15;
int id=0;
int index=0;

logic [15:0] [15:0]distance_vector_fs1 = {16'd838, 16'd804, 16'd832, 16'd814, 16'd833, 16'd804, 16'd818, 
16'd809, 16'd816, 16'd817, 16'd803, 16'd813, 16'd809, 16'd834, 16'd813, 16'd827};
logic [15:0] [15:0]distance_vector_fs2 = {16'd841, 16'd809, 16'd830, 16'd812, 16'd831, 16'd806, 16'd822, 
16'd813, 16'd812, 16'd811, 16'd805, 16'd815, 16'd819, 16'd832, 16'd813, 16'd829};


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
wait(valid_fs1_ACM_o)
repeat(3) @(posedge clk_i);
$display("azimuth%d", azimuth_i);
for(int i=0; i<=15; i++) begin
$display("ID:%d, Distance:%d, x:%d, y:%d, z=%d",i, distance_vector_fs1[i], x_o,y_o,z_o);
@(posedge clk_i);
end
wait(valid_fs2_ACM_o)
repeat(2) @(posedge clk_i);
for(int i=0; i<=15; i++) begin
$display("ID:%d, Distance:%d, x:%d, y:%d, z=%d",i, distance_vector_fs2[i], x_o,y_o,z_o);
@(posedge clk_i);
end
end


initial begin

wait(rstn_i)
@(posedge clk_i);
valid_DDM_i=1'd1;
azimuth_i=16'd14; //if ready ccm 

@(posedge clk_i);
valid_DDM_i='0;


wait(valid_fs1_ACM_o)
@(posedge clk_i); //vado in compute1 ed inizio la conversione 
while(id<=length_sim) begin
channel_ID_i=id;
distance_i=distance_vector_fs1[id];
id++;
@(posedge clk_i);
end
id=0; 

wait(valid_fs2_ACM_o)

@(posedge clk_i); // se ero in wait torno in compute2
while(id<=length_sim) begin
channel_ID_i=id;
distance_i=distance_vector_fs2[id];
@(posedge clk_i);
id++;
end
repeat(5)@(posedge clk_i);


$stop;
end
endmodule
