`timescale 1ns/1ps
/*package load_stim_pkg;
    int stim_fd;
    int num_stim = 0;

    task load_stim(input string stim, output logic [15:0] stimuli[$]);
        int ret;
        logic [15:0] rdata;
        stim_fd = $fopen(stim, "r");

        if (stim_fd == 0)
            $fatal(1, "could not open stimuli file!");

        while (!$feof(stim_fd)) begin
            ret = $fscanf(stim_fd, "%h\n", rdata);
            stimuli.push_back(rdata);
        end

        $fclose(stim_fd);
    endtask
endpackage*/

module TB_Lidar_CCM();

logic clk_i;
logic rstn_i;
logic valid_fs1_ACM_i;
logic valid_fs2_ACM_i;
logic ready_fs1_ACM_i;
logic ready_fs2_ACM_i;
logic [3:0] channel_ID_i;
logic signed [15:0] distance_i;
logic signed [17:0] sina1_i, cosa1_i, sina2_i, cosa2_i;
logic signed [17:0] x_o, y_o, z_o;
logic ready_fs1_CCM_o;
logic ready_fs2_CCM_o;
logic valid_fs1_CCM_o;
logic valid_fs2_CCM_O;

int num_sim=0;
int length_sim=15;
int id=0;

Lidar_CCM DUT (
		.rstn_i(rstn_i),
		.clk_i (clk_i),
		.valid_fs1_ACM_i(valid_fs1_ACM_i),
		.valid_fs2_ACM_i(valid_fs2_ACM_i),
		.ready_fs1_ACM_i(ready_fs1_ACM_i),
		.ready_fs2_ACM_i(ready_fs2_ACM_i),
		.channel_ID_i(channel_ID_i),
		.distance_i (distance_i),
		.cosa1_i (cosa1_i),
		.sina1_i(sina1_i),
		.cosa2_i (cosa2_i),
		.sina2_i(sina2_i),
		.x_o(x_o),
		.y_o(y_o),
		.z_o(z_o),
		.ready_fs1_CCM_o (ready_fs1_CCM_o),
		.ready_fs2_CCM_o (ready_fs2_CCM_o),
		.valid_fs1_CCM_o(valid_fs1_CCM_o),
		.valid_fs2_CCM_o(valid_fs2_CCM_o)
);

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

logic [15:0] [15:0]distance_vector_fs1 = {16'd838, 16'd804, 16'd832, 16'd814, 16'd833, 16'd804, 16'd818, 
16'd809, 16'd816, 16'd817, 16'd803, 16'd813, 16'd809, 16'd834, 16'd813, 16'd827};
logic [15:0] [15:0]distance_vector_fs2 = {16'd841, 16'd809, 16'd830, 16'd812, 16'd831, 16'd806, 16'd822, 
16'd813, 16'd812, 16'd811, 16'd805, 16'd815, 16'd819, 16'd832, 16'd813, 16'd829};

initial begin

wait(rstn_i)

//distanza massimo valore   131070
//simulo un firing sequence
@(posedge clk_i);
valid_fs1_ACM_i=1'd1;
valid_fs2_ACM_i=0;
cosa1_i=18'sd99999;
sina1_i=18'd244;
cosa2_i='0;
sina2_i='0;
while(num_sim<=length_sim) begin
@(posedge clk_i);
channel_ID_i=id;
distance_i=distance_vector_fs1[id];
id++;
num_sim++;
end

@(posedge clk_i);
valid_fs2_ACM_i=1'd1;
cosa2_i=-18'd25648;
sina2_i=-18'd65489;
id=0;
num_sim=0;

while(num_sim<=length_sim) begin
channel_ID_i=id;
distance_i=$urandom_range(65535);
id++;
num_sim++;
@(posedge clk_i);
end

@(posedge clk_i);
@(posedge clk_i);
$stop;
end
endmodule
