
module Lidar_top_level (

	input  logic 							rstn_i			, 
	input  logic 							clk_i			, 
    //AXIS input 
	input  logic		    [7:0] 			data_i			,
	input  logic							valid_data_i	,
	input  logic							tlast_data_i    ,
	output logic 							ready_DDM_o		,
    //AXIS output
	input  logic 							ready_upsizer_i ,
	output logic 							valid_serial_o  ,
	output logic signed		[15:0] 			data_o			,
	output logic			[7:0] 			nr_packets
);

    
	//data and handshake between lidar_ddm and lidar_acm
	logic		 [15:0] azimuth_DDM;
	logic 				valid_azimuth_DDM, ready_ACM;
	//data and control flags between lidar_acm and lidar_ccm
	logic signed [17:0] cosa1, sina1, cosa2, sina2;
	logic 				valid_fs1_CCM, valid_fs2_CCM, valid_fs1_ACM, valid_fs2_ACM;
	//data and handshake between lidar_ccm and coordinate_serializer
	logic 		 [15:0]	x, y, z;
	logic 				valid_datapoint_CCM, ready_serial;
	//data and handshake between lidar_ddm and lidar_ccm
	logic 		[15:0]  distance_DDM;
	logic 		[3:0]   id_DDM;
	logic 				valid_channel_DDM;
	logic				ready_CCM;
	//safety logic 
	logic 				error_rx;

	Lidar_DDM lidar_ddm (
		.rstn_i					( rstn_i			),
		.clk_i					( clk_i				),
		.data_i					( data_i			),
		.valid_data_i			( valid_data_i		),
		.tlast_data_i			( tlast_data_i		),
		.ready_DDM_o			( ready_DDM_o		),
		.error_rx_o				( error_rx			),
		.azimuth_DDM_o			( azimuth_DDM		),
		.valid_azimuth_DDM_o	( valid_azimuth_DDM ),
		.ready_ACM_i			( ready_ACM			),
		.distance_DDM_o			( distance_DDM		),
		.id_DDM_o				( id_DDM			),
		.valid_channel_DDM_o	( valid_channel_DDM ),
		.ready_CCM_i			( ready_CCM			),
		.nr_packets				( nr_packets		)
	);

	Lidar_ACM lidar_acm (
		.rstn_i					( rstn_i			),
		.clk_i 					( clk_i				),
		.error_rx_i 			( error_rx			),
		.valid_azimuth_DDM_i	( valid_azimuth_DDM	),
		.azimuth_DDM_i			( azimuth_DDM		),
		.ready_ACM_o			( ready_ACM			),
		.cosa1_o 				( cosa1				),
		.sina1_o				( sina1				),
		.cosa2_o 				( cosa2				),
		.sina2_o				( sina2				),
		.valid_fs2_CCM_i		( valid_fs2_CCM		),
		.valid_fs1_ACM_o		( valid_fs1_ACM		),
		.valid_fs2_ACM_o		( valid_fs2_ACM		)

	);

	Lidar_CCM lidar_ccm (
		.clk_i 					( clk_i				  ),
		.rstn_i					( rstn_i			  ),
		.error_rx_i 			( error_rx			  ),
		.valid_fs1_ACM_i		( valid_fs1_ACM		  ),
		.valid_fs2_ACM_i		( valid_fs2_ACM		  ),
		.cosa1_i 				( cosa1				  ),
		.sina1_i				( sina1				  ),
		.cosa2_i 				( cosa2				  ),
		.sina2_i				( sina2				  ),
		.valid_fs1_CCM_o 		( valid_fs1_CCM		  ),
		.valid_fs2_CCM_o 		( valid_fs2_CCM		  ),
		.id_DDM_i				( id_DDM			  ),
		.distance_DDM_i 		( distance_DDM		  ),
		.valid_channel_DDM_i	( valid_channel_DDM	  ),
		.ready_CCM_o			( ready_CCM			  ),
		.ready_serial_i			( ready_serial		  ),
		.x_o					( x					  ),
		.y_o					( y					  ),
		.z_o					( z					  ),
		.valid_datapoint_CCM_o	( valid_datapoint_CCM )
	);

	coordinate_serializer parallel_to_serial (
		.clk_i					( clk_i				  ), 
		.rstn_i					( rstn_i			  ),
		.x_i					( x					  ),
		.y_i					( y					  ),
		.z_i					( z					  ),
		.valid_datapoint_CCM_i	( valid_datapoint_CCM ),
		.ready_serial_o			( ready_serial		  ),
		.ready_upsizer_i		( ready_upsizer_i	  ),
		.data_o					( data_o			  ),
		.valid_serial_o			( valid_serial_o	  )
	);

endmodule
