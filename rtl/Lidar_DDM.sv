module Lidar_DDM
  (
    input logic clk_i, 
    input logic rstn_i,

    ///backpressure and handshake
    input logic ready_CCM_i, 
    input logic ready_ACM_i, 

  //data from ethernet phy to be pushed into fifo_in
    input logic [7:0] data_i, //to be connect to data_i pin of fifo_in
    input logic valid_data_i, //to be connect to push_i pin of fifo_in

    //2B azimuth from packet to be send to ACM module 
    output logic [15:0] azimuth_DDM_o,
    output logic valid_azimuth_DDM_o, //to notify ACM that a new azimuth from packet has been decoded 

    //2B distance from fifo_distance and 4 bit id from fifo_id to be popped by CCM
    output logic [15:0] distance_DDM_o, 
    output logic [3:0] id_DDM_o,
    output logic valid_datapoint_DDM_o, //function of fifo_(distance, id) not empty signal

    //fifo signals
    input logic testmode_i 
    
  );

//FIFO_IN control and data signals
  logic fifo_in_pop, fifo_in_flush, fifo_in_full, fifo_in_empty; //fifo_in_push is connect top valid_data_i input signal
  logic [3:0] fifo_in_usage; 
  logic [7:0] data_parser; //output 8 byte of fifo_in

  
  //FIFO_DISTANCE 
  logic fifo_dist_pop, fifo_dist_push, fifo_dist_flush, fifo_dist_full, fifo_dist_empty;
  logic [3:0] fifo_dist_usage;
  logic [15:0] distance_i; 

  //FIFO_ID
  logic fifo_id_pop, fifo_id_push, fifo_id_flush, fifo_id_full, fifo_id_empty;
  logic [3:0] fifo_id_usage;
  logic [3:0] id_i;


// Output handshakes
assign fifo_dist_pop = ready_CCM_i && !fifo_dist_empty; 
assign fifo_id_pop = ready_CCM_i && !fifo_id_empty;

//FIFO_IN instance 
  fifo_v3#(
    .DATA_WIDTH(8),
    .DEPTH(16),
    .FALL_THROUGH(0)
  )
  fifo_in(
    .clk_i(clk_i),
    .rst_ni(rstn_i),
    .flush_i(fifo_in_flush),
    .testmode_i(testmode_i),
    .data_i(data_i),
    .push_i(valid_data_i), //if fifo is full and valid_data_i=1 this should cause a data drop => the depth  might be big enough to avoid this case
    .pop_i(fifo_in_pop), // there might be a backpressure from both ACM and CCM 
    .full_o(fifo_in_full),
    .empty_o(fifo_in_empty),
    .usage_o(fifo_in_usage),
    .data_o(data_parser)
  );

//FIFO_DISTANCE instance of fifo_v3 common cell

  fifo_v3 #(
    .DATA_WIDTH(16),
    .DEPTH(16),
    .FALL_THROUGH(0)
  )
  fifo_distance(
    .clk_i(clk_i),
    .rst_ni(rstn_i),
    .flush_i(fifo_dist_flush), 
    .testmode_i(testmode_i),
    .data_i(distance_i),
    .push_i(fifo_dist_push), //the push might be stopped if if full_fifo_distance=0
    .pop_i(fifo_dist_pop), 
    .full_o(fifo_dist_full),
    .empty_o(fifo_dist_empty),
    .usage_o(fifo_dist_usage),
    .data_o(distance_DDM_o)
  );

//FIFO_ID instance
  
  fifo_v3 #(
    .DATA_WIDTH(4),
    .DEPTH(16),
    .FALL_THROUGH(0
)
  )
  fifo_id(
    .clk_i(clk_i),
    .rst_ni(rstn_i),
    .flush_i(fifo_id_flush),
    .testmode_i(testmode_i),
    .data_i(id_i),
    .push_i(fifo_id_push),
    .pop_i(fifo_id_pop), 
    .full_o(fifo_id_full),
    .empty_o(fifo_id_empty),
    .usage_o(fifo_id_usage),
    .data_o(id_DDM_o)
  );


typedef enum logic [2:0] {
    IDLE,
    GET_FLAG1,     // First byte of header (0xFF)
    GET_FLAG2,     // Second byte of header (0xEE)
    GET_ANGLE1,   // First byte of angle
    GET_ANGLE2,   // Second byte of angle
    DIST_HI,      // High byte of distance
    DIST_LO,      // Low byte of distance
    SKIP_JUNK     // Skip junk byte
} state_t;

state_t state, next_state;
logic [7:0] byte_buf_n, byte_buf_q;         // Temporary buffer for data
logic [5:0] dist_count_n, dist_count_q;   // Counts up to 32 for distance data
logic [3:0] id_count_n, id_count_q;

logic valid_flag;


// Sequential Logic (Registers)
always_ff @(posedge clk_i, negedge rstn_i) begin
    if (!rstn_i) begin
        state <= IDLE;
        byte_buf_q <= 8'd0;
        dist_count_q <= 6'd0;
	id_count_q<=4'd0;
    end else begin
        state <= next_state;
        byte_buf_q<=byte_buf_n;
	dist_count_q<=dist_count_n;
	id_count_q<=id_count_n;
        end
end

// Combinational Logic (State Transitions and Output Logic)
always_comb begin
    // Default values
    fifo_in_pop = 1'b0;
    fifo_dist_push = 1'b0;
    fifo_id_push=1'b0;
    valid_azimuth_DDM_o = 1'b0;
    valid_flag=1'b0;
    valid_datapoint_DDM_o=1'b0;

   distance_i = 16'd0;
   id_i=4'd0;
   id_DDM_o=4'd0;
   distance_DDM_o=4'd0;
   azimuth_DDM_o=16'd0;

    
    next_state = state;
	byte_buf_n=byte_buf_q;
	dist_count_n=dist_count_q;
	id_count_n=id_count_q;

    case (state)
        IDLE: begin
            if (valid_data_i) begin //when i receive the first valid data from phy i can go out from idle
                next_state = GET_FLAG1;
            end
        end

        GET_FLAG1: begin
            if (!fifo_in_empty) begin
                fifo_in_pop = 1;  // Pop the first byte
		byte_buf_n=data_parser;
                next_state = GET_FLAG2;
            end
        end

        GET_FLAG2: begin
            if (!fifo_in_empty) begin
		  fifo_in_pop = 1;
		   next_state = GET_ANGLE1;
                if (byte_buf_q == 8'hFF && data_parser == 8'hEE) 
			valid_flag=1'b1;
            end
        end

        GET_ANGLE1: begin
            if (!fifo_in_empty) begin
                fifo_in_pop = 1;
		byte_buf_n=data_parser;
                next_state = GET_ANGLE2;
            end
        end

        GET_ANGLE2: begin
            if (ready_ACM_i && !fifo_in_empty) begin
		fifo_in_pop = 1;
                // When ACM is ready, push the composed angle
                azimuth_DDM_o = {byte_buf_q, data_parser};
                valid_azimuth_DDM_o= 1;
                next_state = DIST_HI;
                dist_count_n = 0; // Reset distance count
		id_count_n=0;
            end
        end

        DIST_HI: begin
            if (!fifo_in_empty) begin
                fifo_in_pop = 1;  // Pop the high byte of distance
		byte_buf_n=data_parser;
                next_state = DIST_LO;
            end
        end

        DIST_LO: begin
            if (!fifo_in_empty && (!fifo_dist_full && !fifo_id_full)) begin
                fifo_in_pop = 1'b1;  // Pop the low byte of distance
                distance_i= {byte_buf_q, data_parser};  // Compose 16-bit distance
		id_i=id_count_q;
                fifo_dist_push = 1'b1;  // Push to FIFO B
		fifo_id_push=1'b1;
                next_state = SKIP_JUNK;
            end
        end

        SKIP_JUNK: begin
            if (!fifo_in_empty ) begin
                fifo_in_pop = 1;  // Discard junk byte
                if (dist_count_q == 31) begin
                    next_state = IDLE;  // All distances processed, go back to IDLE
                end else begin
                    dist_count_n = dist_count_q+ 1;
		    id_count_n =id_count_q+1;
                    next_state = DIST_HI;
                end
            end
        end
    endcase
end


endmodule
