module Lidar_DDM (
    input logic clk_i, 
    input logic rstn_i,

    //AXI-S SIGNALS modport RX
     input logic [7:0] data_i,
     input logic valid_data_i,
     output logic ready_DDM_o, //if fifo_in is not full DDM is ready to push new data

    //Data and handshake with ACM module
    output logic [15:0] azimuth_DDM_o,
    output logic valid_azimuth_DDM_o, 
    input logic ready_ACM_i, //ACM is ready when in IDLE, if low data will not be popped from fifo_in

    //Data and hanshake with CCM module
    output logic [15:0] distance_DDM_o, 
    output logic [3:0] id_DDM_o,
    output logic valid_channel_DDM_o, //function of fifo_(distance, id) not empty signal
    input logic ready_CCM_i,  //function of fifo full signals in serializer module. If one of fifo is full CCM will not pop data from fifo_id, fifo_distance in DDM

    //for debugging
    output logic [7:0] nr_packets
    
  );


//FIFO_IN control and data signals
  logic fifo_in_pop, fifo_in_flush, fifo_in_full, fifo_in_empty; //fifo_in_push is connect top valid_data_i input signal

  localparam int unsigned FIFO_IN_DEPTH = 32;
  localparam int unsigned FIFO_IN_DATA_WIDTH = 8;
  localparam int unsigned FIFO_IN_USAGE_WIDTH = $clog2(FIFO_IN_DEPTH);

  logic [FIFO_IN_USAGE_WIDTH-1:0] fifo_in_usage;
  logic [FIFO_IN_DATA_WIDTH-1:0] data_parser; //output 8 byte of fifo_in

  
  //FIFO_DISTANCE 
  logic fifo_dist_pop, fifo_dist_push, fifo_dist_flush, fifo_dist_full, fifo_dist_empty;

  localparam int unsigned FIFO_DISTANCE_DEPTH = 16;
  localparam int unsigned FIFO_DISTANCE_DATA_WIDTH = 16;
  localparam int unsigned FIFO_DISTANCE_USAGE_WIDTH = $clog2(FIFO_DISTANCE_DEPTH);

  logic [FIFO_DISTANCE_USAGE_WIDTH-1:0] fifo_dist_usage;
  logic [FIFO_DISTANCE_DATA_WIDTH-1:0] distance_i; 

  //FIFO_ID
  logic fifo_id_pop, fifo_id_push, fifo_id_flush, fifo_id_full, fifo_id_empty;

  localparam int unsigned FIFO_ID_DEPTH = 16;
  localparam int unsigned FIFO_ID_DATA_WIDTH = 4;
  localparam int unsigned FIFO_ID_USAGE_WIDTH = $clog2(FIFO_ID_DEPTH);

  logic [FIFO_ID_USAGE_WIDTH-1:0] fifo_id_usage;
  logic [FIFO_ID_DATA_WIDTH-1:0] id_i;

//FIFO_IN instance 
  fifo_v3#(
    .DATA_WIDTH(FIFO_IN_DATA_WIDTH),
    .DEPTH(FIFO_IN_DEPTH),
    .FALL_THROUGH(0)
  )
  fifo_in(
    .clk_i(clk_i),
    .rst_ni(rstn_i),
    .flush_i(fifo_in_flush),
    .testmode_i(1'b1),
    .data_i(data_i),
    .push_i(valid_data_i), //if fifo is full and valid_data_i=1 this will cause a data drop => the depth  might be big enough to avoid this case
    .pop_i(fifo_in_pop), // there might be a backpressure from both ACM and CCM 
    .full_o(fifo_in_full),
    .empty_o(fifo_in_empty),
    .usage_o(fifo_in_usage),
    .data_o(data_parser)
  );

//FIFO_DISTANCE instance of fifo_v3 common cell

  fifo_v3 #(
    .DATA_WIDTH(FIFO_DISTANCE_DATA_WIDTH),
    .DEPTH(FIFO_DISTANCE_DEPTH),
    .FALL_THROUGH(0)
  )
  fifo_distance(
    .clk_i(clk_i),
    .rst_ni(rstn_i),
    .flush_i(fifo_dist_flush), 
    .testmode_i(1'b1),
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
    .DATA_WIDTH(FIFO_ID_DATA_WIDTH),
    .DEPTH(FIFO_ID_DEPTH),
    .FALL_THROUGH(0)
  )
  fifo_id(
    .clk_i(clk_i),
    .rst_ni(rstn_i),
    .flush_i(fifo_id_flush),
    .testmode_i(1'b1),
    .data_i(id_i),
    .push_i(fifo_id_push),
    .pop_i(fifo_id_pop), 
    .full_o(fifo_id_full),
    .empty_o(fifo_id_empty),
    .usage_o(fifo_id_usage),
    .data_o(id_DDM_o)
  );

//if fifos in serializer are not full, new data from fifo_id and fifo_distance can be popped and pushed in serializer fifos in the next cycle
assign fifo_dist_pop = ready_CCM_i && !fifo_dist_empty; 
assign fifo_id_pop = ready_CCM_i && !fifo_id_empty;


assign valid_channel_DDM_o = !fifo_dist_empty & !fifo_id_empty;

//a low ready will causes data drop
assign ready_DDM_o=!fifo_in_full;

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
logic [4:0] dist_count_n, dist_count_q;   // Counts up to 32 for distance data
logic [3:0] id_count_n, id_count_q;
logic [3:0] block_count_n, block_count_q; //counts up to 12 blocks
logic [7:0] packet_count_n, packet_count_q;
logic valid_flag; 

// Sequential Logic (Registers)
always_ff @(posedge clk_i, negedge rstn_i) begin
    if (!rstn_i) begin
        state <= IDLE;
        byte_buf_q <= 8'd0;
        dist_count_q <= 5'd0;
	      id_count_q<=4'd0;
        block_count_q<=4'd0;
        packet_count_q<=8'd0;
    end else begin
        state <= next_state;
        byte_buf_q<=byte_buf_n;
	      dist_count_q<=dist_count_n;
	      id_count_q<=id_count_n;
        block_count_q<=block_count_n; 
        packet_count_q<=packet_count_n;
        end
end

assign nr_packets=packet_count_q;

// Combinational Logic (State Transitions and Output Logic)
always_comb begin
    // Default values
    fifo_in_pop = 1'b0;
    fifo_dist_push = 1'b0;
    fifo_id_push=1'b0;

    fifo_in_flush=1'b0;
    fifo_dist_flush=1'b0;
    fifo_id_flush=1'b0;

    valid_azimuth_DDM_o = 1'b0;
    valid_flag=1'b0;
    azimuth_DDM_o=16'd0;

    distance_i = 16'd0; 
    id_i=4'd0;
  
  next_state = state;
	byte_buf_n=byte_buf_q;
	dist_count_n=dist_count_q;
	id_count_n=id_count_q;
  block_count_n=block_count_q;
  packet_count_n=packet_count_q;

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
              if (byte_buf_q == 8'hFF && data_parser == 8'hEE)  begin
			        valid_flag=1'b1;
              next_state = GET_ANGLE1;
              end else 
                byte_buf_n=data_parser;
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
          if(!fifo_in_empty)  begin
            valid_azimuth_DDM_o= 1;
            if (ready_ACM_i ) begin
		            fifo_in_pop = 1;
                // When ACM is ready, push the composed angle
                azimuth_DDM_o = {data_parser, byte_buf_q};
                next_state = DIST_HI;
                dist_count_n = 0; // Reset distance count
		            id_count_n=0;
            end
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
                distance_i= {data_parser, byte_buf_q};  // Compose 16-bit distance
		            id_i=id_count_q;
                fifo_dist_push = 1'b1;  // Push to FIFO B
	            	fifo_id_push=1'b1;
                next_state = SKIP_JUNK;
            end
        end

        SKIP_JUNK: begin
            if (!fifo_in_empty ) begin
                fifo_in_pop = 1;  // Discard junk byte
                if (dist_count_q == 31) begin //32 distances in a data block
                    if(block_count_q == 11) begin //12 data block in a data packet
                      next_state=IDLE;
                      packet_count_n=packet_count_q+8'd1;
                      block_count_n=4'd0;
                      fifo_in_flush=1'b1;
                    end
                    else begin
                      block_count_n = block_count_q + 4'd1;
                      next_state=GET_FLAG1;
                     end 
                
                end else begin
                    dist_count_n = dist_count_q+ 5'd1;
		                id_count_n =id_count_q+4'd1;
                    next_state = DIST_HI;
                end
            
        end
        end
    endcase
end


endmodule
