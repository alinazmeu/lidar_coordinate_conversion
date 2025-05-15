module coordinate_serializer (

input logic clk_i,
input logic rstn_i, 

//input parallel
input logic signed [15:0] x_i, 
input logic signed [15:0] y_i, 
input logic signed [15:0] z_i, 
input logic valid_datapoint_CCM_i,
output logic ready_serial_o,

//output serialized
input logic ready_upsizer_i, 
output logic signed [15:0] data_o,
output logic valid_serial_o

);

//3 FIFO:fifo_x, fifo_y, fifo_z => one for each coordinate in order to reduce the throughput from 3*16 bit to 16 bit at clock 

localparam int unsigned DATA_WIDTH = 16;
localparam int unsigned FIFO_OUT_DEPTH = 8;
localparam int unsigned FIFO_OUT_USAGE_WIDTH = $clog2(FIFO_OUT_DEPTH);


logic fifo_x_pop, fifo_y_pop, fifo_z_pop;
logic fifo_flush;
logic fifo_x_full, fifo_y_full, fifo_z_full;
logic fifo_x_empty, fifo_y_empty, fifo_z_empty;
/* logic fifo_x_push, fifo_y_push, fifo_z_push; */

logic [FIFO_OUT_USAGE_WIDTH-1:0] fifo_x_usage, fifo_y_usage, fifo_z_usage;
logic [DATA_WIDTH-1:0] fifo_x_data_o, fifo_y_data_o, fifo_z_data_o;

 // Stall input write when any FIFO is full, no more push
    assign ready_serial_o = !(fifo_x_full || fifo_y_full || fifo_z_full);
    assign fifo_push = valid_datapoint_CCM_i && ready_serial_o;

fifo_v3 #(
    .DATA_WIDTH(DATA_WIDTH),
    .DEPTH(FIFO_OUT_DEPTH)
) 
fifo_x
(
    .clk_i(clk_i),
    .rst_ni(rstn_i),
    .flush_i(fifo_flush),
    .testmode_i(1'b1), //to bypass clock gating
    //status flag
    .full_o(fifo_x_full),
    .empty_o(fifo_x_empty),
    .usage_o(fifo_x_usage),
     // as long as the queue is not full we can push new data
    .data_i(x_i),           // data to push into the queue
    .push_i(fifo_push),           // data is valid and can be pushed to the queue
     // as long as the queue is not empty we can pop new elements
    .data_o(fifo_x_data_o),           // output data
    .pop_i(fifo_x_pop)             // pop head from queue
);

fifo_v3 #(
    .DATA_WIDTH(DATA_WIDTH),
    .DEPTH(FIFO_OUT_DEPTH)
)
fifo_y
(
    .clk_i(clk_i),
    .rst_ni(rstn_i),
    .flush_i(fifo_flush),
    .testmode_i(1'b1), //to bypass clock gating
    //status flag
    .full_o(fifo_y_full),
    .empty_o(fifo_y_empty),
    .usage_o(fifo_y_usage),
     // as long as the queue is not full we can push new data
    .data_i(y_i),           // data to push into the queue
    .push_i(fifo_push),           // data is valid and can be pushed to the queue
     // as long as the queue is not empty we can pop new elements
    .data_o(fifo_y_data_o),           // output data
    .pop_i(fifo_y_pop)             // pop head from queue
);

fifo_v3 #(
    .DATA_WIDTH(DATA_WIDTH),
    .DEPTH(FIFO_OUT_DEPTH)
)
fifo_z
(
    .clk_i(clk_i),
    .rst_ni(rstn_i),
    .flush_i(fifo_flush),
    .testmode_i(1'b1), //to bypass clock gating
    //status flag
    .full_o(fifo_z_full),
    .empty_o(fifo_z_empty),
    .usage_o(fifo_z_usage),
     // as long as the queue is not full we can push new data
    .data_i(z_i),           // data to push into the queue
    .push_i(fifo_push),           // data is valid and can be pushed to the queue
     // as long as the queue is not empty we can pop new elements
    .data_o(fifo_z_data_o),           // output data
    .pop_i(fifo_z_pop)             // pop head from queue
);

// FSM for round-robin read
    typedef enum logic [1:0] {X_PHASE, Y_PHASE, Z_PHASE} state_t;
    state_t state, next_state;

    always_ff @(posedge clk_i or negedge rstn_i) begin
        if (!rstn_i)
            state <= X_PHASE;
        else 
            state <= next_state;
    end

    always_comb begin
        // Default values
        fifo_x_pop = 1'b0;
        fifo_y_pop = 1'b0;
        fifo_z_pop = 1'b0;

        fifo_flush=1'b0; 

        data_o     = 16'd0;
        valid_serial_o    = 1'b0;
        next_state   = state;

        case (state)
            X_PHASE: begin
                if (!fifo_x_empty ) begin
                    valid_serial_o=1'b1;
                    if(ready_upsizer_i)begin
                        fifo_x_pop = 1'b1; 
                        data_o     = fifo_x_data_o;
                        next_state = Y_PHASE;
                    end
                end
            end
            Y_PHASE: begin
                if (!fifo_y_empty) begin
                    valid_serial_o  = 1'b1;
                    if(ready_upsizer_i) begin
                        fifo_y_pop = 1'b1;
                        data_o     = fifo_y_data_o;
                        next_state = Z_PHASE;
                    end
                end
            end
            Z_PHASE: begin
                if (!fifo_z_empty) begin
                    valid_serial_o  = 1'b1;
                    if (ready_upsizer_i) begin
                        fifo_z_pop   = 1'b1;
                        data_o       = fifo_z_data_o;
                        next_state   = X_PHASE;
                    end
                end
            end
        endcase
    end


endmodule
