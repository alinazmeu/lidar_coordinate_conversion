/*
 * Copyright (C) 2026 ETH Zurich, University of Bologna
 * Copyright and related rights are licensed under the Solderpad Hardware
 * License, Version 0.51 (the "License"); you may not use this file except in
 * compliance with the License.  You may obtain a copy of the License at
 * http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
 * or agreed to in writing, software, hardware and materials distributed under
 * this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied. See the License for the
 * specific language governing permissions and limitations under the License.
 *
 * Authors:
 *  - Alina Zmeu <alina.zmeu@studio.unibo.it>
 */

module coordinate_serializer (

    input  logic                 clk_i                 ,
    input  logic                 rstn_i                , 
    //input parallel
    input  logic signed [15:0]   x_i                   , 
    input  logic signed [15:0]   y_i                   , 
    input  logic signed [15:0]   z_i                   , 
    input  logic                 valid_datapoint_CCM_i ,
    output logic                 ready_serial_o        ,
    //output serialized
    input  logic                 ready_upsizer_i       , 
    output logic signed [15:0]   data_o                ,
    output logic                 valid_serial_o

);

    //3 FIFO:fifo_x, fifo_y, fifo_z => one for each coordinate in order to reduce the throughput from 3*16 bit to 16 bit at clock 
    localparam int unsigned DATA_WIDTH           =  16;
    localparam int unsigned FIFO_OUT_DEPTH       =  8;
    localparam int unsigned FIFO_OUT_USAGE_WIDTH =  $clog2(FIFO_OUT_DEPTH);

    typedef enum logic [1:0] {X_PHASE, Y_PHASE, Z_PHASE} state_t;
    state_t state, next_state;

    logic fifo_x_pop,   fifo_y_pop,   fifo_z_pop;
    logic fifo_x_full,  fifo_y_full,  fifo_z_full;
    logic fifo_x_empty, fifo_y_empty, fifo_z_empty;
    logic fifo_flush,   fifo_push;
    logic [FIFO_OUT_USAGE_WIDTH-1:0] fifo_x_usage,  fifo_y_usage,  fifo_z_usage;
    logic [DATA_WIDTH-1:0]           fifo_x_data_o, fifo_y_data_o, fifo_z_data_o;

    // Stop input writes when fifo_*_full is asserted, no more pushes untill it goes deasserted
    assign ready_serial_o = !(fifo_x_full || fifo_y_full || fifo_z_full);
    assign fifo_push = valid_datapoint_CCM_i && ready_serial_o; 

    fifo_v3 #(
        .DATA_WIDTH ( DATA_WIDTH     ) ,
        .DEPTH      ( FIFO_OUT_DEPTH )
    ) fifo_x (
        .clk_i      ( clk_i          ) ,
        .rst_ni     ( rstn_i         ) ,
        .flush_i    ( fifo_flush     ) ,
        .testmode_i ( 1'b1           ) , //to bypass clock gating
        .full_o     ( fifo_x_full    ) ,
        .empty_o    ( fifo_x_empty   ) ,
        .usage_o    ( fifo_x_usage   ) ,
        .data_i     ( x_i            ) ,          
        .push_i     ( fifo_push      ) ,         
        .data_o     ( fifo_x_data_o  ) ,         
        .pop_i      ( fifo_x_pop     )             
    );

    fifo_v3 #(
        .DATA_WIDTH ( DATA_WIDTH     ) ,
        .DEPTH      ( FIFO_OUT_DEPTH )
    ) fifo_y (
        .clk_i      ( clk_i          ) ,
        .rst_ni     ( rstn_i         ) ,
        .flush_i    ( fifo_flush     ) ,
        .testmode_i ( 1'b1           ) , //to bypass clock gating
        .full_o     ( fifo_y_full    ) ,
        .empty_o    ( fifo_y_empty   ) ,
        .usage_o    ( fifo_y_usage   ) ,
        .data_i     ( y_i            ) ,           
        .push_i     ( fifo_push      ) ,           
        .data_o     ( fifo_y_data_o  ) ,           
        .pop_i      ( fifo_y_pop     )             
    );

    fifo_v3 #(
        .DATA_WIDTH ( DATA_WIDTH     ) ,
        .DEPTH      ( FIFO_OUT_DEPTH )
    ) fifo_z (
        .clk_i      ( clk_i          ) ,
        .rst_ni     ( rstn_i         ) ,
        .flush_i    ( fifo_flush     ) ,
        .testmode_i ( 1'b1           ) , //to bypass clock gating
        .full_o     ( fifo_z_full    ) ,
        .empty_o    ( fifo_z_empty   ) ,
        .usage_o    ( fifo_z_usage   ) ,
        .data_i     ( z_i            ) ,          
        .push_i     ( fifo_push      ) ,           
        .data_o     ( fifo_z_data_o  ) ,           
        .pop_i      ( fifo_z_pop     )             
);

    // FSM for round-robin read
    always_comb begin
        fifo_x_pop     = 1'b0;
        fifo_y_pop     = 1'b0;
        fifo_z_pop     = 1'b0;
        fifo_flush     = 1'b0; 
        data_o         = 16'd0;
        valid_serial_o = 1'b0;
        next_state     = state;

        case (state)
            X_PHASE: begin
                if (!fifo_x_empty ) begin
                    valid_serial_o = 1'b1;
                    if(ready_upsizer_i)begin
                        fifo_x_pop = 1'b1; 
                        data_o     = fifo_x_data_o;
                        next_state = Y_PHASE;
                    end
                end
            end
            Y_PHASE: begin
                if (!fifo_y_empty) begin
                    valid_serial_o = 1'b1;
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

    always_ff @(posedge clk_i or negedge rstn_i) begin
        if (!rstn_i)
            state <= X_PHASE;
        else 
            state <= next_state;
    end


endmodule