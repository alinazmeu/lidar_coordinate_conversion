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

package load_stim_pkg;
   int stim_fd; //  is a package level variable can cause issues if multiple task are running concurrently, better to declare locally within each task
    int num_stim = 0;

    task load_azimuth(input string stim, output logic [15:0] azimuth[$]);
        int ret; 
        logic [15:0] rdata;
        stim_fd = $fopen(stim, "r");

        if (stim_fd == 0)
            $fatal(1, "Error: Could not open stimuli file %s!", stim);

        while (!$feof(stim_fd)) begin
            ret = $fscanf(stim_fd, "%h\n", rdata); //fscanf return the number of successfully scanned items, a check on ret can prevent unexpected behavior
            azimuth.push_back(rdata); 
        end

        $fclose(stim_fd);
    endtask

    task load_distance(input string stim, output logic [15:0] distance[$]);
        int ret;
        logic [15:0] rdata;
        stim_fd = $fopen(stim, "r");

        if (stim_fd == 0)
            $fatal(1, "Error: Could not open stimuli file %s!", stim);

        while (!$feof(stim_fd)) begin
            ret = $fscanf(stim_fd, "%h\n", rdata);
            distance.push_back(rdata);
        end

        $fclose(stim_fd);
    endtask

   task load_bytes(input string stim, output logic [7:0] rx[$]);
      int ret; 
      logic [7:0] rdata;
        stim_fd = $fopen(stim, "r");

        if (stim_fd == 0)
            $fatal(1, "Error: Could not open stimuli file %s!", stim);

        while (!$feof(stim_fd)) begin
            ret = $fscanf(stim_fd, "%h", rdata); //fscanf return the number of successfully scanned items, a check on ret can prevent unexpected behavior
            rx.push_back(rdata); 
        end

        $fclose(stim_fd);
   endtask
endpackage
