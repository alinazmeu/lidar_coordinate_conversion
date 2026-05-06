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

package write_out_pkg;

task write_x_out(input logic signed[15:0] x[$], string path);
    int file;

    file = $fopen(path, "w");
    if(file == 0)
        $fatal(1, "Error: Could not create sim/%s!", path);

    foreach(x[i]) begin
        $fwrite(file, "%d\n", x[i]);
    end

    $fclose(file);
    $display("x outputs written correctly to sim/%s", path);
endtask

task write_y_out(input logic signed[15:0] y[$], string path);
    int file;

    file = $fopen(path, "w");
    if(file == 0)
        $fatal(1, "Error: Could not create sim/%s!", path);

    foreach(y[i]) begin
        $fwrite(file, "%d\n", y[i]);
    end

    $fclose(file);
    $display("y outputs written correctly to sim/%s", path);
endtask

task write_z_out(input logic signed[15:0] z[$], string path);
    int file;

    file = $fopen(path, "w");
    if(file == 0)
        $fatal(1, "Error: Could not create sim/%s!", path);

    foreach(z[i]) begin
        $fwrite(file, "%d\n", z[i]);
    end

    $fclose(file);
    $display("z outputs written correctly to sim/%s", path);
endtask

task write_serial_out(input logic signed[15:0] serial[$], string path);
    int file;

    file = $fopen(path, "w");
    if(file == 0)
        $fatal(1, "Error: Could not create sim/%s!", path);

    foreach(serial[i]) begin
        $fwrite(file, "%d\n", serial[i]);
    end

    $fclose(file);
    $display("z outputs written correctly to sim/%s", path);
endtask

endpackage