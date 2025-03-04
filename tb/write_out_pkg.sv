package write_out_pkg;

task write_x_out(input logic signed[17:0] x[$], string path);
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

task write_y_out(input logic signed[17:0] y[$], string path);
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

task write_z_out(input logic signed[17:0] z[$], string path);
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

endpackage