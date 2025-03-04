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
endpackage