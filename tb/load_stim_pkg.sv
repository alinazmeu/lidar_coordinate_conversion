package load_stim_pkg;
    int stim_fd;
    int num_stim = 0;

    task load_azimuth(input string stim, output logic [15:0] azimuth[$]);
        int ret;
        logic [15:0] rdata;
        stim_fd = $fopen(stim, "r");

        if (stim_fd == 0)
            $fatal(1, "could not open stimuli file!");

        while (!$feof(stim_fd)) begin
            ret = $fscanf(stim_fd, "%h\n", rdata);
            azimuth.push_back(rdata);
        end

        $fclose(stim_fd);
    endtask

    task load_distance(input string stim, output logic [15:0] distance[$]);
        int ret;
        logic [15:0] rdata;
        stim_fd = $fopen(stim, "r");

        if (stim_fd == 0)
            $fatal(1, "could not open stimuli file!");

        while (!$feof(stim_fd)) begin
            ret = $fscanf(stim_fd, "%h\n", rdata);
            distance.push_back(rdata);
        end

        $fclose(stim_fd);
    endtask
endpackage