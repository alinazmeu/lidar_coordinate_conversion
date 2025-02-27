QUESTA ?= 
BENDER ?= ./bender

mkfile_path	:= $(dir $(abspath $(firstword $(MAKEFILE_LIST))))
BUILD_DIR	?= $(mkfile_path)/sim/build

compile_script = compile.tcl

questa_compile_flag  += -t 1ns -suppress 3009
questa_opt_flag      += -suppress 3009 -debugdb +acc=npr

tb ?= TB_Lidar_top_level

# Download bender
bender:
	curl --proto '=https'  \
	--tlsv1.2 https://pulp-platform.github.io/bender/init -sSf | sh -s -- 0.28.1

# Generate simulation scripts
gen_sim_scripts:
	mkdir -p sim; \
	$(BENDER) script vsim \
	-t rtl -t test > sim/compile.tcl

# Generate Synopsys DC scripts
gen_synth_scripts:
	$(BENDER) script synopsys \
	-t rtl -t synthesis > gf22/synopsys/scripts/analyze.tcl

# Build implicit rules
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

hw-clean:
	rm -rf $(BUILD_DIR)/work sim/modelsim.ini

hw-lib:
	@touch sim/modelsim.ini
	@mkdir -p $(BUILD_DIR)
	@cd sim; $(QUESTA) vlib $(BUILD_DIR)/work
	@cd sim; $(QUESTA) vmap work $(BUILD_DIR)/work
	@chmod +w sim/modelsim.ini

hw-compile:
	cd sim; $(QUESTA) vsim $(questa_compile_flag) -c +incdir+$(UVM_HOME) -do 'quit -code [source $(compile_script)]'

hw-opt:
	cd sim; $(QUESTA) vopt $(questa_opt_flag) -o vopt_tb $(tb) -floatparameters+$(tb) -work $(BUILD_DIR)/work

hw-all: hw-clean hw-lib hw-compile hw-opt

run:
ifeq ($(gui),1)
	cd sim;											\
	$(QUESTA) vsim vopt_tb $(questa_run_fast_flag)
	
else
	cd sim;											\
	$(QUESTA) vsim -c vopt_tb $(questa_run_fast_flag) -do 'run -a; quit'
endif
