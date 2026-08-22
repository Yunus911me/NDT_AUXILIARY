################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'C2000 Compiler - building file: "$<"'
	"C:/ti/ccs2040/ccs/tools/compiler/ti-cgt-c2000_22.6.3.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --idiv_support=idiv0 --tmu_support=tmu0 -Ooff --include_path="C:/Users/ahmad/workspace_ccstheia/NDT_AUXILIARY" --include_path="C:/Users/ahmad/workspace_ccstheia/NDT_AUXILIARY/device" --include_path="C:/ti/C2000Ware_26_00_00_00/driverlib/f28002x/driverlib/" --include_path="C:/ti/ccs2040/ccs/tools/compiler/ti-cgt-c2000_22.6.3.LTS/include" --define=DEBUG --define=RAM --c99 --diag_suppress=10063 --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" --include_path="C:/Users/ahmad/workspace_ccstheia/NDT_AUXILIARY/CPU1_RAM/syscfg" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

build-1431934398: ../ndt_board.syscfg
	@echo 'SysConfig - building file: "$<"'
	"C:/ti/ccs2040/ccs/utils/sysconfig_1.27.0/sysconfig_cli.bat" -s "C:/ti/C2000Ware_26_00_00_00/.metadata/sdk.json" -d "F28002x" -p "80QFP" -r "F28002x_80QFP" --script "C:/Users/ahmad/workspace_ccstheia/NDT_AUXILIARY/ndt_board.syscfg" -o "syscfg" --compiler ccs
	@echo 'Finished building: "$<"'
	@echo ' '

syscfg/board.c: build-1431934398 ../ndt_board.syscfg
syscfg/board.h: build-1431934398
syscfg/board.cmd.genlibs: build-1431934398
syscfg/board.opt: build-1431934398
syscfg/board.json: build-1431934398
syscfg/pinmux.csv: build-1431934398
syscfg/device.c: build-1431934398
syscfg/device.h: build-1431934398
syscfg/adc.dot: build-1431934398
syscfg/c2000ware_libraries.cmd.genlibs: build-1431934398
syscfg/c2000ware_libraries.opt: build-1431934398
syscfg/c2000ware_libraries.c: build-1431934398
syscfg/c2000ware_libraries.h: build-1431934398
syscfg/clocktree.h: build-1431934398
syscfg: build-1431934398

syscfg/%.obj: ./syscfg/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'C2000 Compiler - building file: "$<"'
	"C:/ti/ccs2040/ccs/tools/compiler/ti-cgt-c2000_22.6.3.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --idiv_support=idiv0 --tmu_support=tmu0 -Ooff --include_path="C:/Users/ahmad/workspace_ccstheia/NDT_AUXILIARY" --include_path="C:/Users/ahmad/workspace_ccstheia/NDT_AUXILIARY/device" --include_path="C:/ti/C2000Ware_26_00_00_00/driverlib/f28002x/driverlib/" --include_path="C:/ti/ccs2040/ccs/tools/compiler/ti-cgt-c2000_22.6.3.LTS/include" --define=DEBUG --define=RAM --c99 --diag_suppress=10063 --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --preproc_with_compile --preproc_dependency="syscfg/$(basename $(<F)).d_raw" --include_path="C:/Users/ahmad/workspace_ccstheia/NDT_AUXILIARY/CPU1_RAM/syscfg" --obj_directory="syscfg" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

f28002x_codestartbranch.obj: C:/ti/C2000Ware_26_00_00_00/device_support/f28002x/common/source/f28002x_codestartbranch.asm $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'C2000 Compiler - building file: "$<"'
	"C:/ti/ccs2040/ccs/tools/compiler/ti-cgt-c2000_22.6.3.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --idiv_support=idiv0 --tmu_support=tmu0 -Ooff --include_path="C:/Users/ahmad/workspace_ccstheia/NDT_AUXILIARY" --include_path="C:/Users/ahmad/workspace_ccstheia/NDT_AUXILIARY/device" --include_path="C:/ti/C2000Ware_26_00_00_00/driverlib/f28002x/driverlib/" --include_path="C:/ti/ccs2040/ccs/tools/compiler/ti-cgt-c2000_22.6.3.LTS/include" --define=DEBUG --define=RAM --c99 --diag_suppress=10063 --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" --include_path="C:/Users/ahmad/workspace_ccstheia/NDT_AUXILIARY/CPU1_RAM/syscfg" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


