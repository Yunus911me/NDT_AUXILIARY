################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Add inputs and outputs from these tool invocations to the build variables 
CMD_SRCS += \
../28002x_generic_ram_lnk.cmd 

SYSCFG_SRCS += \
../ndt_board.syscfg 

LIB_SRCS += \
C:/ti/C2000Ware_26_00_00_00/driverlib/f28002x/driverlib/ccs/Debug/driverlib.lib 

ASM_SRCS += \
C:/ti/C2000Ware_26_00_00_00/device_support/f28002x/common/source/f28002x_codestartbranch.asm 

C_SRCS += \
../i2ca_eeprom.c \
../m24m01e.c \
../main.c \
../max14808.c \
./syscfg/board.c \
./syscfg/device.c \
./syscfg/c2000ware_libraries.c 

GEN_FILES += \
./syscfg/board.c \
./syscfg/board.opt \
./syscfg/device.c \
./syscfg/c2000ware_libraries.opt \
./syscfg/c2000ware_libraries.c 

GEN_MISC_DIRS += \
./syscfg 

C_DEPS += \
./i2ca_eeprom.d \
./m24m01e.d \
./main.d \
./max14808.d \
./syscfg/board.d \
./syscfg/device.d \
./syscfg/c2000ware_libraries.d 

GEN_OPTS += \
./syscfg/board.opt \
./syscfg/c2000ware_libraries.opt 

OBJS += \
./i2ca_eeprom.obj \
./m24m01e.obj \
./main.obj \
./max14808.obj \
./syscfg/board.obj \
./syscfg/device.obj \
./syscfg/c2000ware_libraries.obj \
./f28002x_codestartbranch.obj 

ASM_DEPS += \
./f28002x_codestartbranch.d 

GEN_MISC_FILES += \
./syscfg/board.h \
./syscfg/board.cmd.genlibs \
./syscfg/board.json \
./syscfg/pinmux.csv \
./syscfg/device.h \
./syscfg/adc.dot \
./syscfg/c2000ware_libraries.cmd.genlibs \
./syscfg/c2000ware_libraries.h \
./syscfg/clocktree.h 

GEN_MISC_DIRS__QUOTED += \
"syscfg" 

OBJS__QUOTED += \
"i2ca_eeprom.obj" \
"m24m01e.obj" \
"main.obj" \
"max14808.obj" \
"syscfg\board.obj" \
"syscfg\device.obj" \
"syscfg\c2000ware_libraries.obj" \
"f28002x_codestartbranch.obj" 

GEN_MISC_FILES__QUOTED += \
"syscfg\board.h" \
"syscfg\board.cmd.genlibs" \
"syscfg\board.json" \
"syscfg\pinmux.csv" \
"syscfg\device.h" \
"syscfg\adc.dot" \
"syscfg\c2000ware_libraries.cmd.genlibs" \
"syscfg\c2000ware_libraries.h" \
"syscfg\clocktree.h" 

C_DEPS__QUOTED += \
"i2ca_eeprom.d" \
"m24m01e.d" \
"main.d" \
"max14808.d" \
"syscfg\board.d" \
"syscfg\device.d" \
"syscfg\c2000ware_libraries.d" 

GEN_FILES__QUOTED += \
"syscfg\board.c" \
"syscfg\board.opt" \
"syscfg\device.c" \
"syscfg\c2000ware_libraries.opt" \
"syscfg\c2000ware_libraries.c" 

ASM_DEPS__QUOTED += \
"f28002x_codestartbranch.d" 

C_SRCS__QUOTED += \
"../i2ca_eeprom.c" \
"../m24m01e.c" \
"../main.c" \
"../max14808.c" \
"./syscfg/board.c" \
"./syscfg/device.c" \
"./syscfg/c2000ware_libraries.c" 

SYSCFG_SRCS__QUOTED += \
"../ndt_board.syscfg" 

ASM_SRCS__QUOTED += \
"C:/ti/C2000Ware_26_00_00_00/device_support/f28002x/common/source/f28002x_codestartbranch.asm" 


