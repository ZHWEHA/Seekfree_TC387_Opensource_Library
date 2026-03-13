################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
"../Code/Diff_steering/attitude.c" 

COMPILED_SRCS += \
"Code/Diff_steering/attitude.src" 

C_DEPS += \
"./Code/Diff_steering/attitude.d" 

OBJS += \
"Code/Diff_steering/attitude.o" 


# Each subdirectory must supply rules for building sources it contributes
"Code/Diff_steering/attitude.src":"../Code/Diff_steering/attitude.c" "Code/Diff_steering/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 "-fD:/git_code/test/TEST/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O2 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc38x -Y0 -N0 -Z0 -o "$@" "$<"
"Code/Diff_steering/attitude.o":"Code/Diff_steering/attitude.src" "Code/Diff_steering/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-Code-2f-Diff_steering

clean-Code-2f-Diff_steering:
	-$(RM) ./Code/Diff_steering/attitude.d ./Code/Diff_steering/attitude.o ./Code/Diff_steering/attitude.src

.PHONY: clean-Code-2f-Diff_steering

