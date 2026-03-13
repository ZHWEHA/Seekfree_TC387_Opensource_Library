################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
"../Code/Data.c" 

COMPILED_SRCS += \
"Code/Data.src" 

C_DEPS += \
"./Code/Data.d" 

OBJS += \
"Code/Data.o" 


# Each subdirectory must supply rules for building sources it contributes
"Code/Data.src":"../Code/Data.c" "Code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 "-fF:/AURIX_Workspace/Seekfree_TC387_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O2 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc38x -Y0 -N0 -Z0 -o "$@" "$<"
"Code/Data.o":"Code/Data.src" "Code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-Code

clean-Code:
	-$(RM) ./Code/Data.d ./Code/Data.o ./Code/Data.src

.PHONY: clean-Code

