################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
"../Code/Process/Display.c" \
"../Code/Process/Function.c" \
"../Code/Process/Motor.c" \
"../Code/Process/Process.c" \
"../Code/Process/Remote.c" \
"../Code/Process/State.c" \
"../Code/Process/Wireless.c" 

COMPILED_SRCS += \
"Code/Process/Display.src" \
"Code/Process/Function.src" \
"Code/Process/Motor.src" \
"Code/Process/Process.src" \
"Code/Process/Remote.src" \
"Code/Process/State.src" \
"Code/Process/Wireless.src" 

C_DEPS += \
"./Code/Process/Display.d" \
"./Code/Process/Function.d" \
"./Code/Process/Motor.d" \
"./Code/Process/Process.d" \
"./Code/Process/Remote.d" \
"./Code/Process/State.d" \
"./Code/Process/Wireless.d" 

OBJS += \
"Code/Process/Display.o" \
"Code/Process/Function.o" \
"Code/Process/Motor.o" \
"Code/Process/Process.o" \
"Code/Process/Remote.o" \
"Code/Process/State.o" \
"Code/Process/Wireless.o" 


# Each subdirectory must supply rules for building sources it contributes
"Code/Process/Display.src":"../Code/Process/Display.c" "Code/Process/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 "-fF:/AURIX_Workspace/Seekfree_TC387_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O2 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc38x -Y0 -N0 -Z0 -o "$@" "$<"
"Code/Process/Display.o":"Code/Process/Display.src" "Code/Process/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"Code/Process/Function.src":"../Code/Process/Function.c" "Code/Process/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 "-fF:/AURIX_Workspace/Seekfree_TC387_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O2 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc38x -Y0 -N0 -Z0 -o "$@" "$<"
"Code/Process/Function.o":"Code/Process/Function.src" "Code/Process/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"Code/Process/Motor.src":"../Code/Process/Motor.c" "Code/Process/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 "-fF:/AURIX_Workspace/Seekfree_TC387_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O2 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc38x -Y0 -N0 -Z0 -o "$@" "$<"
"Code/Process/Motor.o":"Code/Process/Motor.src" "Code/Process/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"Code/Process/Process.src":"../Code/Process/Process.c" "Code/Process/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 "-fF:/AURIX_Workspace/Seekfree_TC387_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O2 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc38x -Y0 -N0 -Z0 -o "$@" "$<"
"Code/Process/Process.o":"Code/Process/Process.src" "Code/Process/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"Code/Process/Remote.src":"../Code/Process/Remote.c" "Code/Process/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 "-fF:/AURIX_Workspace/Seekfree_TC387_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O2 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc38x -Y0 -N0 -Z0 -o "$@" "$<"
"Code/Process/Remote.o":"Code/Process/Remote.src" "Code/Process/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"Code/Process/State.src":"../Code/Process/State.c" "Code/Process/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 "-fF:/AURIX_Workspace/Seekfree_TC387_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O2 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc38x -Y0 -N0 -Z0 -o "$@" "$<"
"Code/Process/State.o":"Code/Process/State.src" "Code/Process/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"Code/Process/Wireless.src":"../Code/Process/Wireless.c" "Code/Process/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2012 "-fF:/AURIX_Workspace/Seekfree_TC387_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O2 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc38x -Y0 -N0 -Z0 -o "$@" "$<"
"Code/Process/Wireless.o":"Code/Process/Wireless.src" "Code/Process/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-Code-2f-Process

clean-Code-2f-Process:
	-$(RM) ./Code/Process/Display.d ./Code/Process/Display.o ./Code/Process/Display.src ./Code/Process/Function.d ./Code/Process/Function.o ./Code/Process/Function.src ./Code/Process/Motor.d ./Code/Process/Motor.o ./Code/Process/Motor.src ./Code/Process/Process.d ./Code/Process/Process.o ./Code/Process/Process.src ./Code/Process/Remote.d ./Code/Process/Remote.o ./Code/Process/Remote.src ./Code/Process/State.d ./Code/Process/State.o ./Code/Process/State.src ./Code/Process/Wireless.d ./Code/Process/Wireless.o ./Code/Process/Wireless.src

.PHONY: clean-Code-2f-Process

