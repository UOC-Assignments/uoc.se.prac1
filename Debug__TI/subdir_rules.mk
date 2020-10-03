################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccs901/ccs/tools/compiler/ti-cgt-arm_18.12.3.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="C:/ti/ccs901/ccs/ccs_base/arm/include" --include_path="C:/Users/jordi/Google Drive/beritheundeleted@gmail.com/03. UOC/workspaces/SE/2019-1-prac/lib_PRAC/freertos/cortex-m4" --include_path="C:/Users/jordi/Google Drive/beritheundeleted@gmail.com/03. UOC/workspaces/SE/2019-1-prac/lib_PRAC/freertos/inc" --include_path="C:/Users/jordi/Google Drive/beritheundeleted@gmail.com/03. UOC/workspaces/SE/2019-1-prac/lib_PRAC/freertos/src" --include_path="C:/Users/jordi/Google Drive/beritheundeleted@gmail.com/03. UOC/workspaces/SE/2019-1-prac/lib_PRAC/inc" --include_path="C:/Users/jordi/Google Drive/beritheundeleted@gmail.com/03. UOC/workspaces/SE/2019-1-prac/lib_PRAC/msp432" --include_path="C:/Users/jordi/Google Drive/beritheundeleted@gmail.com/03. UOC/workspaces/SE/2019-1-prac/lib_PRAC/uoc" --include_path="C:/Users/jordi/Google Drive/beritheundeleted@gmail.com/03. UOC/workspaces/SE/2019-1-prac/lib_PRAC/screen" --include_path="C:/Users/jordi/Google Drive/beritheundeleted@gmail.com/03. UOC/workspaces/SE/2019-1-prac/lib_PRAC/graphics" --include_path="C:/ti/ccs901/ccs/ccs_base/arm/include/CMSIS" --include_path="C:/Users/jordi/Google Drive/beritheundeleted@gmail.com/03. UOC/workspaces/SE/2019-1-prac/PRAC-Projecte-1_FINAL" --include_path="C:/ti/ccs901/ccs/tools/compiler/ti-cgt-arm_18.12.3.LTS/include" --advice:power=all --define=__MSP432P401R__ --define=ccs -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


