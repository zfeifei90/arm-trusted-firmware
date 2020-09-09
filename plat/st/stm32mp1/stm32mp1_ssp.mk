#
# Copyright (c) 2015-2020, ARM Limited and Contributors. All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
#

ST_VERSION 		:=	r1.0
VERSION_STRING		:=	v${VERSION_MAJOR}.${VERSION_MINOR}-${ST_VERSION}(${BUILD_TYPE}):${BUILD_STRING}

# Required to use BL2_IN_XIP_MEM
BL2_IN_XIP_MEM 		:= 	1
$(eval $(call add_define,BL2_IN_XIP_MEM))

SEPARATE_CODE_AND_RODATA :=	1

# Macros and rules to build TF-A binary
STM32_TF_ELF_LDFLAGS		:=	--hash-style=gnu --as-needed
STM32_DT_BASENAME		:= 	$(DTB_FILE_NAME:.dtb=)
STM32_TF_SSP_STM32 		:= 	${BUILD_PLAT}/tf-a-ssp-${STM32_DT_BASENAME}.stm32
STM32_TF_SSP_BINARY 		:= 	$(STM32_TF_SSP_STM32:.stm32=.bin)
STM32_TF_SSP_MAPFILE		:=	$(STM32_TF_SSP_STM32:.stm32=.map)
STM32_TF_SSP_LINKERFILE		:=	$(STM32_TF_SSP_STM32:.stm32=.ld)
STM32_TF_SSP_ELF 		:= 	$(STM32_TF_SSP_STM32:.stm32=.elf)
STM32_TF_SSP_OBJS		:=	${BUILD_PLAT}/stm32mp1-ssp.o
STM32_TF_SSP_DTBFILE		:=      ${BUILD_PLAT}/fdts/${DTB_FILE_NAME}

BL2_SOURCES		+=	lib/ssp/ssp.c

${STM32_TF_SSP_OBJS}: plat/st/stm32mp1/stm32mp1_ssp.S bl2 ${STM32_TF_SSP_DTBFILE}
	@echo "  AS      $<"
	${Q}${AS} ${ASFLAGS} ${TF_CFLAGS} \
		-DBL2_BIN_PATH=\"${BUILD_PLAT}/bl2.bin\" \
		-DDTB_BIN_PATH=\"${STM32_TF_SSP_DTBFILE}\" \
		-c plat/st/stm32mp1/stm32mp1_ssp.S -o $@


${STM32_TF_SSP_LINKERFILE}: plat/st/stm32mp1/stm32mp1_ssp.ld.S
	@echo "  LDS     $<"
	${Q}${AS} ${ASFLAGS} ${TF_CFLAGS} -P -E $< -o $@

${STM32_TF_SSP_ELF}: ${STM32_TF_SSP_OBJS} ${STM32_TF_SSP_LINKERFILE}
	@echo "  LDS     $<"
	${Q}${LD} -o $@ ${STM32_TF_ELF_LDFLAGS} \
		-Map=${STM32_TF_SSP_MAPFILE} \
		--script ${STM32_TF_SSP_LINKERFILE} \
		${STM32_TF_SSP_OBJS}

${STM32_TF_SSP_BINARY}: ${STM32_TF_SSP_ELF}
	${Q}${OC} -O binary ${STM32_TF_SSP_ELF} $@
	@echo
	@echo "Built $@ successfully"
	@echo

${STM32_TF_SSP_STM32}: check_dtc_version stm32image ${STM32_TF_SSP_BINARY}
	@echo
	@echo "Generated $@"
	$(eval LOADADDR = $(shell cat ${STM32_TF_SSP_MAPFILE} | grep RAM | awk '{print $$2}'))
	$(eval ENTRY =  $(shell cat ${STM32_TF_SSP_MAPFILE} | grep "__BL2_IMAGE_START" | awk '{print $$1}'))
	${STM32IMAGE} -s ${STM32_TF_SSP_BINARY} -d $@ \
		-l $(LOADADDR) -e ${ENTRY} \
		-v ${STM32_TF_VERSION} \
		-m ${STM32_HEADER_VERSION_MAJOR} \
		-n ${STM32_HEADER_VERSION_MINOR}
	@echo

all: ${STM32_TF_SSP_STM32}
