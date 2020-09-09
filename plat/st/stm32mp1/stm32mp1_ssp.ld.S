/*
 * Copyright (c) 2013-2020, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#define TF_LINKER_SCRIPT
#include <platform_def.h>

OUTPUT_FORMAT(PLATFORM_LINKER_FORMAT)
OUTPUT_ARCH(PLATFORM_LINKER_ARCH)

ENTRY(__BL2_IMAGE_START__)

MEMORY {
	HEADER (rw) : ORIGIN = STM32MP_DTB_BASE, LENGTH = 0x3000
	RAM (rwx) : ORIGIN = STM32MP_BINARY_BASE, LENGTH = STM32MP_BINARY_SIZE
}

SECTIONS
{
    /*
     * TF mapping must conform to ROM code specification.
     */
    .header : {
        __HEADER_START__ = .;
        KEEP(*(.header))
        . = ALIGN(4);
        __HEADER_END__ = .;
    } >HEADER

    . = STM32MP_BINARY_BASE;
    .data . : {
        . = ALIGN(PAGE_SIZE);
        __DATA_START__ = .;
        *(.data*)

	/*
	 * Device tree
	 */
        __DTB_IMAGE_START__ = .;
        *(.dtb_image*)
        __DTB_IMAGE_END__ = .;

	/*
	 * BL2 image
	 */
	. = (STM32MP_BL2_BASE - STM32MP_BINARY_BASE);
        __BL2_IMAGE_START__ = .;
        *(.bl2_image*)
        __BL2_IMAGE_END__ = .;
        __DATA_END__ = .;
    } >RAM

    __TF_END__ = .;

}
#undef TF_LINKER_SCRIPT
