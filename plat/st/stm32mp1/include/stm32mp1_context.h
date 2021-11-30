/*
 * Copyright (c) 2017-2021, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef STM32MP1_CONTEXT_H
#define STM32MP1_CONTEXT_H

#include <stdint.h>

void stm32_clean_context(void);
void stm32_context_save_bl2_param(void);
uint32_t stm32_get_zdata_from_context(void);
void stm32_restore_ddr_training_area(void);
uint32_t stm32_pm_get_optee_ep(void);

#endif /* STM32MP1_CONTEXT_H */
