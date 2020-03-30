/*
 * Copyright (c) 2017-2020, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SSP_LIB_H
#define SSP_LIB_H
#include <boot_api.h>

/* Constants */
#define SSP_BLOB_FILE_MAX_ADDR		(BL2_RW_LIMIT - PLAT_XLAT_SIZE)
#define SSP_KEY_CERTIFICATE_SIZE	U(34) // 4 * 34 bytes : 136 bytes
#define SSP_PART_ID			0xF3

/*
 * SSP message format for flashloader exchange.
 *
 * msg: Message containing public key and certificate.
 * blob: Output buffer for encrypted file.
 */
typedef struct ssp_exchange {
	uint32_t msg[SSP_KEY_CERTIFICATE_SIZE];
	uint8_t *blob;
} ssp_exchange_t __aligned(4);

/*
 * Start the SSP processing.
 *
 * Parameters:
 * boot_context	: Shared boot_context
 */
void ssp_start(boot_api_context_t *boot_context);

#endif /* SSP_LIB_H */
