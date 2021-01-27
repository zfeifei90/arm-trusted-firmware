/*
 * Copyright (c) 2020-2021, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <endian.h>
#include <errno.h>
#include <limits.h>

#include <common/debug.h>
#include <common/tbbr/cot_def.h>
#include <drivers/st/stm32_hash.h>
#include <lib/fconf/fconf.h>
#include <lib/fconf/fconf_dyn_cfg_getter.h>
#include <lib/fconf/fconf_tbbr_getter.h>
#include <lib/xlat_tables/xlat_tables_v2.h>
#include <plat/common/platform.h>

#include <boot_api.h>
#include <platform_def.h>

#define HEADER_AND_EXT_TOTAL_SIZE 512

static uint8_t der_sha256_header[] = {0x30, 0x31, 0x30, 0x0d, 0x06, 0x09, 0x60,
	0x86, 0x48, 0x01, 0x65, 0x03, 0x04, 0x02, 0x01, 0x05, 0x00, 0x04, 0x20};
static uint8_t root_pk_hash[HASH_DER_LEN];

static int copy_hash_from_otp(const char *otp_name, uint8_t *hash, size_t len)
{
	uint32_t otp_idx;
	uint32_t otp_len;
	size_t i;

	assert(len % sizeof(uint32_t) == 0);

	if (stm32_get_otp_index(otp_name, &otp_idx, &otp_len) != 0) {
		VERBOSE("%s: get %s index error\n", __func__, otp_name);
		return -EINVAL;
	}
	if (otp_len != (len * CHAR_BIT)) {
		VERBOSE("%s: length Error\n", __func__);
		return -EINVAL;
	}

	for (i = 0U; i < len / sizeof(uint32_t); i++) {
		uint32_t tmp;
		uint32_t otp_val;

		if (stm32_get_otp_value_from_idx(otp_idx + i, &otp_val) != 0) {
			VERBOSE("%s: unable to read from otp\n", __func__);
			return -EINVAL;
		}

		tmp = bswap32(otp_val);
		memcpy(hash + i * sizeof(uint32_t), &tmp, sizeof(tmp));
	}

	return len;
}

#if STM32MP13
static int get_rotpk_hash(void *cookie, uint8_t *hash, size_t len)
{
	int ret;
	uint32_t pk_idx = 0U;
	uint8_t calc_hash[BOOT_API_SHA256_DIGEST_SIZE_IN_BYTES];
	uint8_t otp_hash[BOOT_API_SHA256_DIGEST_SIZE_IN_BYTES];
	boot_api_image_header_t *hdr = (boot_api_image_header_t *)(SRAM3_BASE + SRAM3_SIZE -
								   HEADER_AND_EXT_TOTAL_SIZE);
	boot_extension_header_t *ext_header = (boot_extension_header_t *)hdr->ext_header;
	boot_ext_header_params_authentication_t *param;

	if (cookie != NULL) {
		return -EINVAL;
	}

	if (hdr->header_version != BOOT_API_HEADER_VERSION) {
		VERBOSE("%s: unexpected header_version\n", __func__);
		return -EINVAL;
	}

	param = (boot_ext_header_params_authentication_t *)ext_header->params;

	pk_idx = param->pk_idx;

	stm32_hash_init(HASH_SHA256);
	ret = stm32_hash_final_update((uint8_t *)param->pk_hashes,
				      param->nb_pk * sizeof(boot_api_sha256_t), calc_hash);
	if (ret != 0) {
		VERBOSE("%s: hash failed\n", __func__);
		return -EINVAL;
	}

	if (copy_hash_from_otp(PKH_OTP, otp_hash, len) < 0) {
		return -EINVAL;
	}

	ret = memcmp(calc_hash, otp_hash, sizeof(calc_hash));
	if (ret != 0) {
		VERBOSE("%s: not expected digest\n", __func__);
		return -EINVAL;
	}

	memcpy(hash, param->pk_hashes[pk_idx], sizeof(otp_hash));
	ret = sizeof(otp_hash);

	return ret;
}
#endif

#if STM32MP15
static int get_rotpk_hash(void *cookie, uint8_t *hash, size_t len)
{
	if (cookie != NULL) {
		return -EINVAL;
	}

	return copy_hash_from_otp(PKH_OTP, hash, len);
}
#endif

int plat_get_rotpk_info(void *cookie, void **key_ptr, unsigned int *key_len,
			unsigned int *flags)
{
	size_t start_copy_idx = 0U;

	memcpy(root_pk_hash, der_sha256_header, sizeof(der_sha256_header));
	start_copy_idx = sizeof(der_sha256_header);

	if (get_rotpk_hash(cookie, root_pk_hash + start_copy_idx,
			   BOOT_API_SHA256_DIGEST_SIZE_IN_BYTES) < 0) {
		return -EINVAL;
	}

	*key_len = HASH_DER_LEN;
	*key_ptr = &root_pk_hash;
	*flags = ROTPK_IS_HASH;

	if (!stm32mp_is_closed_device()) {
		/* Check if key hash values in OTP are 0 or 0xFFFFFFFFF programmed : Invalid Key */
		uint32_t res;
		uint32_t rootpk;
		uint8_t *proot_pk = root_pk_hash;
		uint8_t idx = sizeof(uint32_t);

		idx += sizeof(der_sha256_header);
		proot_pk = root_pk_hash + sizeof(der_sha256_header);

		memcpy(&res, proot_pk, sizeof(uint32_t));
		if ((res == 0U) || (res == 0xFFFFFFFFU)) {
			while (idx < ARRAY_SIZE(root_pk_hash)) {
				memcpy(&rootpk, root_pk_hash + idx, sizeof(uint32_t));
				if (res != rootpk) {
					return 0;
				}

				idx += sizeof(uint32_t);
			}

			*flags |= ROTPK_NOT_DEPLOYED;
		}
	}

	return 0;
}

int plat_get_nv_ctr(void *cookie, unsigned int *nv_ctr)
{
	/*
	 * This monotonic counter is the counter used by ROM code
	 * to identify BL2.
	 */
	if (stm32_get_otp_value(MONOTONIC_OTP, nv_ctr) == 0) {
		return 0;
	}

	return -EINVAL;
}

int plat_set_nv_ctr(void *cookie, unsigned int nv_ctr)
{
	return -EINVAL;
}

int plat_get_mbedtls_heap(void **heap_addr, size_t *heap_size)
{
	assert(heap_addr != NULL);
	assert(heap_size != NULL);

#if STM32MP_USE_EXTERNAL_HEAP
	/* Retrieve the already allocated heap's info from DTB */
	*heap_addr = FCONF_GET_PROPERTY(tbbr, dyn_config, mbedtls_heap_addr);
	*heap_size = FCONF_GET_PROPERTY(tbbr, dyn_config, mbedtls_heap_size);

	/* We expect heap already statically mapped */

	return 0;
#else
	return get_mbedtls_heap_helper(heap_addr, heap_size);
#endif
}
