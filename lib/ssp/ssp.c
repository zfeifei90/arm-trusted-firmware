/*
 * Copyright (c) 2017-2020, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <arch_helpers.h>
#include <endian.h>
#include <limits.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <platform_def.h>

#include <drivers/io/io_driver.h>
#include <drivers/st/bsec.h>
#include <drivers/st/io_stm32image.h>
#include <drivers/st/stm32mp_pmic.h>
#include <drivers/st/stm32_hash.h>
#include <drivers/st/stpmic1.h>
#include <lib/mmio.h>
#include <lib/ssp_lib.h>
#include <lib/utils.h>
#include <lib/utils_def.h>
#include <plat/common/platform.h>

/* Local status for SSP processing sequences */
typedef enum {
	SSP_NONE,
	SSP_GET_CERT,
	SSP_FLASH_OEM,
	SSP_DONE,
	SSP_ERROR
} ssp_result_e;

struct otp_val {
	uint32_t idx;
	uint32_t nb;
};

static struct otp_val otp_ssp;
static struct otp_val otp_rma;
static struct otp_val otp_pubkey;

static int initialize_otp(void)
{
	uint32_t len;

	/* OTP SSP */
	if (stm32_get_otp_index(SSP_OTP, &otp_ssp.idx, NULL) != 0) {
		VERBOSE("%s: get index error\n", __func__);
		return -EINVAL;
	}

	/* OTP public key */
	if (stm32_get_otp_index(PKH_OTP, &(otp_pubkey.idx), &len) != 0) {
		VERBOSE("%s: get index error\n", __func__);
		return -EINVAL;
	}

	if (len != (CHAR_BIT * BOOT_API_SHA256_DIGEST_SIZE_IN_BYTES)) {
		VERBOSE("%s: length Error\n", __func__);
		return -EINVAL;
	}

	otp_pubkey.nb = len / (CHAR_BIT * sizeof(uint32_t));

	/* OTP RMA */
	if (stm32_get_otp_index(RMA_OTP, &otp_rma.idx, NULL) != 0) {
		VERBOSE("%s: get index error\n", __func__);
		return -EINVAL;
	}

	return 0;
}

/*
 * Return device handler for flashloader interface.
 */
static int ssp_get_loader_handle(uintptr_t *dev_handle, uintptr_t *dev_spec)
{
	int result;

	/* Obtain a reference to the image handle to get boot device */
	result = plat_get_image_source(BL33_IMAGE_ID, dev_handle, dev_spec);
	if (result != 0) {
		WARN("Failed to obtain reference to image '%s' (%i)\n",
		     BL33_IMAGE_NAME, result);
		return -EINVAL;
	}

	return result;
}

/*
 * Compute HASH from public key and burn it in OTP.
 */
static int ssp_pub_key_prog(boot_api_context_t *boot_context)
{
	uint8_t key_hash[BOOT_API_SHA256_DIGEST_SIZE_IN_BYTES];
	uint8_t *pubk = (uint8_t *)
		boot_context->p_ssp_config->p_blob_payload->oem_ecdsa_pubk;
	uint32_t *value = (uint32_t *)key_hash;
	uint32_t i;

	if (stm32_hash_register() != 0) {
		return -EINVAL;
	}

	stm32_hash_init(HASH_SHA256);

	if (stm32_hash_final_update(pubk, BOOT_API_SSP_PUBK_KEY_SIZE_BYTES,
				    key_hash) != 0) {
		ERROR("Hash of payload failed\n");
		return -EINVAL;
	}

	for (i = otp_pubkey.idx; i < (otp_pubkey.idx + otp_pubkey.nb); i++) {
		if (bsec_program_otp(bswap32(*value), i) != BSEC_OK) {
			return -EINVAL;
		}

		value++;
		if (bsec_permanent_lock_otp(i) != BSEC_OK) {
			ERROR("Error locking OTP %i\n", i);
			panic();
		}
	}

	return 0;
}

/*
 * Burn OTP to close device.
 */
static int ssp_close_device(void)
{
	uint32_t otp;
	uint32_t value;

	if (stm32_get_otp_index(CFG0_OTP, &otp, NULL) != 0) {
		return -EINVAL;
	}

	if (bsec_read_otp(&value, otp) != BSEC_OK) {
		return -EINVAL;
	}

	if ((value & CFG0_CLOSED_DEVICE) != 0U) {
		ERROR("Device already closed\n");
		return -EINVAL;
	}

	value |= CFG0_CLOSED_DEVICE;
	if (bsec_program_otp(value, otp) != BSEC_OK) {
		return -EINVAL;
	}

	return 0;
}

/*
 * OTP initial check to detect previous values.
 */
static int ssp_secrets_check(boot_api_context_t *boot_ctx)
{
	uint32_t i;
	uint32_t check_val;
	uint32_t otp_byte =
		boot_ctx->p_ssp_config->p_blob_payload->oem_secret_size_bytes;
	uint32_t otp_decrypted = round_up(otp_byte, sizeof(uint32_t)) /
				 sizeof(uint32_t);

	for (i = otp_pubkey.idx; i < (otp_pubkey.idx + otp_pubkey.nb); i++) {
		if (stm32_get_otp_value_from_idx(i, &check_val) != 0) {
			return -EINVAL;
		}

		if (check_val != 0U) {
			ERROR("OTP %d value already programmed\n", i);
			return -EINVAL;
		}
	}

	/* OTP decrypted include RMA password */
	if (otp_decrypted > (2U + SSP_OTP_SECRET_END - SSP_OTP_SECRET_BASE)) {
		return -EINVAL;
	}

	/* Check RMA password */
	if (stm32_get_otp_value_from_idx(otp_rma.idx, &check_val) != 0) {
		return -EINVAL;
	}

	if (check_val != 0U) {
		ERROR("OTP %s value already programmed\n", RMA_OTP);
		return -EINVAL;
	}

	/* Check all OTP available */
	for (i = SSP_OTP_SECRET_BASE;
	     i < SSP_OTP_SECRET_BASE + otp_decrypted - 1; i++) {
		if (stm32_get_otp_value_from_idx(i, &check_val) != 0) {
			return -EINVAL;
		}

		if (check_val != 0U) {
			ERROR("OTP %d value already programmed\n", i);
			return -EINVAL;
		}
	}

	return 0;
}

/*
 * Burn OTP with the decrypted secret received.
 */
static int ssp_secrets_flash(boot_api_context_t *boot_ctx)
{
	uint32_t i;
	uint32_t *val;
	uint32_t otp_byte =
		boot_ctx->p_ssp_config->p_blob_payload->oem_secret_size_bytes;
	uint32_t otp_decrypted = round_up(otp_byte, sizeof(uint32_t)) /
				 sizeof(uint32_t);
	uint32_t otp_mask = 0U;

	if (otp_byte % sizeof(uint32_t) != 0U) {
		otp_mask = GENMASK_32(((otp_byte % sizeof(uint32_t)) *
				       sizeof(uint32_t)) - 1, 0);
	}

	val = (uint32_t *)boot_ctx->p_ssp_config->p_ssp_oem_secrets_decrypted;

	/* Burn RMA password */
	if (otp_decrypted != 0U) {
		if (bsec_program_otp((*val & RMA_OTP_MASK), otp_rma.idx) !=
		    BSEC_OK) {
			WARN("RMA programing failed\n");
			return -EINVAL;
		}

		val++;
		otp_decrypted--;

		for (i = SSP_OTP_SECRET_BASE;
		     i < (SSP_OTP_SECRET_BASE + otp_decrypted - 1); i++) {

			if (*val == 0U) {
				val++;
				continue;
			}

			if (bsec_program_otp(*val, i) != BSEC_OK) {
				WARN("Error writing OTP %i\n", i);
				return -EINVAL;
			}

			if (bsec_permanent_lock_otp(i) != BSEC_OK) {
				WARN("Error locking OTP %i\n", i);
				return -EINVAL;
			}

			val++;
		}

		if (*val != 0U) {
			/* Mask the last OTP value if needed */
			if (otp_mask != 0U) {
				*val &= otp_mask;
			}

			if (bsec_program_otp(*val, i) != BSEC_OK) {
				WARN("Error writing OTP %i\n", i);
				return -EINVAL;
			}

			if (bsec_permanent_lock_otp(i) != BSEC_OK) {
				WARN("Error locking OTP %i\n", i);
				return -EINVAL;
			}
		}
	}

	return 0;
}

/*
 * Finish SSP processing by fusing OTP SSP success.
 */
static int ssp_finish_process(void)
{
	uint32_t val;

	if (stm32_get_otp_value_from_idx(otp_ssp.idx, &val) != 0) {
		return -EINVAL;
	}

	if ((val & SSP_OTP_SUCCESS) != 0U) {
		WARN("Error while configuring OTP\n");
		return -EINVAL;
	}

	val |= SSP_OTP_SUCCESS;
	if (bsec_program_otp(val, otp_ssp.idx) != BSEC_OK) {
		return -EINVAL;
	}

	VERBOSE("Write OTP Success\n");

	return 0;
}

/*
 * Transform integer to string.
 */
static void itoa(uint32_t num, char *str, int nb)
{
	if (num == 0U) {
		while (nb--) {
			str[nb] = '0';
		}
		return;
	}

	while (num != 0U) {
		int rem = num % 16;

		str[--nb] = (rem > 9) ? (rem - 10) + 'A' : rem + '0';
		num /= 16;
	}

	while (nb != 0) {
		str[--nb] = '0';
	}
}

/*
 * Return chip product ID.
 */
static int ssp_get_product_id(char *msg)
{
	uint32_t otp;
	uint32_t otp_idx;
	uint32_t chip_id;

	if (stm32_get_otp_index(CFG2_OTP, &otp_idx, NULL) != 0) {
		VERBOSE("Get index error\n");
		return -EINVAL;
	}

	if (stm32_get_otp_value_from_idx(otp_idx, &otp) != 0) {
		return -EINVAL;
	}

	if (stm32mp1_dbgmcu_get_chip_dev_id(&chip_id) < 0) {
		return -EINVAL;
	}

	itoa(chip_id, msg, 3);
	itoa((otp & OTP_CFG2_SEC_COUNTER_MASK) >>
	     OTP_CFG2_SEC_COUNTER_SHIFT, msg + 3, 2);

	itoa(0, msg + 5, 1);
	itoa((otp & OTP_CFG2_ST_KEY_MASK) >>
	     OTP_CFG2_ST_KEY_SHIFT, msg + 6, 2);

	return 0;
}

/*
 * Clean external data and bootrom context secret values.
 */
static void ssp_cleanup(boot_api_context_t *boot_context)
{
	boot_api_ssp_config_t *ssp_config = boot_context->p_ssp_config;

	/* Cleanup boot_context */
	if (ssp_config->p_ssp_oem_secrets_decrypted != NULL) {
		zeromem(ssp_config->p_ssp_oem_secrets_decrypted,
			BOOT_API_SSP_OEM_SECRETS_MAX_SIZE_BYTES);
		zeromem(ssp_config->p_chip_pubk,
			BOOT_API_SSP_PUBK_KEY_SIZE_BYTES);
		zeromem(ssp_config->p_blob_license,
			sizeof(boot_api_ssp_blob_license_t));
		zeromem(ssp_config->p_blob_payload,
			sizeof(boot_api_ssp_blob_payload_t));
	}

	ssp_config->ssp_cmd = 0U;

#ifndef DCACHE_OFF
	flush_dcache_range((uintptr_t)ssp_config->p_ssp_oem_secrets_decrypted,
			   BOOT_API_SSP_OEM_SECRETS_MAX_SIZE_BYTES);

	flush_dcache_range((uintptr_t)ssp_config->p_chip_pubk,
			   BOOT_API_SSP_PUBK_KEY_SIZE_BYTES);

	flush_dcache_range((uintptr_t)ssp_config->p_blob_license,
			   sizeof(boot_api_ssp_blob_license_t));

	flush_dcache_range((uintptr_t)ssp_config->p_blob_payload,
			   sizeof(boot_api_ssp_blob_payload_t));
#endif

	ssp_config->p_ssp_oem_secrets_decrypted = NULL;
	ssp_config->p_chip_pubk = NULL;
	ssp_config->p_blob_license = NULL;
	ssp_config->p_blob_payload = NULL;

#ifndef DCACHE_OFF
	flush_dcache_range((uintptr_t)boot_context,
			   sizeof(boot_api_context_t));

	flush_dcache_range((uintptr_t)ssp_config,
			   sizeof(boot_api_ssp_config_t));
#endif
}

/*
 * Send certificate to the programmer and retrieve the associated
 * encrypted file.
 */
static int ssp_download_phase(boot_api_context_t *boot_ctx)
{
	uint32_t i;
	uint32_t j;
	uint32_t otp;
	uint32_t otp_idx;
	uint32_t otp_len;
	uint8_t *blob_file;
	int result = 0;
	uintptr_t dev_handle, handle, dev_spec;
	size_t length_read;
	struct stm32image_part_info partition_spec = {
		.name = "SSP"
	};

	blob_file = (uint8_t *)page_align(SSP_BLOB_FILE_MAX_ADDR -
					  sizeof(boot_api_ssp_blob_license_t) -
					  sizeof(boot_api_ssp_blob_payload_t),
					  DOWN);

	ssp_exchange_t flash_exch = {
		.blob = blob_file
	};

	/* Prepare the ROM Security constant */
	if (ssp_get_product_id((char *)flash_exch.msg) != 0) {
		return -EINVAL;
	}

	/* Prepare public key and certificate for flashloader */
	/* Read Public Key from boot_context */
	memcpy((uint8_t *)flash_exch.msg + 8,
	       boot_ctx->p_ssp_config->p_chip_pubk,
	       BOOT_API_SSP_PUBK_KEY_SIZE_BYTES);

	if (stm32_get_otp_index(CHIP_CERTIFICATE_OTP,
				&otp_idx, &otp_len) != 0) {
		VERBOSE("Get index error\n");
		return -EINVAL;
	}

	if (otp_len != (CHAR_BIT * CHIP_CERTIFICATE_MAX_SIZE)) {
		VERBOSE("Length Error\n");
		return -EINVAL;
	}

	otp_len /= (CHAR_BIT * sizeof(uint32_t));

	/* Read Certificat from OTP */
	for (i = otp_idx, j = 0; i < (otp_idx + otp_len); i++, j++) {
		stm32_get_otp_value_from_idx(i, &otp);
		flash_exch.msg[18 + j] = bswap32(otp);
	}

	if (ssp_get_loader_handle(&dev_handle, &dev_spec) != 0) {
		WARN("Error while retrieving handle\n");
		return -ENOENT;
	}

	result = io_open(dev_handle, (uintptr_t)&partition_spec, &handle);
	if (result != 0) {
		WARN("SSP io open error %i\n", result);
		return -EINVAL;
	}

	result = io_read(handle, (uintptr_t)&flash_exch,
			 sizeof(boot_api_ssp_blob_license_t) +
			 sizeof(boot_api_ssp_blob_payload_t),
			 &length_read);
	if (result != 0) {
		WARN("SSP read command error %i\n", result);
		return -EINVAL;
	}

	boot_ctx->p_ssp_config->p_blob_license =
		(boot_api_ssp_blob_license_t *)blob_file;

	/* Payload is concatene with license file */
	boot_ctx->p_ssp_config->p_blob_payload =
		(boot_api_ssp_blob_payload_t *)(blob_file +
		sizeof(boot_api_ssp_blob_license_t));

	/* Set return address for decrypted_secrets */
	boot_ctx->p_ssp_config->p_ssp_oem_secrets_decrypted =
		boot_ctx->p_ssp_config->p_blob_payload->oem_encrypted_secrets;

#ifndef DCACHE_OFF
	flush_dcache_range((uintptr_t)boot_ctx->p_ssp_config->p_blob_license,
			   sizeof(boot_api_ssp_blob_license_t));
	flush_dcache_range((uintptr_t)boot_ctx->p_ssp_config->p_blob_payload,
			   sizeof(boot_api_ssp_blob_payload_t));
#endif
	result = io_close(handle);

	return result;
}

/*
 * Burn decrypted secrets into OTP, clean memory and close the device.
 */
static int ssp_secret_programming(boot_api_context_t *boot_context)
{
	int result;

	result = ssp_secrets_check(boot_context);
	if (result != 0) {
		WARN("SSP ERROR checking OTP\n");
		goto clean;
	}

	result = ssp_pub_key_prog(boot_context);
	if (result != 0) {
		WARN("SSP ERROR writing HASH key\n");
		goto clean;
	}

	result = ssp_close_device();
	if (result != 0) {
		WARN("SSP close device failed\n");
		goto clean;
	}

	result = ssp_secrets_flash(boot_context);
	if (result != 0) {
		WARN("SSP Secret flash failed\n");
	}

clean:
	ssp_cleanup(boot_context);

	if (result != 0) {
		return result;
	}

	return ssp_finish_process();
}

/*
 * Enable the SSP processing.
 */
static int ssp_enable_processing(boot_api_context_t *boot_context)
{
	static const char buf_err[] = "Provisioning";
	uint32_t val;
	int result;
	uintptr_t dev_handle, handle, dev_spec;
	size_t length_read;
	struct stm32image_part_info partition_spec = {
		.name = "SSP_INIT"
	};

	if (stm32_get_otp_value_from_idx(otp_ssp.idx, &val) != 0) {
		return -EINVAL;
	}

	if (((val & SSP_OTP_MASK) == SSP_OTP_MASK) ||
	    ((val & SSP_OTP_MASK) == SSP_OTP_SUCCESS)) {
		return -EINVAL;
	}

	if ((val & SSP_OTP_MASK) == 0U) {
		if (bsec_program_otp(SSP_OTP_REQ, otp_ssp.idx) != BSEC_OK) {
			return -EINVAL;
		}
	}

	if (ssp_get_loader_handle(&dev_handle, &dev_spec) != 0) {
		WARN("Error while retrieving handle\n");
		return -ENOENT;
	}

	result = io_open(dev_handle, (uintptr_t)&partition_spec, &handle);
	if (result != 0) {
		WARN("SSP io open error  %i\n", result);
		return -EINVAL;
	}

	result = io_read(handle, (uintptr_t)&buf_err, sizeof(buf_err),
			 &length_read);
	if (result != 0) {
		WARN("SSP read command error %i\n", result);
		return -EINVAL;
	}

	result = io_close(handle);
	if (result != 0) {
		WARN("SSP io close error %i\n", result);
		return -EINVAL;
	}

	boot_context->p_ssp_config->ssp_cmd =
		BOOT_API_CTX_SSP_CMD_CALC_CHIP_PUBK;
#ifndef DCACHE_OFF
	flush_dcache_range((uintptr_t)boot_context->p_ssp_config,
			   sizeof(boot_api_ssp_config_t));
#endif
	return 0;
}

/*
 * Retrieve the current status of the SSP from bootrom context and OTP value.
 */
static ssp_result_e ssp_check_status(boot_api_context_t *boot_context)
{
	uint32_t otp;

	if (initialize_otp() < 0) {
		return SSP_ERROR;
	}

	if (stm32_get_otp_value_from_idx(otp_ssp.idx, &otp) != 0) {
		return SSP_ERROR;
	}

	if ((otp & SSP_OTP_REQ) == 0U) {
		return SSP_NONE;
	}

	if ((otp & SSP_OTP_SUCCESS) != 0U) {
		return SSP_DONE;
	}

	VERBOSE("Start Get ssp_cmd : %x\n",
		boot_context->p_ssp_config->ssp_cmd);

	switch (boot_context->p_ssp_config->ssp_cmd) {
	case BOOT_API_CTX_SSP_CMD_CALC_CHIP_PUBK_ACK:
		INFO("Detected start SSP Phase 2\n");
		return SSP_GET_CERT;
	case BOOT_API_CTX_SSP_CMD_PROV_SECRET_ACK:
		INFO("Detected start SSP Phase 3\n");
		return SSP_FLASH_OEM;
	default:
		return SSP_NONE;
	}
}

/*
 * Start the SSP processing.
 */
void ssp_start(boot_api_context_t *boot_context)
{
	int result;

	switch (ssp_check_status(boot_context)) {
	case SSP_GET_CERT:
		result = ssp_download_phase(boot_context);
		if (result != 0) {
			/*
			 * Download Phase failed, clean, reset
			 */
			ssp_cleanup(boot_context);

			ERROR("SSP_Error: Resetting target\n");
			goto out;
		}

		/* Process completed, go to Phase 3 */
		boot_context->p_ssp_config->ssp_cmd =
			BOOT_API_CTX_SSP_CMD_PROV_SECRET;
#ifndef DCACHE_OFF
		flush_dcache_range((uintptr_t)boot_context->p_ssp_config,
				   sizeof(boot_api_ssp_config_t));
#endif
		break;

	case SSP_FLASH_OEM:
		result = ssp_secret_programming(boot_context);
		if (result != 0) {
			ERROR("Error during provisionning\n");
			goto out;
		}

		NOTICE("Provisioning completed\n");
		goto out;

	case SSP_ERROR:
		/*
		 * Error during bootrom SSP processing
		 */
		ERROR("SSP_Error: Resetting target\n");
		goto out;

	case SSP_NONE:
	default:
		result = ssp_enable_processing(boot_context);
		if (result != 0) {
			ERROR("Start SSP Failed (%i)\n", result);
			goto out;
		}
	}

	if (result == 0) {
		/*
		 * Keep VDDCORE && VDD enabled if pmic used to generate
		 * the required MPSYSRST.
		 */
		if (dt_pmic_status() > 0) {
			const char *name;

			name = stm32mp_get_cpu_supply_name();
			if (name == NULL) {
				goto out;
			}

			if (stpmic1_regulator_mask_reset_set(name) != 0) {
				WARN("Failed to write %s reset mask\n", name);
			}

			name = stm32mp_get_vdd_supply_name();
			if (name == NULL) {
				goto out;
			}

			if (stpmic1_regulator_mask_reset_set(name) != 0) {
				WARN("Failed to write %s reset mask\n", name);
			}
		}
	}

out:
	stm32mp_plat_reset(plat_my_core_pos());
}
