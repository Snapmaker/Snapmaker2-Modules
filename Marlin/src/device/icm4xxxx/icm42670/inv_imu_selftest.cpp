/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2015-2015 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively "Software") is subject
 * to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws.
 *
 * InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from InvenSense is strictly prohibited.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE SOFTWARE IS
 * PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * INVENSENSE BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
 * DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * ________________________________________________________________________________________________________
 */

#include "inv_imu_selftest.h"
#include "inv_imu_extfunc.h"
#include "inv_imu_transport.h"

static int configure_selftest_parameters(struct inv_imu_device *s,
                                         const inv_imu_selftest_parameters_t st_params);

int inv_imu_run_selftest(struct inv_imu_device *s,
                         const inv_imu_selftest_parameters_t st_params,
                         inv_imu_selftest_output_t *st_output)
{
	int status = 0;
	uint8_t value;
	uint8_t data[2] = {0};
	uint8_t st_done = 0;
	int polling_timeout_ms = 1000;

	/* Disables Gyro/Accel sensors */
	status |= inv_imu_read_reg(s, PWR_MGMT0, 1, &value);
	value &= ~(PWR_MGMT0_ACCEL_MODE_MASK | PWR_MGMT0_GYRO_MODE_MASK);
	status |= inv_imu_write_reg(s, PWR_MGMT0, 1, &value);

	/* Enable RC oscillator */
	status |= inv_imu_switch_on_mclk(s);

	/* Clear DMP SRAM (1 ms wait included in `inv_imu_reset_dmp()`) */
	status |= inv_imu_reset_dmp(s, APEX_CONFIG0_DMP_MEM_RESET_APEX_ST_EN);
	/* Update `dmp_is_on` since APEX features will have to restart from scratch */
	s->dmp_is_on = 0;

	/* Load self-test data */
	status |= inv_imu_load_selftest_data(s);

	/* Set self-test parameters */
	status |= configure_selftest_parameters(s, st_params);

	/*
	 * Enable accel and/or gyro self-test.
	 * If both accel and gyro self-test are enabled,
	 * they should be set simultaneously in the same write access
	 */
	status |= inv_imu_read_reg(s, SELFTEST_MREG1, 1, &value);
	value &= ~SELFTEST_EN;
	value |= (uint8_t)st_params.st_control;
	status |= inv_imu_write_reg(s, SELFTEST_MREG1, 1, &value);

	/* Poll int_status_st_done bit */
	do {
		inv_imu_sleep_us(1000);
		status |= inv_imu_read_reg(s, INT_STATUS, 1, &st_done);
		st_done &= INT_STATUS_ST_INT_MASK;

		if (0 == --polling_timeout_ms)
			return (status | -1); /* Return error if timeout is reached */

	} while (   !st_done /* Exit if ST_DONE */
	         && !status  /* Or if error is detected */);

	/* Read self-test results */
	status |= inv_imu_read_reg(s, ST_STATUS1_MREG1, 2, &data[0]);
	st_output->accel_status = (data[0] & ST_STATUS1_ACCEL_ST_PASS_MASK) >> ST_STATUS1_ACCEL_ST_PASS_POS;
	st_output->ax_status = (data[0] & ST_STATUS1_AX_ST_PASS_MASK) >> ST_STATUS1_AX_ST_PASS_POS;
	st_output->ay_status = (data[0] & ST_STATUS1_AY_ST_PASS_MASK) >> ST_STATUS1_AY_ST_PASS_POS;
	st_output->az_status = (data[0] & ST_STATUS1_AZ_ST_PASS_MASK) >> ST_STATUS1_AZ_ST_PASS_POS;
	st_output->gyro_status = (data[1] & ST_STATUS2_GYRO_ST_PASS_MASK) >> ST_STATUS2_GYRO_ST_PASS_POS;
	st_output->gyro_status |= ((data[1] & ST_STATUS2_ST_INCOMPLETE_MASK) >> ST_STATUS2_ST_INCOMPLETE_POS) << 1;
	st_output->gx_status = (data[1] & ST_STATUS2_GX_ST_PASS_MASK) >> ST_STATUS2_GX_ST_PASS_POS;
	st_output->gy_status = (data[1] & ST_STATUS2_GY_ST_PASS_MASK) >> ST_STATUS2_GY_ST_PASS_POS;
	st_output->gz_status = (data[1] & ST_STATUS2_GZ_ST_PASS_MASK) >> ST_STATUS2_GZ_ST_PASS_POS;

	/* Disable self-test */
	status |= inv_imu_read_reg(s, SELFTEST_MREG1, 1, &value);
	value &= ~SELFTEST_EN;
	value |= (uint8_t)SELFTEST_DIS;
	status |= inv_imu_write_reg(s, SELFTEST_MREG1, 1, &value);

	/* Re-sync all clocking scheme with a dummy write */
	value = 0;
	status |= inv_imu_write_reg(s, WHO_AM_I, 1, &value);

	/* Reset FIFO because ST data may have been pushed to it */
	status |= inv_imu_reset_fifo(s);

	/* Restore idle bit */
	status |= inv_imu_switch_off_mclk(s);

	return status;
}

int inv_imu_init_selftest_parameters_struct(struct inv_imu_device *s,
                                            inv_imu_selftest_parameters_t *st_params)
{
	(void)s;
	st_params->st_num_samples = ST_CONFIG_16_SAMPLES;
	st_params->st_control     = (SELFTEST_ACCEL_GYRO_ST_EN_t)SELFTEST_EN;
	return 0;
}

int inv_imu_load_selftest_data(struct inv_imu_device *s)
{
	int status = 0;
	uint8_t value;

	/* Enable RC oscillator */
	status |= inv_imu_switch_on_mclk(s);

	/* Set up OTP controller to reload factory-trimmed self-test response into SRAM */
	status |= inv_imu_read_reg(s, OTP_CONFIG_MREG1, 1, &value);
	value &= ~OTP_CONFIG_OTP_COPY_MODE_MASK;
	value |= (uint8_t)OTP_CONFIG_OTP_COPY_ST_DATA;
	status |= inv_imu_write_reg(s, OTP_CONFIG_MREG1, 1, &value);

	/* Take the OTP macro out of power-down mode */
	status |= inv_imu_read_reg(s, OTP_CTRL7_MREG2, 1, &value);
	value &= ~OTP_CTRL7_OTP_PWR_DOWN_MASK;
	value |= (uint8_t)OTP_CTRL7_PWR_DOWN_DIS;
	status |= inv_imu_write_reg(s, OTP_CTRL7_MREG2, 1, &value);

	/* Wait for voltage generator to power on */
	inv_imu_sleep_us(100);

	/* Host should disable INT function first before kicking off OTP copy operation */

	/* Trigger OTP to reload data (this time in self-test mode) */
	status |= inv_imu_read_reg(s, OTP_CTRL7_MREG2, 1, &value);
	value &= ~OTP_CTRL7_OTP_RELOAD_MASK;
	value |= (uint8_t)OTP_CTRL7_OTP_RELOAD_EN;
	status |= inv_imu_write_reg(s, OTP_CTRL7_MREG2, 1, &value);

	/* Wait for OTP reload */
	inv_imu_sleep_us(20);

	/* Disable RC oscillator */
	status |= inv_imu_switch_off_mclk(s);

	return status;
}

static int configure_selftest_parameters(struct inv_imu_device *s,
                                         const inv_imu_selftest_parameters_t st_params)
{
	int status = 0;
	uint8_t value;

	/* Self-test configuration cannot be updated if it already running */
	status |= inv_imu_read_reg(s, SELFTEST_MREG1, 1, &value);
	if ((value & SELFTEST_EN) != SELFTEST_DIS)
		return INV_ERROR_UNEXPECTED;

	status |= inv_imu_read_reg(s, ST_CONFIG_MREG1, 1, &value);
	value &= ~((uint8_t)ST_CONFIG_ST_NUMBER_SAMPLE_MASK
	         | (uint8_t)ST_CONFIG_ACCEL_ST_LIM_MASK
	         | (uint8_t)ST_CONFIG_GYRO_ST_LIM_MASK);
	value |=  (uint8_t)st_params.st_num_samples
	        | (uint8_t)ST_CONFIG_ACCEL_ST_LIM_50
	        | (uint8_t)ST_CONFIG_GYRO_ST_LIM_50;
	status |= inv_imu_write_reg(s, ST_CONFIG_MREG1, 1, &value);

	return status;
}