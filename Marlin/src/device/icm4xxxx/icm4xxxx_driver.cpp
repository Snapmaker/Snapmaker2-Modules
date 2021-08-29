
#include "icm4xxxx_driver.h"
#include "../../HAL/std_library/inc/stm32f10x.h"
#include "icm4xxxx_delay.h"
#include <math.h>
#include "icm42670/system_interface.h"
#include "icm42670/inv_imu_extfunc.h"

/*
 * ICM mounting matrix
 * Coefficients are coded as Q30 integer
 */
static int32_t icm_mounting_matrix[9] = {  0,        -(1<<30),      0,
                                          (1<<30),     0,           0,
                                           0,          0,          (1<<30) };

ICM4xxxxDriver icm42670;

static uint8_t SPIReadWriteByte(const uint8_t TxData)
{
	uint8_t retry=0;
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) {
		retry++;
		if(retry > 400) return 0;
	}

	SPI_I2S_SendData(SPI1, TxData);
	retry = 0;

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) {
		retry++;
		if(retry > 400) return 0;
	}

	return SPI_I2S_ReceiveData(SPI1);
}

static uint8_t HAL_SPI_Transmit(const uint8_t *pData, uint16_t size) {
	while (size--) {
		SPIReadWriteByte(*pData);
		pData++;
	}

	return 0;
}

static uint8_t HAL_SPI_Receive(uint8_t *pData, uint16_t size) {
	while (size--) {
		*pData = SPIReadWriteByte(0xff);
		pData++;
	}

	return 0;
}

unsigned char MPU_Write_Len(unsigned char addr,unsigned char reg,unsigned char len, const unsigned char *buf)
{
	uint8_t status;
	ICM4xxxx_ENABLE();
	status = HAL_SPI_Transmit(&reg, 1);
	status = HAL_SPI_Transmit(buf, len);
	ICM4xxxx_DISABLE();

	return status;
}

unsigned char MPU_Read_Len(unsigned char addr,unsigned char reg,unsigned char len,unsigned char *buf)
{
	uint8_t status;
	ICM4xxxx_ENABLE();
	reg = reg|0x80;
	status=HAL_SPI_Transmit(&reg, 1);
 	status=HAL_SPI_Receive(buf, len);
	ICM4xxxx_DISABLE();
	return status;
}

// static uint8_t ICM4xxxx_Write_Reg(uint8_t reg,uint8_t value)
// {
// 	uint8_t status;
// 	ICM4xxxx_ENABLE();
// 	status = HAL_SPI_Transmit(&reg, 1);
// 	status = HAL_SPI_Transmit(&value, 1);
// 	ICM4xxxx_DISABLE();
// 	return(status);
// }

static uint8_t ICM4xxxx_Read_Reg(uint8_t reg)
{
	uint8_t reg_val;
	ICM4xxxx_ENABLE();
	reg = reg|0x80;
	HAL_SPI_Transmit(&reg, 1);
 	HAL_SPI_Receive(&reg_val, 1);
	ICM4xxxx_DISABLE();
	return(reg_val);
}

void ICM4xxxxDriver::SPIInit() {
 	GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;

	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_SPI1,  ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOA, &GPIO_InitStructure);
 	GPIO_SetBits(GPIOA, GPIO_Pin_4);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

 	GPIO_ResetBits(GPIOA, GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7);

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	// SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	// SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 10;
	SPI_Init(SPI1, &SPI_InitStructure);

	SPI_Cmd(SPI1, ENABLE);

	// SPIReadWriteByte(0xff);
}
int ICM4xxxxDriver::setup_mcu(struct inv_imu_serif *icm_serif) {

	int rc = 0;

	/* Initialize serial interface between MCU and IMU */
	icm_serif->context   = 0;        /* no need */
	icm_serif->read_reg  = inv_io_hal_read_reg;
	icm_serif->write_reg = inv_io_hal_write_reg;
	icm_serif->max_read  = 2*32;  /* maximum number of bytes allowed per serial read */
	icm_serif->max_write = 2*32;  /* maximum number of bytes allowed per serial write */
	icm_serif->serif_type = SERIF_TYPE;

	return rc;
}

/* --------------------------------------------------------------------------------------
 *  Static functions definition
 * -------------------------------------------------------------------------------------- */

static void apply_mounting_matrix(const int32_t matrix[9], int32_t raw[3]) {
	unsigned i;
	int64_t data_q30[3];

	for(i = 0; i < 3; i++) {
		data_q30[i] =  ((int64_t)matrix[3*i+0] * raw[0]);
		data_q30[i] += ((int64_t)matrix[3*i+1] * raw[1]);
		data_q30[i] += ((int64_t)matrix[3*i+2] * raw[2]);
	}
	raw[0] = (int32_t)(data_q30[0]>>30);
	raw[1] = (int32_t)(data_q30[1]>>30);
	raw[2] = (int32_t)(data_q30[2]>>30);
}

static void get_accel_and_gyr_fsr(uint16_t * accel_fsr_g, uint16_t * gyro_fsr_dps) {
	ACCEL_CONFIG0_FS_SEL_t accel_fsr_bitfield;
	GYRO_CONFIG0_FS_SEL_t gyro_fsr_bitfield;

	inv_imu_get_accel_fsr(&icm42670.icm_driver_, &accel_fsr_bitfield);
	switch(accel_fsr_bitfield) {
	case ACCEL_CONFIG0_FS_SEL_2g:   *accel_fsr_g = 2;
	break;
	case ACCEL_CONFIG0_FS_SEL_4g:   *accel_fsr_g = 4;
	break;
	case ACCEL_CONFIG0_FS_SEL_8g:   *accel_fsr_g = 8;
	break;
	case ACCEL_CONFIG0_FS_SEL_16g:  *accel_fsr_g = 16;
	break;
	default:                                 *accel_fsr_g = -1;
	}

	inv_imu_get_gyro_fsr(&icm42670.icm_driver_, &gyro_fsr_bitfield);
	switch(gyro_fsr_bitfield) {
	case GYRO_CONFIG0_FS_SEL_250dps:  *gyro_fsr_dps = 250;
	break;
	case GYRO_CONFIG0_FS_SEL_500dps:  *gyro_fsr_dps = 500;
	break;
	case GYRO_CONFIG0_FS_SEL_1000dps: *gyro_fsr_dps = 1000;
	break;
	case GYRO_CONFIG0_FS_SEL_2000dps: *gyro_fsr_dps = 2000;
	break;
	default:                                   *gyro_fsr_dps = -1;
	}
}

static void imu_callback(struct inv_imu_device *s, inv_imu_sensor_event_t *event) {
	uint64_t timestamp;
	int32_t accel[3], gyro[3];
#if SCALED_DATA_G_DPS
	float accel_g[3];
	float gyro_dps[3];
	float temp_degc;
	uint16_t accel_fsr_g, gyro_fsr_dps;
#endif

#if USE_FIFO
	static uint64_t last_fifo_timestamp = 0;
	static uint32_t rollover_num = 0;

	// Handle rollover
	if (last_fifo_timestamp > event->timestamp_fsync)
		rollover_num++;
	last_fifo_timestamp = event->timestamp_fsync;

	// Compute timestamp in us
	timestamp = event->timestamp_fsync + rollover_num * UINT16_MAX;
	timestamp *= inv_imu_get_fifo_timestamp_resolution_us_q24(&icm42670.icm_driver_);
	timestamp /= (1UL << 24);

	if (icm42670.icm_driver_.fifo_highres_enabled) {
		accel[0] = (((int32_t)event->accel[0] << 4)) | event->accel_high_res[0];
		accel[1] = (((int32_t)event->accel[1] << 4)) | event->accel_high_res[1];
		accel[2] = (((int32_t)event->accel[2] << 4)) | event->accel_high_res[2];

		gyro[0] = (((int32_t)event->gyro[0] << 4)) | event->gyro_high_res[0];
		gyro[1] = (((int32_t)event->gyro[1] << 4)) | event->gyro_high_res[1];
		gyro[2] = (((int32_t)event->gyro[2] << 4)) | event->gyro_high_res[2];

	} else {
		accel[0] = event->accel[0];
		accel[1] = event->accel[1];
		accel[2] = event->accel[2];

		gyro[0] = event->gyro[0];
		gyro[1] = event->gyro[1];
		gyro[2] = event->gyro[2];
	}
#else
	inv_disable_irq();
	if (!RINGBUFFER_EMPTY(&timestamp_buffer))
		RINGBUFFER_POP(&timestamp_buffer, &timestamp);
	inv_enable_irq();

	accel[0] = event->accel[0];
	accel[1] = event->accel[1];
	accel[2] = event->accel[2];

	gyro[0] = event->gyro[0];
	gyro[1] = event->gyro[1];
	gyro[2] = event->gyro[2];

	// Force sensor_mask so it gets displayed below
	event->sensor_mask |= (1 << INV_SENSOR_TEMPERATURE);
	event->sensor_mask |= (1 << INV_SENSOR_ACCEL);
	event->sensor_mask |= (1 << INV_SENSOR_GYRO);
#endif

	// apply_mounting_matrix(icm_mounting_matrix, accel);
	// apply_mounting_matrix(icm_mounting_matrix, gyro);

#if SCALED_DATA_G_DPS
	/*
	 * Convert raw data into scaled data in g and dps
	*/
	get_accel_and_gyr_fsr(&accel_fsr_g, &gyro_fsr_dps);
	accel_g[0]  = (float)(accel[0] * accel_fsr_g)  / INT16_MAX;
	accel_g[1]  = (float)(accel[1] * accel_fsr_g)  / INT16_MAX;
	accel_g[2]  = (float)(accel[2] * accel_fsr_g)  / INT16_MAX;
	gyro_dps[0] = (float)(gyro[0]  * gyro_fsr_dps) / INT16_MAX;
	gyro_dps[1] = (float)(gyro[1]  * gyro_fsr_dps) / INT16_MAX;
	gyro_dps[2] = (float)(gyro[2]  * gyro_fsr_dps) / INT16_MAX;


	if (USE_HIGH_RES_MODE || !USE_FIFO)
		temp_degc = 25 + ((float)event->temperature / 128);
	else
		temp_degc = 25 + ((float)event->temperature / 2);

	s->accel_x_f = accel_g[0];
	s->accel_y_f = accel_g[1];
	s->accel_z_f = accel_g[2];

	s->gyro_x_f  = gyro_dps[0];
	s->gyro_y_f  = gyro_dps[1];
	s->gyro_z_f  = gyro_dps[2];

#endif

	s->accel_x = accel[0];
	s->accel_y = accel[1];
	s->accel_z = accel[2];

	s->gyro_x  = gyro[0];
	s->gyro_y  = gyro[1];
	s->gyro_z  = gyro[2];

	s->data_is_ready = true;
}

int ICM4xxxxDriver::setup_imu_device(struct inv_imu_serif *icm_serif) {
	int rc = 0;
	uint8_t who_am_i;

	/* Init device */
	rc = inv_imu_init(&icm_driver_, icm_serif, imu_callback);
	if (rc != INV_ERROR_SUCCESS) {
		return rc;
	}

	/* Check WHOAMI */
	rc = inv_imu_get_who_am_i(&icm_driver_, &who_am_i);
	if (rc != INV_ERROR_SUCCESS) {
		return rc;
	}

	if (who_am_i != ICM_WHOAMI) {

		return INV_ERROR;
	}

	return rc;
}

int ICM4xxxxDriver::configure_imu_device() {
	int rc = 0;

	if (!USE_FIFO)
		rc |= inv_imu_configure_fifo(&icm_driver_, INV_IMU_FIFO_DISABLED);

	if (USE_HIGH_RES_MODE) {
		rc |= inv_imu_enable_high_resolution_fifo(&icm_driver_);
	} else {
		rc |= inv_imu_set_accel_fsr(&icm_driver_, ACCEL_CONFIG0_FS_SEL_4g);
		rc |= inv_imu_set_gyro_fsr(&icm_driver_, GYRO_CONFIG0_FS_SEL_2000dps);
	}

	if (USE_LOW_NOISE_MODE) {
		rc |= inv_imu_set_accel_frequency(&icm_driver_, ACCEL_CONFIG0_ODR_50_HZ);
		rc |= inv_imu_set_gyro_frequency(&icm_driver_, GYRO_CONFIG0_ODR_50_HZ);
		rc |= inv_imu_enable_accel_low_noise_mode(&icm_driver_);
	} else {
		rc |= inv_imu_set_accel_frequency(&icm_driver_, ACCEL_CONFIG0_ODR_100_HZ);
		rc |= inv_imu_set_gyro_frequency(&icm_driver_, GYRO_CONFIG0_ODR_100_HZ);
		rc |= inv_imu_enable_accel_low_power_mode(&icm_driver_);
	}

	rc |= inv_imu_enable_gyro_low_noise_mode(&icm_driver_);

	if (!USE_FIFO)
		inv_imu_sleep_us(GYR_STARTUP_TIME_US);

	return rc;
}

bool ICM4xxxxDriver::get_imu_data(void) {
	icm_driver_.data_is_ready = false;
#if USE_FIFO
	inv_imu_get_data_from_fifo(&icm_driver_);
#else
	inv_imu_get_data_from_registers(&icm_driver);
#endif

	if (icm_driver_.data_is_ready) {
		accel_x_raw_ = icm_driver_.accel_x;
		accel_y_raw_ = icm_driver_.accel_y;
		accel_z_raw_ = icm_driver_.accel_z;

		gyro_x_raw_ = icm_driver_.gyro_x;
		gyro_y_raw_ = icm_driver_.gyro_y;
		gyro_z_raw_ = icm_driver_.gyro_z;
		return true;
	} else {
		return false;
	}
}

bool ICM4xxxxDriver::ChipInit() {
	SPIInit();

	// Based on datasheet, start up time for register read/write after POR is 1ms and supply ramp time is 3ms
	icm4xxxx_delay_ms(3);

	// chip identification
	who_am_i_ = ICM4xxxx_Read_Reg(REG_ICM42670P_WHO_AM_I);

	if (who_am_i_ == WHO_AM_I_ICM42670P) {
		setup_mcu(&icm_serif_);
		setup_imu_device(&icm_serif_);
		configure_imu_device();
		return false;
	}

	// discard the first 20 data
	bool ret;
	int32_t i = 0;

	do {
		ret = GetRawDataFromFIFO();
		if (ret == true) {
			i++;
		}
	}while(i < 20);
	attitude_solving_stage = ATTITUDE_SOLVING_PREPARE;
	return true;
}

bool ICM4xxxxDriver::GetRawDataFromFIFO() {
	return get_imu_data();
}

bool ICM4xxxxDriver::AttitudeSolving() {
	bool ret;
	bool gesture_solved = false;

	if (attitude_solving_stage == ATTITUDE_SOLVING_PREPARE) {
		do {
			ret = GetRawDataFromFIFO();
		}	while (ret);

		do {
			ret = GetRawDataFromFIFO();
			if (ret) {
				accel_x_raw_buff_[sliding_window_index_] = accel_x_raw_;
				accel_y_raw_buff_[sliding_window_index_] = accel_y_raw_;
				accel_z_raw_buff_[sliding_window_index_] = accel_z_raw_;
				gyro_x_raw_buff_[sliding_window_index_]  = gyro_x_raw_;
				gyro_y_raw_buff_[sliding_window_index_]  = gyro_y_raw_;
				gyro_z_raw_buff_[sliding_window_index_]  = gyro_z_raw_;
				sliding_window_index_++;
			}
		} while(sliding_window_index_ < SLIDING_WINDOW_SIZE);

		accel_x_raw_acc_ = 0;
		accel_y_raw_acc_ = 0;
		accel_z_raw_acc_ = 0;
		gyro_x_raw_acc_  = 0;
		gyro_y_raw_acc_  = 0;
		gyro_z_raw_acc_  = 0;
		for (int32_t i = 0; i < SLIDING_WINDOW_SIZE; i++) {
			accel_x_raw_acc_ += accel_x_raw_buff_[i];
			accel_y_raw_acc_ += accel_y_raw_buff_[i];
			accel_z_raw_acc_ += accel_z_raw_buff_[i];
			gyro_x_raw_acc_  += gyro_x_raw_buff_[i];
			gyro_y_raw_acc_  += gyro_y_raw_buff_[i];
			gyro_z_raw_acc_  += gyro_z_raw_buff_[i];
		}

		attitude_solving_stage = ATTITUDE_SOLVING_DOING;

		gesture_solved = false;
	}
	else if (attitude_solving_stage == ATTITUDE_SOLVING_DOING) {
		// sliding window filtering
		do {
			ret = GetRawDataFromFIFO();
			if (ret) {
				if (sliding_window_index_ == SLIDING_WINDOW_SIZE) {
					sliding_window_index_ = 0;
				}

				accel_x_raw_acc_ -= accel_x_raw_buff_[sliding_window_index_];
				accel_y_raw_acc_ -= accel_y_raw_buff_[sliding_window_index_];
				accel_z_raw_acc_ -= accel_z_raw_buff_[sliding_window_index_];
				gyro_x_raw_acc_  -= gyro_x_raw_buff_[sliding_window_index_];
				gyro_y_raw_acc_  -= gyro_y_raw_buff_[sliding_window_index_];
				gyro_z_raw_acc_  -= gyro_z_raw_buff_[sliding_window_index_];

				accel_x_raw_buff_[sliding_window_index_] = accel_x_raw_;
				accel_y_raw_buff_[sliding_window_index_] = accel_y_raw_;
				accel_z_raw_buff_[sliding_window_index_] = accel_z_raw_;
				gyro_x_raw_buff_[sliding_window_index_]  = gyro_x_raw_;
				gyro_y_raw_buff_[sliding_window_index_]  = gyro_y_raw_;
				gyro_z_raw_buff_[sliding_window_index_]  = gyro_z_raw_;

				accel_x_raw_acc_ += accel_x_raw_buff_[sliding_window_index_];
				accel_y_raw_acc_ += accel_y_raw_buff_[sliding_window_index_];
				accel_z_raw_acc_ += accel_z_raw_buff_[sliding_window_index_];
				gyro_x_raw_acc_  += gyro_x_raw_buff_[sliding_window_index_];
				gyro_y_raw_acc_  += gyro_y_raw_buff_[sliding_window_index_];
				gyro_z_raw_acc_  += gyro_z_raw_buff_[sliding_window_index_];

				sliding_window_index_++;

				accel_x_raw_filter_ = accel_x_raw_acc_ / SLIDING_WINDOW_SIZE;
				accel_y_raw_filter_ = accel_y_raw_acc_ / SLIDING_WINDOW_SIZE;
				accel_z_raw_filter_ = accel_z_raw_acc_ / SLIDING_WINDOW_SIZE;
				gyro_x_raw_filter_  = gyro_x_raw_acc_ / SLIDING_WINDOW_SIZE;
				gyro_y_raw_filter_  = gyro_y_raw_acc_ / SLIDING_WINDOW_SIZE;
				gyro_z_raw_filter_  = gyro_z_raw_acc_ / SLIDING_WINDOW_SIZE;

				int32_t accel[3], gyro[3];
				accel[0] = accel_x_raw_filter_;
				accel[1] = accel_y_raw_filter_;
				accel[2] = accel_z_raw_filter_;
				gyro[0]  = gyro_x_raw_filter_;
				gyro[1]  = gyro_y_raw_filter_;
				gyro[2]  = gyro_z_raw_filter_;

				apply_mounting_matrix(icm_mounting_matrix, accel);
				apply_mounting_matrix(icm_mounting_matrix, gyro);

				uint16_t accel_fsr_g, gyro_fsr_dps;
				get_accel_and_gyr_fsr(&accel_fsr_g, &gyro_fsr_dps);

				float accel_x, accel_y, accel_z;
				float gyro_x, gyro_y, gyro_z;
				accel_x  = (float)(accel[0] * accel_fsr_g)  / INT16_MAX;
				accel_y  = (float)(accel[1] * accel_fsr_g)  / INT16_MAX;
				accel_z  = (float)(accel[2] * accel_fsr_g)  / INT16_MAX;
				gyro_x   = (float)(gyro[0]  * gyro_fsr_dps) / INT16_MAX;
				gyro_y   = (float)(gyro[1]  * gyro_fsr_dps) / INT16_MAX;
				gyro_z   = (float)(gyro[2]  * gyro_fsr_dps) / INT16_MAX;

				gyro_x = (PI / 180) * gyro_x;
				gyro_y = (PI / 180) * gyro_y;
				gyro_z = (PI / 180) * gyro_z;

				ImuUpdate(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);
				if (++gesture_preprocessing_count_ > 100) gesture_preprocessing_count_ = 101;

				gesture_solved = true;
			}
		}while(gesture_preprocessing_count_ <= 100);
	}

	return gesture_solved;
}

void ICM4xxxxDriver::ImuUpdate(float accel_x, float accel_y, float accel_z, float gyro_x, float gyro_y, float gyro_z) {
  float q0temp,q1temp,q2temp,q3temp;
  float norm;
  float vx, vy, vz;
  float ex, ey, ez;
  float q0q0 = q0_*q0_;
  float q0q1 = q0_*q1_;
  float q0q2 = q0_*q2_;
  float q1q1 = q1_*q1_;
  float q1q3 = q1_*q3_;
  float q2q2 = q2_*q2_;
  float q2q3 = q2_*q3_;
  float q3q3 = q3_*q3_;

  norm = sqrt(accel_x*accel_x + accel_y*accel_y + accel_z*accel_z);
  accel_x = accel_x /norm;
  accel_y = accel_y / norm;
  accel_z = accel_z / norm;

  vx = 2*(q1q3 - q0q2);
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;

  ex = (accel_y*vz - accel_z*vy) ;
  ey = (accel_z*vx - accel_x*vz) ;
  ez = (accel_x*vy - accel_y*vx) ;

  exInt_ = exInt_ + ex * Ki;
  eyInt_ = eyInt_ + ey * Ki;
  ezInt_ = ezInt_ + ez * Ki;

  gyro_x = gyro_x + Kp*ex + exInt_;
  gyro_y = gyro_y + Kp*ey + eyInt_;
  gyro_z = gyro_z + Kp*ez + ezInt_;

  q0temp = q0_;
  q1temp = q1_;
  q2temp = q2_;
  q3temp = q3_;

  q0_ = q0temp + (-q1temp*gyro_x - q2temp*gyro_y -q3temp*gyro_z)*halfT;
  q1_ = q1temp + (q0temp*gyro_x + q2temp*gyro_z -q3temp*gyro_y)*halfT;
  q2_ = q2temp + (q0temp*gyro_y - q1temp*gyro_z +q3temp*gyro_x)*halfT;
  q3_ = q3temp + (q0temp*gyro_z + q1temp*gyro_y -q2temp*gyro_x)*halfT;

  norm = sqrt(q0_*q0_ + q1_*q1_ + q2_*q2_ + q3_*q3_);
  q0_ = q0_ / norm;
  q1_ = q1_ / norm;
  q2_ = q2_ / norm;
  q3_ = q3_ / norm;

  roll_ = atan2(2 * q2_ * q3_ + 2 * q0_ * q1_, -2 * q1_ * q1_ - 2 * q2_* q2_ + 1)* 57.3;
	if (fabs(roll_) < 90.0) {
		pitch_ = asin(-2 * q1_ * q3_ + 2 * q0_* q2_) * 57.3;
	}
	else {
		pitch_ = (fabs(q0_ * q2_ - q1_ * q3_) / (q0_ * q2_ - q1_ * q3_)) * (180.0 - fabs(asin(-2 * q1_ * q3_ + 2 * q0_ * q2_) * 57.3));
	}
}

uint8_t ICM4xxxxDriver::GetGesture(float & yaw, float & pitch, float & roll) {
	yaw = yaw_;
	pitch = pitch_;
	roll = roll_;

	return 1;
}