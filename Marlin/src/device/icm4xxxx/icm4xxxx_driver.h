#ifndef SNAPMAKER_MODULES_ICM20689_H_
#define SNAPMAKER_MODULES_ICM20689_H_

#include <stdint.h>
#include "../device_base.h"
#include "icm42670/inv_imu_transport.h"
#include "icm42670/inv_imu_driver.h"

#define ICM4xxxx_ENABLE() {GPIO_ResetBits(GPIOA, GPIO_Pin_4);}
#define ICM4xxxx_DISABLE() {GPIO_SetBits(GPIOA, GPIO_Pin_4);}

#define WHO_AM_I_ICM42605     0x42
#define WHO_AM_I_ICM42670P    0x67
#define FIFO_PACKET_SIZE      16

#define PI                    3.1415926f
#define LSB_GYRO_FS_2000      16.4f
#define LSB_GYRO_FS_1000      32.8f
#define LSB_GYRO_FS_500       65.5
#define LSB_GYRO_FS_250       131f
#define LSB_GYRO_FS_125       262f

#define LSB_ACCEL_FS_16       2048
#define LSB_ACCEL_FS_8        4096
#define LSB_ACCEL_FS_4        8192
#define LSB_ACCEL_FS_2        16384

/*
 * MPUREG_INT_STATUS
 * Register Name: INT_STATUS
 */
#define BIT_INT_STATUS_UI_FSYNC   0x40
#define BIT_INT_STATUS_PLL_RDY    0x20
#define BIT_INT_STATUS_RESET_DONE 0x10
#define BIT_INT_STATUS_DRDY       0x08
#define BIT_INT_STATUS_FIFO_THS   0x04
#define BIT_INT_STATUS_FIFO_FULL  0x02
#define BIT_INT_STATUS_AGC_RDY    0x01


/*		ICM42670-P REGISTER MAP			*/
// MREG0
#define REG_ICM42670P_WHO_AM_I          0x75
#define REG_ICM42670P_DEVICE_CONFIG     0x01
#define REG_ICM42670P_SIGNAL_PATH_RESET 0x02
#define REG_ICM42670P_GYRO_CONFIG0      0x20
#define REG_ICM42670P_ACCEL_CONFIG0     0x21
#define REG_ICM42670P_FIFO_CONFIG1      0x28
#define REG_ICM42670P_INTF_CONFIG0      0x35
#define REG_ICM42670P_BLK_SEL_W         0x79
#define REG_ICM42670P_MADDR_W           0x7A
#define REG_ICM42670P_M_W               0x7B
#define REG_ICM42670P_BLK_SEL_R         0x7C
#define REG_ICM42670P_MADDR_R           0x7D
#define REG_ICM42670P_M_R               0x7E
#define REG_ICM42670P_PWR_MEMT0         0x1F
#define REG_ICM42670P_FIFO_COUNTH       0x3D
#define REG_ICM42670P_FIFO_COUNTL       0x3E
#define REG_ICM42670P_FIFO_DATA         0x3F
#define REG_ICM42670P_ACCEL_DATA_X1     0x0B
#define REG_ICM42670P_ACCEL_DATA_X0     0x0C
#define REG_ICM42670P_ACCEL_DATA_Y1     0x0D
#define REG_ICM42670P_ACCEL_DATA_Y0     0x0E
#define REG_ICM42670P_ACCEL_DATA_Z1     0x0F
#define REG_ICM42670P_ACCEL_DATA_Z0     0x10
#define REG_ICM42670P_GYRO_DATA_X1      0x11
#define REG_ICM42670P_GYRO_DATA_X0      0x12
#define REG_ICM42670P_GYRO_DATA_Y1      0x13
#define REG_ICM42670P_GYRO_DATA_Y0      0x14
#define REG_ICM42670P_GYRO_DATA_Z1      0x15
#define REG_ICM42670P_GYRO_DATA_Z0      0x16
#define REG_ICM42670P_MCLK_RDY          0x00

// MREG1
#define REG_ICM42670P_FIFO_CONFIG5      0x01
#define REG_ICM42670P_OTP_CONFIG        0x2B
// MREG2
#define REG_ICM42670P_OTP_CTRL7         0x06
// MREG3


#define SLIDING_WINDOW_SIZE   8

#define Kp 10.0f                        // 这里的KpKi是用于调整加速度计修正陀螺仪的速度

#define Ki 0.008f

#define halfT 0.01f                    // 采样周期的一半，用于求解四元数微分方程时计算角增量

/*
 * Select communication link between SmartMotion and IMU
 */
#define SERIF_TYPE           UI_SPI4
//#define SERIF_TYPE           UI_I2C

/*
 * Set power mode flag
 * Set this flag to run example in low-noise mode.
 * Reset this flag to run example in low-power mode.
 * Note: low-noise mode is not available with sensor data frequencies less than 12.5Hz.
 */
#define USE_LOW_NOISE_MODE   1

/*
 * Select Fifo resolution Mode (default is low resolution mode)
 * Low resolution mode: 16 bits data format
 * High resolution mode: 20 bits data format
 * Warning: Enabling High Res mode will force FSR to 16g and 2000dps
 */
#define USE_HIGH_RES_MODE    0

/*
 * Select to use FIFO or to read data from registers
 */
#define USE_FIFO             1

/*
 * Print raw data or scaled data
 * 0 : print raw accel, gyro and temp data
 * 1 : print scaled accel, gyro and temp data in g, dps and degree Celsius
 */
#define SCALED_DATA_G_DPS 0

typedef enum {
  ATTITUDE_SOLVING_PREPARE,
  ATTITUDE_SOLVING_DOING,
}ATTITUDE_SOLVING_STAGE_TYPE;

class ICM4xxxxDriver {
  public:
    ICM4xxxxDriver () {
      q0_ = 1;
      q1_ = 0;
      q2_ = 0;
      q3_ = 0;
      exInt_ = 0;
      eyInt_ = 0;
      ezInt_ = 0;
      gesture_preprocessing_count_ = 0;
      who_am_i_ = 0xff;
      imu_inited_ = false;
    }
    void SPIInit();
    int setup_mcu(struct inv_imu_serif *icm_serif);
    int setup_imu_device(struct inv_imu_serif *icm_serif);
    int configure_imu_device();
    bool get_imu_data(void);
    bool ChipInit();
    void GetRawDataFromDataReg();
    bool GetRawDataFromFIFO();
    bool AttitudeSolving();
    void ImuUpdate(float accel_x, float accel_y, float accel_z, float gyro_x, float gyro_y, float gyro_z);
    uint8_t GetGesture(float & yaw, float & pitch, float & roll);
    float GetTemperature();
  private:
    uint8_t who_am_i_;
    bool imu_inited_;
    int16_t accel_x_raw_;
    int16_t accel_y_raw_;
    int16_t accel_z_raw_;
    int16_t gyro_x_raw_;
    int16_t gyro_y_raw_;
    int16_t gyro_z_raw_;
    float imu_temperature_;
    int16_t accel_x_raw_buff_[SLIDING_WINDOW_SIZE];
    int16_t accel_y_raw_buff_[SLIDING_WINDOW_SIZE];
    int16_t accel_z_raw_buff_[SLIDING_WINDOW_SIZE];
    int16_t gyro_x_raw_buff_[SLIDING_WINDOW_SIZE];
    int16_t gyro_y_raw_buff_[SLIDING_WINDOW_SIZE];
    int16_t gyro_z_raw_buff_[SLIDING_WINDOW_SIZE];
    uint8_t sliding_window_index_;
    int32_t accel_x_raw_acc_;
    int32_t accel_y_raw_acc_;
    int32_t accel_z_raw_acc_;
    int32_t gyro_x_raw_acc_;
    int32_t gyro_y_raw_acc_;
    int32_t gyro_z_raw_acc_;
    int16_t accel_x_raw_filter_;
    int16_t accel_y_raw_filter_;
    int16_t accel_z_raw_filter_;
    int16_t gyro_x_raw_filter_;
    int16_t gyro_y_raw_filter_;
    int16_t gyro_z_raw_filter_;
    uint16_t gesture_preprocessing_count_;

    ATTITUDE_SOLVING_STAGE_TYPE attitude_solving_stage;
    float q0_, q1_, q2_, q3_;
    float exInt_, eyInt_, ezInt_;

    float yaw_;
    float pitch_;
    float roll_;
  public:
    struct inv_imu_serif icm_serif_;
    struct inv_imu_device icm_driver_;
};

extern ICM4xxxxDriver icm42670;

#endif