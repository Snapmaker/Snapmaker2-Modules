#ifndef MODULES_WHIMSYCWD_MARLIN_SRC_FEATURE_BLDC_H_
#define MODULES_WHIMSYCWD_MARLIN_SRC_FEATURE_BLDC_H_

#include <stdint.h>
#include "device_base.h"

#define USE_PID_CTRL_MOTOR_RPM 
// #define USE_PID_BREAK_MODE_MOTOR_RPM 
#define USE_PID_CTRL_SLOW_START 
#define PID_CTRL_MAX_CHANGE_POWER    (0.05)
#define BLDC_MOTOR_WORKING_VOLTAGE   (24.0)
#define BLDC_MOTOR_WORK_MAX_CURRENT  (6.0)
#define BLDC_MOTOR_BREAK_MAX_CURRENT (10.0)
#define BLDC_MOTOR_RESISTANCE        (24.0/35)
#define USE_PID_CTRL_KE_LIMIT        (24.0/19000)//(24.0/18000)
#define BLDC_SLOW_START_KEEP_TIME    (100) 

#define BLDC_MOTOR_STALL_SCALE_LIMIT  (0.5)

// Default PID parameters
#define  P_DEFAULT_DATA                   0.0001 //0.1  //0.14               
#define  I_DEFAULT_DATA                   0.01   //0.001                
#define  D_DEFAULT_DATA                   0            
            
#define  MOTOR_MIN_SPEED                  8000     
#define  MOTOR_RATED_SPEED                18000 
#define  MOTOR_BLOCK_SPEED                5500.0 
#define  MOTOR_POLE_PAIR_NUM              2  

#define MIN_DUTY_CYCLE                    0.07
#define MAX_DUTY_CYCLE                    1

#define BLDC_MOS_SELF_TEST_INTERVAL       50   // ms
#define BLDC_MOS_SELF_TEST_NEXT_INTERVAL  200   // ms
#define BLDC_MOS_SELF_TEST_DUTY_CYCLE     0.02 
#define BLDC_MOS_SELF_TEST_CURRENT_LIMIT  600   // mA

#define BLDC_HALL_SELF_TEST_INTERVAL      200
#define BLDC_HALL_SELF_TEST_DUTY_CYCLE    0.08 

#define CHECK_HALL_CNT                    (6)

// F103TB Only two types of supporters  
typedef enum {
  DEFAULT_REMAP = 0,
  PARTIAL_REMAP,
  FULL_REMAP
} ADVANCED_TIMER1_MAP;

//Direction of rotation of the motor
typedef enum {
  CW = 0,   //Clockwise rotation  
  CCW = 1   //Anticlockwise rotation
} MOTOR_DIR;

//Motor operation status
typedef enum {
  STOP = 0,  
  RUN,
  STOPING
} MOTOR_STATE;

typedef enum {
  MOTOR_BLOCK_NORMAL = 0,
  MOTOR_BLOCK_S_SPIN,
  MOTOR_BLOCK_H_PROTECT,
}MOTOR_BLOCK_STATE;

// Motor equipment
typedef struct {
  volatile float            motor_speed;           
  volatile uint32_t         step_counter;       
  volatile uint16_t         stalling_count;
  volatile uint8_t          pole_pair_num;      
  volatile MOTOR_DIR        motor_direction;
  volatile MOTOR_STATE      motor_state;        
} MOTOR_DEVICE;

typedef enum {
  V_ADC_CHANNEL = 0,
//  S_ADC_CHANNEL,
  M_ADC_CHANNEL,
  P_ADC_CHANNEL,
  I_ADC_CHANNEL,
  MAX_CHANNEL
}MultiChannel;

// PID
typedef struct {
  float SetPoint;      
  float Proportion;   
  float Integral;     
  float Derivative;  
  float PrevError;   
  float LastError; 
  bool  TargetRpmDown;  
  bool  ErrorLimit;    
} PID;

typedef enum {
  SELF_TEST_IDLE = 0,
  SELF_TEST_START,
  SELF_TESTING_MOS_PRE,
  SELF_TESTING_MOS_CHECKING,
  SELF_TESTING_MOS_POST,

  SELF_TESTING_HALL_PRE,
  SELF_TESTING_HALL_CHECKING,
  SELF_TESTING_HALL_POST,

  SELF_TEST_END,
} BLDC_SELF_TEST_STEP;

typedef enum {
  SELF_TEST_MOS_UH_INDEX = 0,
  SELF_TEST_MOS_VH_INDEX,
  SELF_TEST_MOS_WH_INDEX,
  SELF_TEST_MOS_UL_INDEX,
  SELF_TEST_MOS_VL_INDEX,
  SELF_TEST_MOS_WL_INDEX,
  
  SELF_TEST_MOS_INVALID_INDEX
} BLDC_SELF_TEST_MOS_EXCEPTIONAL;

typedef enum {
  SELF_TEST_MOS_INFO = 0,
  SELF_TEST_HALL_INFO,
  SELF_TEST_END_INFO,
} BLDC_SELF_TEST_MSG_INFO;
   
class BldcMotor {
public: 
  BldcMotor();
  bool Init();
  void Loop();

  void BldcSetMosEnableState(bool enable);
  void BldcSetMotorFanEnableState(bool enable);
  bool BldcSetMotorDirection(MOTOR_DIR dir);
  bool BldcControlMotorRunProcess(MOTOR_STATE ctr);
  void BldcSetMotorSpeedPower(float new_power);
  void BldcSetMotorTargetRpm(float rpm);
  void BldcSetMotorPolePairNum(uint8_t num);
  void BldcIncPIDInit(void); 
  void BldcIncPIDSetPID(uint8_t index, float parm);
  float BldcIncPIDGetPID(uint8_t index);
  float BldcGetMotorSpeedPower(void);
  float BldcGetMotorRpm(void);
  float BldcCalCustomOutput(float cur_prm, float target_prm);
  uint32_t BldcGetMultiChannelAdc(MultiChannel channel,uint8_t mode);
   
  MOTOR_STATE BldcGetMotorState(void);
  MOTOR_DIR BldcGetMotorDirection(void);
  bool BldcGetMotorPidControl(void);
  bool BldcSetMotorPidControl(bool operation);
  MOTOR_BLOCK_STATE BldcGetMotorBlockState(void);
  void BldcSetMotorBlockState(MOTOR_BLOCK_STATE state);

  // bldc self test api interface
  static bool BldcStartSelfTest(void);
  static void BldcSelfTestLoop(uint32_t cur_tick);

private:
  void BldcTim1GpioConfig(ADVANCED_TIMER1_MAP type);
  void BldcTim1SetConfig(void);
  void BldcAdcInit(void);
  bool CheckInit(void);
  void BldcMosEnableGpioInit(void);
  void BldcMotorFanGpioInit(void);
  float BldcIncPIDCalc(PID *bldc_pid, float NextPoint);
  static void BldcPhaseChange(unsigned int step);
  static void BldcPublicTimerCallBack(void);
  static void BldcHallIrqCallBack(uint8_t line);
  static void BldcHardProductIrqCallBack(uint8_t line); 
  static void BldcPublicTimerInit(void); 
  static void BldcHallIrqInit(void);

  static void BldcSelfTestMosEnableIrq(BLDC_SELF_TEST_MOS_EXCEPTIONAL index);
  static void BldcDisableAllIrq(void);
  static uint8_t BldcSelfNextMosIndex(BLDC_SELF_TEST_MOS_EXCEPTIONAL index);
  static uint8_t BldcSelfNextHallIndex(uint8_t index); 
  static void BldcStartHallSelfTest(uint8_t index);

private:
  static float speed_power_;
  static uint32_t pre_step_;
  static float motor_rpm_;
  static MOTOR_BLOCK_STATE motor_block_;
  static BldcMotor *p_blcd_motor_dev_;

  uint8_t adc_index[MAX_CHANNEL];
  bool init_ok_ = false;
  bool pid_control_;    //Enable or disable PID Speed control
  MOTOR_DEVICE bldc_;
  PID bldc_pid_parm_;
  bool break_enable_;
  uint32 break_keep_cnt_;
  uint32 break_close_sum_;
  uint32 rpm_block_cnt_;
};
#endif
