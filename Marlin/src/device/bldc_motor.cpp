#include <wirish_math.h>
#include <board/board.h>
#include <include/libmaple/libmaple_types.h>
#include "src/HAL/std_library/inc/stm32f10x.h"
#include "src/HAL/hal_tim.h"
#include "src/HAL/hal_adc.h"
#include "src/HAL/hal_exti.h"
#include "bldc_motor.h"

#define BLDC_TIMx                                   TIM1     // All six channels are in one timer, with this way timer must select advanced timer.
#define BLDC_TIM_PWM_FREQ                           20000 //50000 Hz
#define BLDC_TIM_PRESCALER                          0          // 72M
#define BLDC_TIM_PERIOD                             ((uint16_t)(SystemCoreClock/(BLDC_TIM_PRESCALER+1)/BLDC_TIM_PWM_FREQ)) //3600
#define BLDC_TIM_REPETITIONCOUNTER                  0
#define BLDC_TIM_DEAD_TIME                          0 //127//15   //DTG[7:5]=0xx => DT=DTG[7:0] × Tdtg， Tdtg = TDTS；  DeadTime = 1 / 72M  *  BLDC_TIM_DEAD_TIME = 13.89ns*15 = 208ns
#define BLDC_TIM_BKIN_ENABLE                        TIM_Break_Disable
#define BLDC_TIM_AUTOMATICOUTPUT                    TIM_AutomaticOutput_Enable
#define BLDC_TIM_BKIN_PIN_POLARITY                  TIM_BreakPolarity_Low

#define BLDC_PUBLIC_TIMER                           2 //STM32F10X_MD TIM 2\3\4
#define BLDC_PUBLIC_PRO_TIMES                       100 // 100-> 1000/100 = 10ms
#define BLDC_PUBLIC_CAPTURE_SPEED_CNT               2 // 2 * 10 = 20ms PID control time

#define BLDC_OUTBREAK_RPM_ERROR_TRIGGER             0.3 
#define BLDC_OUTBREAK_KEEP_MAX_CNT                  50 
#define BLDC_OUTBREAK_CLOSE_SUM_CNT                 200                  

#define BLDC_PUBLIC_TIMER_PRESCALER                 7200
#define BLDC_PUBLIC_STALLING_MAX_CNT                25 // 25 * PID control time = 25 * 20 = 500ms  Blocked turn determination time
#define BLDC_PUBLIC_STALLING_RPM_MAX_CNT            40 
#define BLDC_PUBLIC_TIMER_PREEMPTIONPRIORITY        2
#define BLDC_PUBLIC_TIMER_SUBPRIORITY               0
#define BLDC_PUBLIC_TIMER_PERIOD                    ((uint16_t)(SystemCoreClock/BLDC_PUBLIC_TIMER_PRESCALER/BLDC_PUBLIC_PRO_TIMES))

#define BLDC_MOTOR_SWJ_REMAP_TYPE                   GPIO_Remap_SWJ_JTAGDisable

#define ADC_CHANNEL_NUM                             5 
#define ADC_SAMPLING_DEPTH                          10
#define ADC_DMA_SIZE                                (ADC_CHANNEL_NUM * ADC_SAMPLING_DEPTH)

#define BLDC_TIM_CH1_PORT                           GPIOA
#define BLDC_TIM_CH1_PIN                            GPIO_Pin_8
#define BLDC_TIM_CH2_PORT                           GPIOA
#define BLDC_TIM_CH2_PIN                            GPIO_Pin_9
#define BLDC_TIM_CH3_PORT                           GPIOA
#define BLDC_TIM_CH3_PIN                            GPIO_Pin_10

#define BLDC_TIM_CH1N_PORT                          GPIOA
#define BLDC_TIM_CH1N_PIN                           GPIO_Pin_7
#define BLDC_TIM_CH2N_PORT                          GPIOB
#define BLDC_TIM_CH2N_PIN                           GPIO_Pin_0
#define BLDC_TIM_CH3N_PORT                          GPIOB
#define BLDC_TIM_CH3N_PIN                           GPIO_Pin_1

#define BLDC_GD_STOP_GPIO_CLK                       RCC_APB2Periph_GPIOA
#define BLDC_GD_STOP_PORT                           GPIOA
#define BLDC_GD_STOP_PIN                            GPIO_Pin_5//GPIO_Pin_4
#define BLDC_GD_STOP_ON                             1 

#define BLDC_MOTOR_FAN_GPIO_CLK                     RCC_APB2Periph_GPIOA
#define BLDC_MOTOR_FAN_PORT                         GPIOA
#define BLDC_MOTOR_FAN_PIN                          GPIO_Pin_2
#define BLDC_MOTOR_FAN_ON                           1 

#define HALL_EXTI_IRQn_U                            EXTI3_IRQn
#define HALL_EXTI_IRQn_V                            EXTI4_IRQn
#define HALL_EXTI_IRQn_W                            EXTI9_5_IRQn

#define HALL_U_GPIO_PIN_MAP                         PB3
#define HALL_U_EXITLINE                             EXTI_Line3
#define HALL_V_GPIO_PIN_MAP                         PB4
#define HALL_V_EXITLINE                             EXTI_Line4
#define HALL_W_GPIO_PIN_MAP                         PB5
#define HALL_W_EXITLINE                             EXTI_Line5
#define GET_HALL_STATE                              ((GPIOB->IDR &0x38) >> 3)//((GPIOB->IDR &0xE0) >> 5)

#define HARD_FAULT_GPIO_PIN_MAP                     PA6
#define HARD_FAULT_EXITLINE                         EXTI_Line6

#define PURIFIER_ADC_PERIOD_US                      1000
#define PURIFIER_ADC_TIM                            ADC_TIM_4
#define V_ADC                                       PA3 , PURIFIER_ADC_TIM, PURIFIER_ADC_PERIOD_US
#define M_ADC                                       PA0 , PURIFIER_ADC_TIM, PURIFIER_ADC_PERIOD_US
#define P_ADC                                       PA1 , PURIFIER_ADC_TIM, PURIFIER_ADC_PERIOD_US
#define I_ADC                                       PA4 , PURIFIER_ADC_TIM, PURIFIER_ADC_PERIOD_US

uint32_t BldcMotor::pre_step_ = 0;
BldcMotor* BldcMotor::p_blcd_motor_dev_ = NULL;
float BldcMotor::speed_power_ = 0;
float BldcMotor::motor_rpm_ = 0;
MOTOR_BLOCK_STATE BldcMotor::motor_block_ = MOTOR_BLOCK_NORMAL;

volatile BLDC_SELF_TEST_STEP bldc_self_test_step = SELF_TEST_IDLE;
volatile BLDC_SELF_TEST_MOS_EXCEPTIONAL mos_test_index = SELF_TEST_MOS_INVALID_INDEX;
volatile bool hardware_protect_trigger = false;
volatile bool hall_check_is_ok = false;
volatile float bldc_mos_current = 0;
volatile uint8_t hall_check_cnt = 0;
volatile uint8_t hall_check_index = 0;
volatile uint32_t g_hall_sta = 0;
volatile uint32_t bldc_self_test_tick = 0;
volatile uint32_t bldc_self_test_err_sta = 0;
volatile uint32_t self_test_send_flag = 0;

uint8_t hall_check_order[CHECK_HALL_CNT] =  {2, 3, 1, 5, 4, 6};

BldcMotor::BldcMotor(){
  bldc_.motor_state = STOP;
  bldc_.motor_speed = 0;
  bldc_.stalling_count = 0;
  bldc_.motor_direction = CW;
  bldc_.step_counter = 0;
  bldc_.pole_pair_num = MOTOR_POLE_PAIR_NUM;
  init_ok_ = false;
  pid_control_ = true;
  rpm_block_cnt_ = 0;
  break_close_sum_ = 0;
  break_keep_cnt_ = BLDC_OUTBREAK_KEEP_MAX_CNT;
  break_enable_ = false;
}

void BldcMotor::BldcTim1GpioConfig(ADVANCED_TIMER1_MAP type) {
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  if (type == DEFAULT_REMAP) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure); 
  }
  else if (type == PARTIAL_REMAP) {   
    // Partial Remap
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_PartialRemap_TIM1, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);  
  }
  else {
    // Full Remap
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM1, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_Init(GPIOE, &GPIO_InitStructure);  

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
  }

#ifdef BLDC_MOTOR_SWJ_REMAP_TYPE
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  GPIO_PinRemapConfig(BLDC_MOTOR_SWJ_REMAP_TYPE, ENABLE);
#endif
}

void BldcMotor::BldcTim1SetConfig(void) {
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

    TIM_TimeBaseStructure.TIM_Period = BLDC_TIM_PERIOD - 1; 
    TIM_TimeBaseStructure.TIM_Prescaler = BLDC_TIM_PRESCALER;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = BLDC_TIM_REPETITIONCOUNTER;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity= TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);

    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);

    TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
    TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
    TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
    TIM_BDTRInitStructure.TIM_DeadTime = BLDC_TIM_DEAD_TIME;
    TIM_BDTRInitStructure.TIM_BreakPolarity = BLDC_TIM_BKIN_PIN_POLARITY;
    TIM_BDTRInitStructure.TIM_AutomaticOutput = BLDC_TIM_AUTOMATICOUTPUT;
    TIM_BDTRInitStructure.TIM_Break = BLDC_TIM_BKIN_ENABLE;
    TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);

    TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM1,TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(BLDC_TIMx, ENABLE);
    TIM_CCxCmd(TIM1,TIM_Channel_1,TIM_CCx_Disable);
    TIM_CCxNCmd(TIM1,TIM_Channel_1,TIM_CCxN_Disable);
    TIM_CCxCmd(TIM1,TIM_Channel_2,TIM_CCx_Disable);
    TIM_CCxNCmd(TIM1,TIM_Channel_2,TIM_CCxN_Disable);
    TIM_CCxCmd(TIM1,TIM_Channel_3,TIM_CCx_Disable);
    TIM_CCxNCmd(TIM1,TIM_Channel_3,TIM_CCxN_Disable);
}

void BldcMotor::BldcMosEnableGpioInit(void) {
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(BLDC_GD_STOP_GPIO_CLK, ENABLE);

  GPIO_InitStructure.GPIO_Pin = BLDC_GD_STOP_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(BLDC_GD_STOP_PORT, &GPIO_InitStructure);
  GPIO_WriteBit(BLDC_GD_STOP_PORT, BLDC_GD_STOP_PIN, (BitAction)(!BLDC_GD_STOP_ON)); 
}

void BldcMotor::BldcSetMosEnableState(bool enable) {
  GPIO_WriteBit(BLDC_GD_STOP_PORT, BLDC_GD_STOP_PIN, enable  ? \
      (BitAction)(BLDC_GD_STOP_ON) : (BitAction)(!BLDC_GD_STOP_ON));
}

void BldcMotor::BldcMotorFanGpioInit(void) {
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(BLDC_MOTOR_FAN_GPIO_CLK, ENABLE);

  GPIO_InitStructure.GPIO_Pin = BLDC_MOTOR_FAN_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(BLDC_MOTOR_FAN_PORT, &GPIO_InitStructure);
  GPIO_WriteBit(BLDC_MOTOR_FAN_PORT, BLDC_MOTOR_FAN_PIN, (BitAction)(!BLDC_MOTOR_FAN_ON)); 
}

void BldcMotor::BldcSetMotorFanEnableState(bool enable) {
  GPIO_WriteBit(BLDC_MOTOR_FAN_PORT, BLDC_MOTOR_FAN_PIN, enable  ? \
      (BitAction)(BLDC_MOTOR_FAN_ON) : (BitAction)(!BLDC_MOTOR_FAN_ON));
}

void BldcMotor::BldcAdcInit(void) {
  adc_index[V_ADC_CHANNEL] = HAL_adc_init(V_ADC);
//  adc_index[S_ADC_CHANNEL] = HAL_adc_init(S_ADC);
  adc_index[M_ADC_CHANNEL] = HAL_adc_init(M_ADC);
  adc_index[P_ADC_CHANNEL] = HAL_adc_init(P_ADC);
  adc_index[I_ADC_CHANNEL] = HAL_adc_init(I_ADC);
}

uint32_t BldcMotor::BldcGetMultiChannelAdc(MultiChannel channel,uint8_t mode) {
  uint32_t adc = 0;
  if (channel < MAX_CHANNEL) {
    if (!mode)
      adc = ADC_Get(adc_index[channel]);
    else
      adc = ADC_GetCusum(adc_index[channel]);
  }
  return adc;
}

bool BldcMotor::CheckInit(void) {
  return (p_blcd_motor_dev_ && init_ok_);
}

MOTOR_STATE BldcMotor::BldcGetMotorState(void) {
  return bldc_.motor_state;
}

MOTOR_DIR BldcMotor::BldcGetMotorDirection(void) {
  return bldc_.motor_direction;
}

bool BldcMotor::BldcSetMotorDirection(MOTOR_DIR dir){
  bool ret = false;
  if (bldc_.motor_state == STOP) {
    bldc_.motor_direction = dir;
    ret = true;
  }
  return ret;
}

float BldcMotor::BldcGetMotorRpm(void) {
  return bldc_.motor_speed;
}

void BldcMotor::BldcSetMotorSpeedPower(float new_power) {
  if (!CheckInit())
    return;
  if (speed_power_ < MIN_DUTY_CYCLE)
    speed_power_ = MIN_DUTY_CYCLE;
  else if (speed_power_ >= MAX_DUTY_CYCLE)
    speed_power_ = MAX_DUTY_CYCLE;
  else
    speed_power_ = new_power;
}

float BldcMotor::BldcGetMotorSpeedPower(void) {
  return speed_power_;
}

void BldcMotor::BldcSetMotorPolePairNum(uint8_t num) {
  bldc_.pole_pair_num = num;
}

bool BldcMotor::BldcControlMotorRunProcess(MOTOR_STATE ctr) {
  if (!CheckInit())
    return false;

  switch (ctr) {
    case STOP:
      if(bldc_.motor_state == RUN) {
        speed_power_ = 0;
        BldcSetMosEnableState(false);
        BldcSetMotorFanEnableState(false);
        TIM_CCxCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCx_Disable);
        TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCxN_Disable);
        TIM_CCxCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCx_Disable);
        TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCxN_Disable);
        TIM_CCxCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCx_Disable);
        TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCxN_Disable);
        bldc_.motor_state = STOPING; 
      }
      break;

    case RUN:
      if(bldc_.motor_state != RUN){
        bldc_.stalling_count = 0;
        bldc_.step_counter = 0;
        bldc_pid_parm_.LastError = 0;
        bldc_pid_parm_.PrevError = 0;
        bldc_pid_parm_.TargetRpmDown = false;
        bldc_pid_parm_.ErrorLimit = false;
        break_enable_ = false;
        rpm_block_cnt_ = 0;
        break_close_sum_ = 0;
        break_keep_cnt_ = BLDC_OUTBREAK_KEEP_MAX_CNT;
        BldcSetMotorBlockState(MOTOR_BLOCK_NORMAL);
        BldcSetMosEnableState(true);
        BldcSetMotorFanEnableState(true);
        speed_power_ = MIN_DUTY_CYCLE;    
        EXTI_ClearITPendingBit(HALL_U_EXITLINE|HALL_V_EXITLINE|HALL_W_EXITLINE|HARD_FAULT_EXITLINE);    
        NVIC_EnableIRQ(HALL_EXTI_IRQn_U);
        NVIC_EnableIRQ(HALL_EXTI_IRQn_V);
        NVIC_EnableIRQ(HALL_EXTI_IRQn_W);
        bldc_.motor_state = RUN;    
        BldcHallIrqCallBack(0);
        BldcHallIrqCallBack(0);
      }      
      break;

    default:
      break;
  }
  return true;
}

void BldcMotor::BldcSetMotorTargetRpm(float rpm) {
  if (rpm < MOTOR_MIN_SPEED) {
    rpm = MOTOR_MIN_SPEED;
  }
  else if (rpm > MOTOR_RATED_SPEED) {
    rpm = MOTOR_RATED_SPEED;
  }
  if (bldc_pid_parm_.SetPoint > rpm) 
    bldc_pid_parm_.TargetRpmDown = true;
  else 
    bldc_pid_parm_.TargetRpmDown = false;
  bldc_pid_parm_.SetPoint = rpm;
}

bool BldcMotor::BldcGetMotorPidControl(void) {
  return pid_control_;
}

bool BldcMotor::BldcSetMotorPidControl(bool operation) {
  bool ret = false;
  if (bldc_.motor_state != RUN) {
    ret = true;
    pid_control_ = operation;
  }
  return ret;
}

MOTOR_BLOCK_STATE BldcMotor::BldcGetMotorBlockState(void) {
  return motor_block_;
}

void BldcMotor::BldcSetMotorBlockState(MOTOR_BLOCK_STATE state) {
  motor_block_ = state;
}

void BldcMotor::BldcPhaseChange(unsigned int step) {
  if (!p_blcd_motor_dev_ || bldc_self_test_step)
    return;

  switch(step) {
    case 4: //B+ C-
      /* Next step: Step 2 Configuration -------------------------------------- */ 
      TIM_CCxCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCx_Disable);
      TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCxN_Disable);
      
      /*  Channel1 configuration */
      /*  Channel2 configuration */ 
      TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCxN_Disable);   
      TIM_SetCompare2(BLDC_TIMx,BLDC_TIM_PERIOD*speed_power_);
      TIM_CCxCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCx_Enable);

      /*  Channel3 configuration */
      TIM_CCxCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCx_Disable);
      TIM_SetCompare3(BLDC_TIMx,BLDC_TIM_PERIOD);
      TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCxN_Enable);
      break;

    case 5: //B+ A-
      /* Next step: Step 3 Configuration -------------------------------------- */      
      TIM_CCxCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCx_Disable);
      TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCxN_Disable);
      
      /*  Channel1 configuration */
      TIM_CCxCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCx_Disable);
      TIM_SetCompare1(BLDC_TIMx,BLDC_TIM_PERIOD);
      TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCxN_Enable);
    
      /*  Channel2 configuration */
      TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCxN_Disable);
      TIM_SetCompare2(BLDC_TIMx,BLDC_TIM_PERIOD*speed_power_);
      TIM_CCxCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCx_Enable);
      /*  Channel3 configuration */
      break;

    case 1: //C+ A-
      /* Next step: Step 4 Configuration -------------------------------------- */
      TIM_CCxCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCx_Disable);
      TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCxN_Disable);
    
      /*  Channel1 configuration */
      TIM_CCxCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCx_Disable);
      TIM_SetCompare1(BLDC_TIMx,BLDC_TIM_PERIOD);
      TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCxN_Enable);
      
      /*  Channel2 configuration */ 
      /*  Channel3 configuration */
      TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCxN_Disable);
      TIM_SetCompare3(BLDC_TIMx,BLDC_TIM_PERIOD*speed_power_);
      TIM_CCxCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCx_Enable);
      break;

    case 3: //C+ B-
      /* Next step: Step 5 Configuration -------------------------------------- */
      TIM_CCxCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCx_Disable);
      TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCxN_Disable);    
    
      /*  Channel1 configuration */      
      /*  Channel2 configuration */   
      TIM_CCxCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCx_Disable);
      TIM_SetCompare2(BLDC_TIMx,BLDC_TIM_PERIOD);
      TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCxN_Enable);
    
      /*  Channel3 configuration */    
      TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCxN_Disable);  
      TIM_SetCompare3(BLDC_TIMx,BLDC_TIM_PERIOD*speed_power_);
      TIM_CCxCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCx_Enable);
      break;

    case 2: //A+ B-
      /* Next step: Step 6 Configuration -------------------------------------- */
      TIM_CCxCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCx_Disable);
      TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCxN_Disable);
      
      /*  Channel1 configuration */
      TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCxN_Disable);
      TIM_SetCompare1(BLDC_TIMx,BLDC_TIM_PERIOD*speed_power_);
      TIM_CCxCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCx_Enable);

      /*  Channel2 configuration */
      TIM_CCxCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCx_Disable);
      TIM_SetCompare2(BLDC_TIMx,BLDC_TIM_PERIOD);
      TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCxN_Enable);
      /*  Channel3 configuration */
      break;
      
    case 6: //A+ C-
      /* Next step: Step 1 Configuration -------------------------------------- */
      TIM_CCxCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCx_Disable);
      TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCxN_Disable);
      
      /*  Channel1 configuration */
      TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCxN_Disable);
      TIM_SetCompare1(BLDC_TIMx,BLDC_TIM_PERIOD*speed_power_);
      TIM_CCxCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCx_Enable);
      /*  Channel2 configuration */      
      /*  Channel3 configuration */
      TIM_CCxCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCx_Disable);
      TIM_SetCompare3(BLDC_TIMx,BLDC_TIM_PERIOD);
      TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCxN_Enable);
      break;

    default:
      TIM_CCxCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCx_Disable);
      TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCxN_Disable);
      TIM_CCxCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCx_Disable);
      TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCxN_Disable);
      TIM_CCxCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCx_Disable);
      TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCxN_Disable);
      break;
  }
}


void BldcMotor::BldcPublicTimerCallBack(void) {
  static unsigned int time_count = 0;
  float pid_result = 0;
  if (p_blcd_motor_dev_ && bldc_self_test_step == SELF_TEST_IDLE) {
    if (p_blcd_motor_dev_->bldc_.motor_state != STOP) {
      time_count++;
      if (time_count >= BLDC_PUBLIC_CAPTURE_SPEED_CNT) { 
        //  (1000 / BLDC_PUBLIC_PRO_TIMES * BLDC_PUBLIC_CAPTURE_SPEED_CNT) MS
        float seep_rpm_tmp = ((p_blcd_motor_dev_->bldc_.step_counter *  BLDC_PUBLIC_PRO_TIMES * 10) \
           / (p_blcd_motor_dev_->bldc_.pole_pair_num * BLDC_PUBLIC_CAPTURE_SPEED_CNT));
        p_blcd_motor_dev_->bldc_.motor_speed = seep_rpm_tmp;
        
        if (p_blcd_motor_dev_->bldc_.motor_state == RUN) {
          if (p_blcd_motor_dev_->BldcGetMotorPidControl()) {
            float tmp_speed_power = speed_power_;
            float limit_stall_speed = p_blcd_motor_dev_->bldc_pid_parm_.SetPoint * BLDC_MOTOR_STALL_SCALE_LIMIT;
            if (seep_rpm_tmp < MOTOR_BLOCK_SPEED || seep_rpm_tmp < limit_stall_speed)
              p_blcd_motor_dev_-> rpm_block_cnt_++;
            else 
              p_blcd_motor_dev_-> rpm_block_cnt_ = 0;

            #ifdef USE_PID_CTRL_MOTOR_RPM
              pid_result = p_blcd_motor_dev_->BldcIncPIDCalc(&p_blcd_motor_dev_->bldc_pid_parm_, seep_rpm_tmp);
              pid_result = pid_result / BLDC_TIM_PERIOD; 

              if (pid_result > PID_CTRL_MAX_CHANGE_POWER) 
                pid_result = PID_CTRL_MAX_CHANGE_POWER;
            #else
              pid_result = p_blcd_motor_dev_->BldcCalCustomOutput(seep_rpm_tmp, p_blcd_motor_dev_->bldc_pid_parm_.SetPoint);
            #endif
            if (pid_result + tmp_speed_power < 0)
              tmp_speed_power = 0;
            else if((pid_result + tmp_speed_power) > MAX_DUTY_CYCLE)
              tmp_speed_power = MAX_DUTY_CYCLE;
            else
              tmp_speed_power += pid_result; 
            #ifdef USE_PID_CTRL_KE_LIMIT
              float v_out = tmp_speed_power * BLDC_MOTOR_WORKING_VOLTAGE;
              float v_ke = seep_rpm_tmp * USE_PID_CTRL_KE_LIMIT;
              #ifdef USE_PID_BREAK_MODE_MOTOR_RPM
              float s_v = 0;
              if (p_blcd_motor_dev_->break_enable_)
                s_v = BLDC_MOTOR_RESISTANCE * BLDC_MOTOR_BREAK_MAX_CURRENT;
              else 
                s_v = BLDC_MOTOR_RESISTANCE * BLDC_MOTOR_WORK_MAX_CURRENT;

              if (v_out - v_ke > s_v) {
                if (p_blcd_motor_dev_->bldc_pid_parm_.ErrorLimit) {
                  if (!p_blcd_motor_dev_->break_enable_ && p_blcd_motor_dev_->break_close_sum_ == 0) {
                    p_blcd_motor_dev_->break_enable_ = true;
                  }
                }
                tmp_speed_power = (s_v + v_ke) / BLDC_MOTOR_WORKING_VOLTAGE;
              }
              #endif
              if (p_blcd_motor_dev_->bldc_pid_parm_.TargetRpmDown) {
                v_ke = p_blcd_motor_dev_->bldc_pid_parm_.SetPoint * USE_PID_CTRL_KE_LIMIT *0.4;
                if (v_out < v_ke)
                  tmp_speed_power = v_ke / BLDC_MOTOR_WORKING_VOLTAGE;
              }
              #ifdef USE_PID_BREAK_MODE_MOTOR_RPM
              if (p_blcd_motor_dev_->break_enable_) {
                p_blcd_motor_dev_->break_close_sum_ = BLDC_OUTBREAK_CLOSE_SUM_CNT;
                if (p_blcd_motor_dev_->break_keep_cnt_ > BLDC_OUTBREAK_KEEP_MAX_CNT)
                  p_blcd_motor_dev_->break_keep_cnt_ = BLDC_OUTBREAK_KEEP_MAX_CNT;
                
                if (p_blcd_motor_dev_->break_keep_cnt_ > 0)
                  p_blcd_motor_dev_->break_keep_cnt_--;
                
                if (p_blcd_motor_dev_->break_keep_cnt_ == 0) {
                  p_blcd_motor_dev_->break_enable_ = false;
                }
              }
              else {
                if (p_blcd_motor_dev_->break_close_sum_ > 0)
                  p_blcd_motor_dev_->break_close_sum_--;
                p_blcd_motor_dev_->break_keep_cnt_ = BLDC_OUTBREAK_KEEP_MAX_CNT;
              }
              #endif
            #endif
            speed_power_ = tmp_speed_power;
          }
          else {
            p_blcd_motor_dev_-> rpm_block_cnt_ = 0;
          }
        }
        else {
          p_blcd_motor_dev_-> rpm_block_cnt_ = 0;
        }
        time_count = 0; 
        p_blcd_motor_dev_->bldc_.step_counter = 0;
      }

      if (p_blcd_motor_dev_->bldc_.motor_state != STOP)
        p_blcd_motor_dev_->bldc_.stalling_count++;

      if(p_blcd_motor_dev_->bldc_.stalling_count > BLDC_PUBLIC_STALLING_MAX_CNT || \
          p_blcd_motor_dev_-> rpm_block_cnt_ > BLDC_PUBLIC_STALLING_RPM_MAX_CNT) {
        if (p_blcd_motor_dev_->bldc_.motor_state == RUN && \
          p_blcd_motor_dev_->motor_block_ == MOTOR_BLOCK_NORMAL)
          p_blcd_motor_dev_->motor_block_ = MOTOR_BLOCK_S_SPIN;
        p_blcd_motor_dev_->bldc_.motor_state = STOP;
        EXTI_ClearITPendingBit(HALL_U_EXITLINE|HALL_V_EXITLINE|HALL_W_EXITLINE|HARD_FAULT_EXITLINE);
        NVIC_DisableIRQ(HALL_EXTI_IRQn_U);
        NVIC_DisableIRQ(HALL_EXTI_IRQn_V);
        NVIC_DisableIRQ(HALL_EXTI_IRQn_W);
        TIM_CCxCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCx_Disable);
        TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCxN_Disable);
        TIM_CCxCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCx_Disable);
        TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCxN_Disable);
        TIM_CCxCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCx_Disable);
        TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCxN_Disable);  
        p_blcd_motor_dev_->bldc_.stalling_count = 0;   
        p_blcd_motor_dev_-> rpm_block_cnt_ = 0;
        p_blcd_motor_dev_->BldcSetMosEnableState(false); 
        p_blcd_motor_dev_->BldcSetMotorFanEnableState(false);
      }
    }
    else {
      time_count = 0;
      p_blcd_motor_dev_-> rpm_block_cnt_  = 0;
    }
  }
}

void BldcMotor::BldcHallIrqCallBack(uint8_t line) {
  if (p_blcd_motor_dev_) {
    unsigned int uwStep = GET_HALL_STATE;
    if (p_blcd_motor_dev_->BldcGetMotorState() == RUN) {
      if (p_blcd_motor_dev_->BldcGetMotorDirection() == CW) {
        uwStep = 7 - uwStep;
      }
      BldcPhaseChange(uwStep);
    }

    if (p_blcd_motor_dev_->BldcGetMotorState() != STOP) {
      if (pre_step_ != uwStep) {
        p_blcd_motor_dev_->bldc_.step_counter ++;
      }
      pre_step_ = uwStep;
    }
    p_blcd_motor_dev_->bldc_.stalling_count = 0;
  }
}

void BldcMotor::BldcHardProductIrqCallBack(uint8_t line) {
  if (p_blcd_motor_dev_) {
    bool bldc_disable = false;
    if (bldc_self_test_step) {
      if (bldc_self_test_step == SELF_TESTING_MOS_CHECKING || bldc_self_test_step == SELF_TESTING_HALL_CHECKING) {
        hardware_protect_trigger = true;
        bldc_disable = true;
      }
    }
    else {
      if (p_blcd_motor_dev_->bldc_.motor_state == RUN) {
        bldc_disable = true;
        p_blcd_motor_dev_->bldc_.motor_state = STOPING; 
        if (p_blcd_motor_dev_->motor_block_ == MOTOR_BLOCK_NORMAL)
          p_blcd_motor_dev_->motor_block_ = MOTOR_BLOCK_H_PROTECT;
      }  
    }  
    
    if (bldc_disable) {
      EXTI_ClearITPendingBit(HALL_U_EXITLINE|HALL_V_EXITLINE|HALL_W_EXITLINE|HARD_FAULT_EXITLINE);
      NVIC_DisableIRQ(HALL_EXTI_IRQn_U);
      NVIC_DisableIRQ(HALL_EXTI_IRQn_V);
      NVIC_DisableIRQ(HALL_EXTI_IRQn_W);
      TIM_CCxCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCx_Disable);
      TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCxN_Disable);
      TIM_CCxCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCx_Disable);
      TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCxN_Disable);
      TIM_CCxCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCx_Disable);
      TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCxN_Disable);  
      p_blcd_motor_dev_->bldc_.stalling_count = 0;  
      p_blcd_motor_dev_->BldcSetMosEnableState(false); 
      p_blcd_motor_dev_->BldcSetMotorFanEnableState(false);       
    }  
  }
}

void BldcMotor::BldcPublicTimerInit(void) {
  HAL_timer_init(BLDC_PUBLIC_TIMER, BLDC_PUBLIC_TIMER_PRESCALER, BLDC_PUBLIC_TIMER_PERIOD);
  HAL_timer_nvic_init(BLDC_PUBLIC_TIMER, BLDC_PUBLIC_TIMER_PREEMPTIONPRIORITY, BLDC_PUBLIC_TIMER_SUBPRIORITY);
  HAL_timer_cb_init(BLDC_PUBLIC_TIMER, BldcPublicTimerCallBack);
  HAL_timer_enable(BLDC_PUBLIC_TIMER);
}

void BldcMotor::BldcHallIrqInit(void) { 
  ExtiInit(HALL_U_GPIO_PIN_MAP,EXTI_Rising_Falling,BldcHallIrqCallBack,EXTI_MODE_IPU);
  ExtiInit(HALL_V_GPIO_PIN_MAP,EXTI_Rising_Falling,BldcHallIrqCallBack,EXTI_MODE_IPU);
  ExtiInit(HALL_W_GPIO_PIN_MAP,EXTI_Rising_Falling,BldcHallIrqCallBack,EXTI_MODE_IPU);
  // add product check
  ExtiInit(HARD_FAULT_GPIO_PIN_MAP,EXTI_Falling,BldcHardProductIrqCallBack,EXTI_MODE_IPU);
}

void BldcMotor::BldcIncPIDInit(void) {
  bldc_pid_parm_.LastError   = 0;                        //Error[-1]
  bldc_pid_parm_.PrevError   = 0;                        //Error[-2]
  bldc_pid_parm_.Proportion  = P_DEFAULT_DATA;           //Proportional Const
  bldc_pid_parm_.Integral    = I_DEFAULT_DATA;           //Integral Const
  bldc_pid_parm_.Derivative  = D_DEFAULT_DATA;           //Derivative Const
  bldc_pid_parm_.SetPoint    = 0;                        //Desired Value
  bldc_pid_parm_.TargetRpmDown = false;
  bldc_pid_parm_.ErrorLimit = false;
}

void BldcMotor::BldcIncPIDSetPID(uint8_t index, float parm) {
  switch (index) {
    case 0:
      bldc_pid_parm_.Proportion = parm;
      break;
      
    case 1:
      bldc_pid_parm_.Integral = parm;
      break;

    case 2:
      bldc_pid_parm_.Derivative = parm;
      break;
    
    default:
      break;
  }
}

float BldcMotor::BldcIncPIDGetPID(uint8_t index) {
  float value = 0;
  switch (index) {
    case 0:
      value = bldc_pid_parm_.Proportion;
      break;
      
    case 1:
      value = bldc_pid_parm_.Integral;
      break;

    case 2:
      value = bldc_pid_parm_.Derivative;
      break;
    
    default:
      break;
  }
  return value;
}

float BldcMotor::BldcIncPIDCalc(PID *bldc_pid, float NextPoint) {
  float iError, iIncpid;  
  iError  = bldc_pid->SetPoint - NextPoint; 

  if (abs(iError) <= 200) 
    iError = 0; 


  if ((iError  > 0 ) && (iError > BLDC_OUTBREAK_RPM_ERROR_TRIGGER * bldc_pid->SetPoint)) 
    bldc_pid->ErrorLimit = true;
  else
    bldc_pid->ErrorLimit = false;
  
  // iIncpid = (bldc_pid->Proportion * iError)                   
  //             -(bldc_pid->Integral * bldc_pid->PrevError)     
  //             +(bldc_pid->Derivative * bldc_pid->LastError);  

  iIncpid = (bldc_pid->Proportion * (iError - bldc_pid->PrevError))                   
               +(bldc_pid->Integral * iError)     
               +(bldc_pid->Derivative * (iError - 2*bldc_pid->PrevError + bldc_pid->LastError)); 

  bldc_pid->LastError = bldc_pid->PrevError;
  bldc_pid->PrevError = iError;   
  return (iIncpid);                                    
}

float BldcMotor::BldcCalCustomOutput(float cur_prm, float target_prm) {
  float output_ = 0.01;
  float error = 0;
  error = target_prm - cur_prm;

  if (abs(error) <= 1000)
    output_ = 0.001;
  
  if (abs(error) <= 150) {
    error = 0;
    output_ = 0;
  }
    
  if (error < 0)
    output_ = -1 * output_ * 0.1;
  return output_;                                    
}

bool BldcMotor::Init(void) {
  if (p_blcd_motor_dev_ == NULL) {
    p_blcd_motor_dev_ = this;
    BldcIncPIDInit();
    BldcTim1GpioConfig(PARTIAL_REMAP);
    BldcTim1SetConfig();
    BldcMosEnableGpioInit();
    BldcMotorFanGpioInit();
    BldcAdcInit();
    BldcPublicTimerInit();
    BldcHallIrqInit();
    this->init_ok_ = true;
  }
  return this->init_ok_;
}

// bldc self test api interface
void BldcMotor::BldcDisableAllIrq(void) {
  EXTI_ClearITPendingBit(HALL_U_EXITLINE|HALL_V_EXITLINE|HALL_W_EXITLINE|HARD_FAULT_EXITLINE);
  NVIC_DisableIRQ(HALL_EXTI_IRQn_U);
  NVIC_DisableIRQ(HALL_EXTI_IRQn_V);
  NVIC_DisableIRQ(HALL_EXTI_IRQn_W);
  TIM_CCxCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCx_Disable);
  TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCxN_Disable);
  TIM_CCxCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCx_Disable);
  TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCxN_Disable);
  TIM_CCxCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCx_Disable);
  TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCxN_Disable);
  if (p_blcd_motor_dev_) {
    p_blcd_motor_dev_->BldcSetMosEnableState(false);
    p_blcd_motor_dev_->BldcSetMotorFanEnableState(false);
  }
}

uint8_t BldcMotor::BldcSelfNextMosIndex(BLDC_SELF_TEST_MOS_EXCEPTIONAL index) {
  uint8_t next_index = index;
  if (next_index < SELF_TEST_MOS_INVALID_INDEX) {
    next_index += 1;
  }
  return next_index;
}
uint8_t BldcMotor::BldcSelfNextHallIndex(uint8_t index) {
  uint8_t next_index = index;
  uint8_t i = 0;
  uint32_t hall_sta = GET_HALL_STATE;
  if (next_index == 0xff) {
    for ( ; i < CHECK_HALL_CNT; i++) {
      if (hall_sta == hall_check_order[i]) {
        break;
      }
    }
    if (i >= CHECK_HALL_CNT)
      next_index = 0;
    else 
      next_index = i;
  }
  else {
    next_index++;
    if (next_index >= CHECK_HALL_CNT)
      next_index = 0;
  }
  return next_index;
}

void BldcMotor::BldcSelfTestMosEnableIrq(BLDC_SELF_TEST_MOS_EXCEPTIONAL index) {
  BldcDisableAllIrq();
  NVIC_EnableIRQ(HALL_EXTI_IRQn_U);
  NVIC_EnableIRQ(HALL_EXTI_IRQn_V);
  NVIC_EnableIRQ(HALL_EXTI_IRQn_W);
  switch (index) {
    case SELF_TEST_MOS_UH_INDEX:
      TIM_CCxCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCx_Disable);
      TIM_SetCompare1(BLDC_TIMx,BLDC_TIM_PERIOD * BLDC_MOS_SELF_TEST_DUTY_CYCLE);
      TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCxN_Enable);
    break;

    case SELF_TEST_MOS_VH_INDEX:
      TIM_CCxCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCx_Disable);
      TIM_SetCompare2(BLDC_TIMx,BLDC_TIM_PERIOD * BLDC_MOS_SELF_TEST_DUTY_CYCLE);
      TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCxN_Enable);
    break;

    case SELF_TEST_MOS_WH_INDEX:
      TIM_CCxCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCx_Disable);
      TIM_SetCompare3(BLDC_TIMx,BLDC_TIM_PERIOD * BLDC_MOS_SELF_TEST_DUTY_CYCLE);
      TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCxN_Enable);
    break;

    case SELF_TEST_MOS_UL_INDEX:
      TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCxN_Disable);
      TIM_SetCompare1(BLDC_TIMx,BLDC_TIM_PERIOD * BLDC_MOS_SELF_TEST_DUTY_CYCLE);
      TIM_CCxCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCx_Enable);
    break;

    case SELF_TEST_MOS_VL_INDEX:
      TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCxN_Disable);   
      TIM_SetCompare2(BLDC_TIMx,BLDC_TIM_PERIOD * BLDC_MOS_SELF_TEST_DUTY_CYCLE);
      TIM_CCxCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCx_Enable);
    break;

    case SELF_TEST_MOS_WL_INDEX:
      TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCxN_Disable);  
      TIM_SetCompare3(BLDC_TIMx,BLDC_TIM_PERIOD * BLDC_MOS_SELF_TEST_DUTY_CYCLE);
      TIM_CCxCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCx_Enable);
    break;

    default:
    break;
  }

  if (p_blcd_motor_dev_) {
    p_blcd_motor_dev_->BldcSetMosEnableState(true);
    p_blcd_motor_dev_->BldcSetMotorFanEnableState(true);
  }
}


void BldcMotor::BldcStartHallSelfTest(uint8_t index) {
  NVIC_EnableIRQ(HALL_EXTI_IRQn_U);
  NVIC_EnableIRQ(HALL_EXTI_IRQn_V);
  NVIC_EnableIRQ(HALL_EXTI_IRQn_W);

  switch (index) {
    case 0: 
      //B+ A-
      TIM_CCxCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCx_Disable);
      TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCxN_Disable);
    
      TIM_CCxCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCx_Disable);
      TIM_SetCompare1(BLDC_TIMx,BLDC_TIM_PERIOD);
      TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCxN_Enable);
    
      TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCxN_Disable);
      TIM_SetCompare2(BLDC_TIMx,BLDC_TIM_PERIOD*BLDC_HALL_SELF_TEST_DUTY_CYCLE);
      TIM_CCxCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCx_Enable);
 
    break;

    case 1:
      //B+ C-
      TIM_CCxCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCx_Disable);
      TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCxN_Disable);
      
      TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCxN_Disable);   
      TIM_SetCompare2(BLDC_TIMx,BLDC_TIM_PERIOD*BLDC_HALL_SELF_TEST_DUTY_CYCLE);
      TIM_CCxCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCx_Enable);

      TIM_CCxCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCx_Disable);
      TIM_SetCompare3(BLDC_TIMx,BLDC_TIM_PERIOD);
      TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCxN_Enable);
    break;      
    
    case 2:
      //A+ C-
      TIM_CCxCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCx_Disable);
      TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCxN_Disable);
      
      TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCxN_Disable);
      TIM_SetCompare1(BLDC_TIMx,BLDC_TIM_PERIOD*BLDC_HALL_SELF_TEST_DUTY_CYCLE);
      TIM_CCxCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCx_Enable);

      TIM_CCxCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCx_Disable);
      TIM_SetCompare3(BLDC_TIMx,BLDC_TIM_PERIOD);
      TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCxN_Enable);
    break;      
    
    case 3:
      //A+ B-
      TIM_CCxCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCx_Disable);
      TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCxN_Disable);

      TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCxN_Disable);
      TIM_SetCompare1(BLDC_TIMx,BLDC_TIM_PERIOD*BLDC_HALL_SELF_TEST_DUTY_CYCLE);
      TIM_CCxCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCx_Enable);

      TIM_CCxCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCx_Disable);
      TIM_SetCompare2(BLDC_TIMx,BLDC_TIM_PERIOD);
      TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCxN_Enable);
    break;      
    
    case 4:
      //C+ B-
      TIM_CCxCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCx_Disable);
      TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCxN_Disable);    
  
      TIM_CCxCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCx_Disable);
      TIM_SetCompare2(BLDC_TIMx,BLDC_TIM_PERIOD);
      TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCxN_Enable);
    
      TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCxN_Disable);  
      TIM_SetCompare3(BLDC_TIMx,BLDC_TIM_PERIOD*BLDC_HALL_SELF_TEST_DUTY_CYCLE);
      TIM_CCxCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCx_Enable);
    break;      
    
    case 5:
      //C+ A-
      TIM_CCxCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCx_Disable);
      TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCxN_Disable);
    
      TIM_CCxCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCx_Disable);
      TIM_SetCompare1(BLDC_TIMx,BLDC_TIM_PERIOD);
      TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCxN_Enable);
      
      TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCxN_Disable);
      TIM_SetCompare3(BLDC_TIMx,BLDC_TIM_PERIOD*BLDC_HALL_SELF_TEST_DUTY_CYCLE);
      TIM_CCxCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCx_Enable);
    break;  

    default:
    break;
  }
  p_blcd_motor_dev_->BldcSetMosEnableState(true);
  p_blcd_motor_dev_->BldcSetMotorFanEnableState(true);
}

bool BldcCheckHallSelfTestSta(uint8_t index) {
  bool ret = false;
  index ++;
  if (index >= CHECK_HALL_CNT)
    index = 0;

  g_hall_sta = GET_HALL_STATE;

  if (g_hall_sta == hall_check_order[index])
    ret = true;
  return ret;
}

bool BldcMotor::BldcStartSelfTest(void) {
  bool ret = false;
  if (p_blcd_motor_dev_) {
    if ((p_blcd_motor_dev_->bldc_.motor_state == STOP) \
        && (bldc_self_test_step == SELF_TEST_IDLE)) {
      bldc_self_test_step = SELF_TEST_START;  
      self_test_send_flag = 0;
      ret = true;
    }
  }
  return ret;
}

void BldcMotor::BldcSelfTestLoop(uint32_t cur_tick) {
  if (p_blcd_motor_dev_) {
    switch (bldc_self_test_step) {
      case SELF_TEST_START:
        bldc_self_test_step = SELF_TESTING_MOS_PRE;
        bldc_self_test_err_sta = 0;
        mos_test_index = SELF_TEST_MOS_UH_INDEX;
      break;

      case SELF_TESTING_MOS_PRE:
        BldcSelfTestMosEnableIrq(mos_test_index);
        bldc_self_test_tick = cur_tick + BLDC_MOS_SELF_TEST_INTERVAL;
        hardware_protect_trigger = false;
        bldc_self_test_step = SELF_TESTING_MOS_CHECKING;
      break;

      case SELF_TESTING_MOS_CHECKING:
        if ((((int32_t)(cur_tick - bldc_self_test_tick)) > 0) || hardware_protect_trigger) {
          // bldc_mos_current = ((float)p_blcd_motor_dev_->BldcGetMultiChannelAdc(I_ADC_CHANNEL,0)) * 66000 / (4095 * 4.7);
          bldc_mos_current = ((float)p_blcd_motor_dev_->BldcGetMultiChannelAdc(I_ADC_CHANNEL,0)) * 16500 / 4095; // 0.01R * 20
          if (hardware_protect_trigger || bldc_mos_current > BLDC_MOS_SELF_TEST_CURRENT_LIMIT) {
            bldc_self_test_err_sta |= (1 << mos_test_index);
          }
      
          bldc_self_test_step = SELF_TESTING_MOS_POST;
          self_test_send_flag |= (1 << SELF_TEST_MOS_INFO);
          BldcDisableAllIrq();
          bldc_self_test_tick = cur_tick + BLDC_MOS_SELF_TEST_NEXT_INTERVAL;
        }
      break;

      case SELF_TESTING_MOS_POST:
        if ((self_test_send_flag & (1 << SELF_TEST_MOS_INFO)) || (((int32_t)(cur_tick - bldc_self_test_tick)) < 0))
          return;

        BldcDisableAllIrq();
        mos_test_index = (BLDC_SELF_TEST_MOS_EXCEPTIONAL)BldcSelfNextMosIndex(mos_test_index);
        if (mos_test_index < SELF_TEST_MOS_INVALID_INDEX) {
          bldc_self_test_step = SELF_TESTING_MOS_PRE;
        }
        else {
          if (bldc_self_test_err_sta) {
            bldc_self_test_step = SELF_TEST_END;
            bldc_self_test_err_sta |= (1 << 31);
          }
          else {
            bldc_self_test_step = SELF_TESTING_HALL_PRE;
            hall_check_cnt = 0;
            hall_check_index = 0xff;
            self_test_send_flag &= (~(1 << SELF_TEST_HALL_INFO));
          }
        }
      break;

      case SELF_TESTING_HALL_PRE:
        if (hall_check_cnt < CHECK_HALL_CNT) {
          hall_check_index = BldcSelfNextHallIndex(hall_check_index);
          BldcStartHallSelfTest(hall_check_index);
          hall_check_cnt++;
          bldc_self_test_tick = cur_tick + BLDC_HALL_SELF_TEST_INTERVAL;
          bldc_self_test_step = SELF_TESTING_HALL_CHECKING;
          hall_check_is_ok = false;
        }
        else {
          bldc_self_test_step = SELF_TEST_END;
        }
      break;

      case SELF_TESTING_HALL_CHECKING:
        hall_check_is_ok = BldcCheckHallSelfTestSta(hall_check_index);
        if ((((int32_t)(cur_tick - bldc_self_test_tick)) > 0) || hall_check_is_ok || hardware_protect_trigger) {
          if (!hall_check_is_ok || hardware_protect_trigger) {
            bldc_self_test_err_sta |= (1 << (hall_check_index + SELF_TEST_MOS_INVALID_INDEX));
          }
          if (hardware_protect_trigger) {
            bldc_self_test_err_sta |= (1 << 30);
            bldc_self_test_step = SELF_TEST_END;
          }
          else {
            bldc_self_test_step = SELF_TESTING_HALL_POST;
          }
          self_test_send_flag |= (1 << SELF_TEST_HALL_INFO);
        }
      break;

      case SELF_TESTING_HALL_POST:
        if (self_test_send_flag & (1 << SELF_TEST_HALL_INFO))
          return;

        if (hall_check_cnt < CHECK_HALL_CNT) {
          bldc_self_test_step = SELF_TESTING_HALL_PRE;
        }
        else {
          bldc_self_test_step = SELF_TEST_END;
        }
      break;

      case SELF_TEST_END:
        BldcDisableAllIrq();
        self_test_send_flag |= (1 << SELF_TEST_END_INFO);
        bldc_self_test_step = SELF_TEST_IDLE;
      break;

      default:
      break;
    }
  }
}

