#ifndef SNAPMAKERMODULES_MARLIN_SRC_MODULE_CNC_HEAD_200W_H_
#define SNAPMAKERMODULES_MARLIN_SRC_MODULE_CNC_HEAD_200W_H_
#include "src/configuration.h"
#include "src/device/bldc_motor.h"
#include "module_base.h"

#define PWM_MODE_CHANGE_RANGE 0.01
#define MOTOR_CTR_INTERVAL_TIME 20
#define MAX_WORK_CURRENT 8500 //mA
#define MAX_WORK_CURRENT_SAMP_CNT 50
#define MIN_PCB_TEMP_LIMIT 0
#define MAX_PCB_TEMP_LIMIT 70
#define MAX_PCB_TEMP_SAMP_CNT 100
#define MIN_MOTOR_TEMP_LIMIT 0
#define MAX_MOTOR_TEMP_LIMIT 95
#define MAX_MOTOR_TEMP_SAMP_CNT 100
#define MIN_MOTOR_VOLTAGE_LIMIT 22
#define MAX_MOTOR_VOLTAGE_LIMIT 26
#define MAX_MOTOR_VOLTAGE_SAMP_CNT 300
#define PENDING(NOW,SOON) ((int32_t)(NOW-(SOON))<0)
#define ELAPSED(NOW,SOON) (!PENDING(NOW,SOON))
#define NOMORE(P, V) (P > V ? P = V : P)

typedef enum {
  CNC_EXCEPTIONAL_STALL = 0,
  CNC_EXCEPTIONAL_H_PROTECT,
  CNC_EXCEPTIONAL_OVERCURRENT,
  CNC_EXCEPTIONAL_P_TEMP,
  CNC_EXCEPTIONAL_M_TEMP,
  CNC_EXCEPTIONAL_V_POWER,
}CNC_EXCEPTIONAL_STATE;

class CncHead200W : public ModuleBase {
  public:
    void Init();
    void HandModule(uint16_t func_id, uint8_t * data, uint8_t data_len);
    void Loop();
    void EmergencyStop();
    void ReportMotorState(void);
    void ReportMotorTemperature(void);
    void ReportMotorPidValue(uint8_t index);
    void SetMotorDirState(uint8_t dir);
    void SetMotorSpeedPower(uint8_t power);
    void SetMotorSpeedRpm(uint16_t rpm);
    void SetMotorCtrMode(bool pid_mode);
    void SetMotorFan(bool ctr);
    void MotorSpeedControlLoop(void);
    void CncHeadReportHWVersion(void);

    // TEST
    void ReportMotorSelfTestState(void);
    void ReportMotorSelfTestHallState(void);
    void ReportMotorSelfTestMosState(void);

  private:
    void ReportConfigResult(uint16_t func_id, uint8_t * data, uint8_t data_len);

  private:
    bool report_msg_ = false;
    uint8_t curtten_cnc_state_ = 0;
    uint32_t time_ = 0;
    uint32_t loop_ctr_time_ = 0;
    uint32_t overcurrent_  = 0;
    uint32_t pcb_protect_cnt_ = 0;
    uint32_t motor_protect_cnt_ = 0;  
    uint32_t voltage_protect_cnt_ = 0; 
    float target_speed_duty_  = 0;
    float temp_pcb_  = 0;
    float temp_motor_  = 0;
    float motor_current_ = 0;
    float motor_voltage_ = 0;
    BldcMotor bldc_module_dev_;
    MOTOR_BLOCK_STATE motor_block_bak_ = MOTOR_BLOCK_NORMAL;
};
#endif 
