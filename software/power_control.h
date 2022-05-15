/**
  ******************************************************************************
  * @file  
  * @author liuming
  * @brief 
  * @date     
  ******************************************************************************
  * @attention
  *
  * Copyright (c) LM.
  * All rights reserved.
  *
  ******************************************************************************
  */

#ifndef __POWER_CONTROL_H
#define __POWER_CONTROL_H

/* includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "fytpi_control.h"
/* typedef -------------------------------------------------------------------*/
typedef struct Cap_
{
    PID_ABSOLUTE_T PidAbsCurrent;
    PID_ABSOLUTE_T PidAbsPower;
    
    int16_t tar_power;
    int16_t cur_power;
    int16_t tar_current;
    int16_t cur_current;
    int16_t tar_voltage;
    int16_t cur_voltage;
    
    int16_t output;
}CAP_T;

typedef struct Bat_
{
    PID_ABSOLUTE_T PidAbsCurrent;
    PID_ABSOLUTE_T PidAbsPower;
    
    int16_t tar_power;
    int16_t cur_power;
    int16_t tar_current;
    int16_t cur_current;
    int16_t tar_voltage;
    int16_t cur_voltage;
    
    int16_t output;
}BAT_T;


typedef struct ControlDevice_
{
    CAP_T Cap;
    BAT_T Bat;
    PID_ABSOLUTE_T PidAbsPower;
    PID_ABSOLUTE_T PidAbsCurrent;
    PID_ABSOLUTE_T PidAbsVoltage;
    
    int16_t chassis_power;
    int16_t chassis_power_obser;
    int16_t battery_power;
    int16_t remain_energy;
    int16_t limit_power;
    uint8_t cap_enable;
    
}CONTROL_DEVICE_T;

CONTROL_DEVICE_T* getControlDevice(void);
uint16_t pid_Update(uint16_t target, uint16_t current, PID_ABSOLUTE_T *pid);
void cap_charge_control(void);
void powerControlRun(void);
void capEnable(void);
void capDisable(void);
void capTransmit(void);
void capSwitchScan(void);
void PowerControlRun(void);

#endif
