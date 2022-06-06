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

#include "power_control.h"
#include "main.h"
#include "fytpi_math.h"
#include "can.h"

CONTROL_DEVICE_T ControlDevice = {0};

/**
  * @brief  对外接口
  * @param  void
  * @retval void
  * @attention 
  */
CONTROL_DEVICE_T* getControlDevice(void)
{
    return &ControlDevice;
}

/**
  * @brief  pid初始化
  * @param  void
  * @retval void
  * @attention 
  */
void powerControlInit()
{
    pidAbsoluteInit(&ControlDevice.PidAbsCurrent,0.05,0,0,0,0);
    pidAbsoluteInit(&ControlDevice.PidAbsPower,0.05,0,0,0,0);
    pidAbsoluteInit(&ControlDevice.PidAbsVoltage,0.005,0,0,0,0);
    
    
    pidAbsoluteInit(&ControlDevice.Cap.PidAbsPower,2,0.2,0,0,0);
}

/**
  * @brief  
  * @param  void
  * @retval void
  * @attention 
  */
uint16_t pwm2_output=1;
float pwm2_err=2.0f;
void cap_charge_control()
{
    uint16_t pwm_lim = 4198;
	powerControlInit();
    capSwitchScan();
    
    if (ControlDevice.Cap.cur_voltage<1000) 
	{
	  pwm2_err=pidAbsoluteUpdate(8000.0f, (float)ControlDevice.Cap.cur_current,&ControlDevice.PidAbsCurrent);
	}
	if (ControlDevice.Cap.cur_voltage>=1000)
	{
	  pwm2_err=pidAbsoluteUpdate(800.0f, (float)ControlDevice.Cap.cur_power,&ControlDevice.PidAbsPower);
	}
	pwm2_err = constrainFloat(pwm2_err, -100, 1);
	pwm2_output += pwm2_err;

    pwm_lim = constrainUint16(4198*ControlDevice.Cap.cur_voltage/2400 + 3500, 0, 3780);
	pwm2_output = constrainUint16(pwm2_output, 0, pwm_lim);
//    pwm2_output = 800;
	TIM2->CCR1=pwm2_output;
}

//uint16_t final_out;
float err_out;
float err_out2;
float err_out3;
uint16_t transfer;
void PowerControlRun(void)
{
    powerControlInit();
//	if (ControlDevice.Cap.cur_voltage<1200) ControlDevice.cap_enable=1;
	  capTransmit();
    capSwitchScan();
  err_out2 = 0.01f*(ControlDevice.limit_power + ControlDevice.remain_energy*0.01f - ControlDevice.Bat.cur_power);
	err_out2 = constrainFloat(err_out2, -40, 8);
	ControlDevice.Cap.tar_power +=err_out2;
	ControlDevice.Cap.tar_power = constrainUint16(ControlDevice.Cap.tar_power, 0, ControlDevice.limit_power-900);
	ControlDevice.Cap.tar_power = constrainUint16(ControlDevice.Cap.tar_power, 0, 12000);
//    ControlDevice.chassis_power_obser 
//    = 0.5f*ControlDevice.chassis_power + 0.5f*ControlDevice.battery_power - ControlDevice.Cap.cur_power;
//    
//    ControlDevice.Cap.tar_power 
//    = ControlDevice.limit_power + ControlDevice.remain_energy*0.01f - ControlDevice.chassis_power_obser;
    transfer=ControlDevice.Cap.tar_power/9;
	  if (ControlDevice.Cap.cur_voltage<transfer) 
	{
		err_out = 0.07f* (9000-ControlDevice.Cap.cur_current);
	}
	 if (ControlDevice.Cap.cur_voltage>=transfer)
	 {
		 if(ControlDevice.Cap.tar_power < ControlDevice.Cap.cur_power)
		 {
		 err_out3= -(ControlDevice.Cap.tar_power/100-ControlDevice.Cap.cur_power/100)^2;
		 }
		 else err_out3=0;
	   err_out = 0.7f*(ControlDevice.Cap.tar_power-ControlDevice.Cap.cur_power);
	 }
	 err_out = constrainFloat(err_out, -7, 3);
	 err_out3 = constrainFloat(err_out3, -1000, 0);
	 ControlDevice.Cap.output += err_out+0.07f*err_out3;
	 ControlDevice.Cap.output = constrainUint16(ControlDevice.Cap.output, 0, 4120);
	 if (ControlDevice.remain_energy<8) ControlDevice.Cap.output=500;
	if (ControlDevice.Bat.cur_voltage<100) 
	{
		ControlDevice.Cap.tar_power = ControlDevice.limit_power-900;
		ControlDevice.Cap.output = 1;
	}
//    final_out
//    = pidAbsoluteUpdate(ControlDevice.Cap.tar_power, 
//                        ControlDevice.Cap.cur_power,
//                        &ControlDevice.Cap.PidAbsPower);
//	  final_out = constrainUint16(final_out, 0, 4198);
//    ControlDevice.Cap.output = rampUint16(final_out, ControlDevice.Cap.output, 1);

//    ControlDevice.Cap.output 
//    = constrainUint16(ControlDevice.Cap.output, 0, 2098);
    
    TIM2->CCR2 = ControlDevice.Cap.output/2;
	
}

uint8_t can_data[8];
void capTransmit(void)
{
	can_data[0]=ControlDevice.cap_enable;
	can_data[1]=ControlDevice.Cap.cur_voltage>>8;
	can_data[2]=ControlDevice.Cap.cur_voltage;
	CAN1_Transmit(0x211,can_data);
}



void capSwitchScan(void)
{
    switch(ControlDevice.cap_enable)
    {
        case 0x01:
            capDisable();
        break;
        
        case 0x00:
            capEnable();
        break;
        
        default:
            capEnable();
        break;
    }
    
    if (ControlDevice.Cap.cur_voltage < 800)
    {
        capDisable();
    }
}

uint8_t cap_switch = 0;
void capEnable(void)
{
    if (cap_switch == 0)
    {
        cap_switch = 1;
        HAL_GPIO_WritePin(BAT_OUT_GPIO_Port, BAT_OUT_Pin, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(CAP_OUT_GPIO_Port, CAP_OUT_Pin, GPIO_PIN_SET);
        HAL_Delay(1);
    }    
}

void capDisable(void)
{
    if (cap_switch == 1)
    {
			cap_switch = 0;
        HAL_GPIO_WritePin(CAP_OUT_GPIO_Port, CAP_OUT_Pin, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(BAT_OUT_GPIO_Port, BAT_OUT_Pin, GPIO_PIN_SET);
        HAL_Delay(1);
    }
}

