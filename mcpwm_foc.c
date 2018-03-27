/*
	Copyright 2016 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "mcpwm_foc.h"
#include "mc_interface.h"
#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "digital_filter.h"
#include "utils.h"
#include "ledpwm.h"
#include "terminal.h"
#include "encoder.h"
#include "commands.h"
#include "timeout.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

// Private types
typedef struct {
	float id_target;
	float iq_target;
	float max_duty;
	float duty_now;
	float phase;
	float i_alpha;
	float i_beta;
	float i_abs;
	float i_abs_filter;
	float i_bus;
	float v_bus;
	float v_alpha;
	float v_beta;
	float mod_d;
	float mod_q;
	float id;
	float iq;
	float id_filter;
	float iq_filter;
	float vd;
	float vq;
	float vd_int;
	float vq_int;
	uint32_t svm_sector;
} motor_state_t;

typedef struct {
	int sample_num;
	float avg_current_tot;
	float avg_voltage_tot;
	bool measure_inductance_now;
	float measure_inductance_duty;
} mc_sample_t;

// Private variables
static volatile mc_configuration *m_conf;
static volatile mc_state m_state;
static volatile mc_control_mode m_control_mode;
static volatile motor_state_t m_motor_state;
static volatile int m_curr0_sum;
static volatile int m_curr1_sum;
static volatile int m_curr_samples;
static volatile int m_curr0_offset;
static volatile int m_curr1_offset;
static volatile bool m_phase_override;
static volatile float m_phase_now_override;
static volatile float m_duty_cycle_set;
static volatile float m_id_set;
static volatile float m_iq_set;
static volatile float m_openloop_speed;
static volatile bool m_dccal_done;
static volatile bool m_output_on;
static volatile float m_pos_pid_set;
static volatile float m_speed_pid_set_rpm;
static volatile float m_phase_now_observer;
static volatile float m_phase_now_observer_override;
static volatile bool m_phase_observer_override;
static volatile float m_phase_now_encoder;
static volatile float m_phase_now_encoder_no_index;
static volatile float m_observer_x1;
static volatile float m_observer_x2;
static volatile float m_pll_phase;
static volatile float m_pll_speed;
static volatile mc_sample_t m_samples;
static volatile int m_tachometer;
static volatile int m_tachometer_abs;
static volatile float last_inj_adc_isr_duration;
static volatile float m_pos_pid_now;
static volatile bool m_init_done;
static volatile float m_gamma_now;

#ifdef HW_HAS_3_SHUNTS
static volatile int m_curr2_sum;
static volatile int m_curr2_offset;
#endif

// Private functions
static void do_dc_cal(void);
static void control_current(volatile motor_state_t *state_m, float dt);
static void run_pid_control_pos(float angle_now, float angle_set, float dt);
static void run_pid_control_speed(float dt);
static void stop_pwm_hw(void);
static void start_pwm_hw(void);

// Threads
static THD_WORKING_AREA(timer_thread_wa, 2048);
static THD_FUNCTION(timer_thread, arg);
static volatile bool timer_thd_stop;

// Macros
#ifdef HW_HAS_3_SHUNTS
#define TIMER_UPDATE_DUTY(duty1, duty2, duty3) \
		TIM1->CR1 |= TIM_CR1_UDIS; \
		TIM1->CCR1 = duty1; \
		TIM1->CCR2 = duty2; \
		TIM1->CCR3 = duty3; \
		TIM1->CR1 &= ~TIM_CR1_UDIS;
#else
#define TIMER_UPDATE_DUTY(duty1, duty2, duty3) \
		TIM1->CR1 |= TIM_CR1_UDIS; \
		TIM1->CCR1 = duty1; \
		TIM1->CCR2 = duty3; \
		TIM1->CCR3 = duty2; \
		TIM1->CR1 &= ~TIM_CR1_UDIS;
#endif

#define TIMER_UPDATE_SAMP(samp) \
		TIM8->CCR1 = samp;

#define TIMER_UPDATE_SAMP_TOP(samp, top) \
		TIM1->CR1 |= TIM_CR1_UDIS; \
		TIM8->CR1 |= TIM_CR1_UDIS; \
		TIM1->ARR = top; \
		TIM8->CCR1 = samp; \
		TIM1->CR1 &= ~TIM_CR1_UDIS; \
		TIM8->CR1 &= ~TIM_CR1_UDIS;

#ifdef HW_HAS_3_SHUNTS
#define TIMER_UPDATE_DUTY_SAMP(duty1, duty2, duty3, samp) \
		TIM1->CR1 |= TIM_CR1_UDIS; \
		TIM8->CR1 |= TIM_CR1_UDIS; \
		TIM1->CCR1 = duty1; \
		TIM1->CCR2 = duty2; \
		TIM1->CCR3 = duty3; \
		TIM8->CCR1 = samp; \
		TIM1->CR1 &= ~TIM_CR1_UDIS; \
		TIM8->CR1 &= ~TIM_CR1_UDIS;
#else
#define TIMER_UPDATE_DUTY_SAMP(duty1, duty2, duty3, samp) \
		TIM1->CR1 |= TIM_CR1_UDIS; \
		TIM8->CR1 |= TIM_CR1_UDIS; \
		TIM1->CCR1 = duty1; \
		TIM1->CCR2 = duty3; \
		TIM1->CCR3 = duty2; \
		TIM8->CCR1 = samp; \
		TIM1->CR1 &= ~TIM_CR1_UDIS; \
		TIM8->CR1 &= ~TIM_CR1_UDIS;
#endif

void mcpwm_foc_init(volatile mc_configuration *configuration) {
	utils_sys_lock_cnt();

	m_init_done = false;

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

	m_conf = configuration;

	// Initialize variables
	m_conf = configuration;
	m_state = MC_STATE_OFF;
	m_control_mode = CONTROL_MODE_NONE;
	m_curr0_sum = 0;
	m_curr1_sum = 0;
	m_curr_samples = 0;
	m_dccal_done = false;
	m_phase_override = false;
	m_phase_now_override = 0.0;
	m_duty_cycle_set = 0.0;
	m_id_set = 0.0;
	m_iq_set = 0.0;
	m_openloop_speed = 0.0;
	m_output_on = false;
	m_pos_pid_set = 0.0;
	m_speed_pid_set_rpm = 0.0;
	m_phase_now_observer = 0.0;
	m_phase_now_observer_override = 0.0;
	m_phase_observer_override = false;
	m_phase_now_encoder = 0.0;
	m_phase_now_encoder_no_index = 0.0;
	m_observer_x1 = 0.0;
	m_observer_x2 = 0.0;
	m_pll_phase = 0.0;
	m_pll_speed = 0.0;
	m_tachometer = 0;
	m_tachometer_abs = 0;
	last_inj_adc_isr_duration = 0;
	m_pos_pid_now = 0.0;
	m_gamma_now = 0.0;
	memset((void*)&m_motor_state, 0, sizeof(motor_state_t));
	memset((void*)&m_samples, 0, sizeof(mc_sample_t));

#ifdef HW_HAS_3_SHUNTS
	m_curr2_sum = 0;
#endif

	TIM_DeInit(TIM1);
	TIM_DeInit(TIM8);
	TIM1->CNT = 0;
	TIM8->CNT = 0;

	// TIM1 clock enable
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	// Time Base configuration
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
	TIM_TimeBaseStructure.TIM_Period = SYSTEM_CORE_CLOCK / (int)m_conf->foc_f_sw;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	// Channel 1, 2 and 3 Configuration in PWM mode
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = TIM1->ARR / 2;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;

	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

	// Automatic Output enable, Break, dead time and lock configuration
	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
	TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
	TIM_BDTRInitStructure.TIM_DeadTime = HW_DEAD_TIME_VALUE;
	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;

	TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);
	TIM_CCPreloadControl(TIM1, ENABLE);
	TIM_ARRPreloadConfig(TIM1, ENABLE);

	/*
	 * ADC!
	 */
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;

	// Clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2 | RCC_APB2Periph_ADC3, ENABLE);

	dmaStreamAllocate(STM32_DMA_STREAM(STM32_DMA_STREAM_ID(2, 4)),
			3,
			(stm32_dmaisr_t)mcpwm_foc_adc_int_handler,
			(void *)0);

	// DMA for the ADC
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC_Value;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC->CDR;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = HW_ADC_CHANNELS;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream4, &DMA_InitStructure);

	// DMA2_Stream0 enable
	DMA_Cmd(DMA2_Stream4, ENABLE);

	// Enable transfer complete interrupt
	DMA_ITConfig(DMA2_Stream4, DMA_IT_TC, ENABLE);

	// ADC Common Init
	// Note that the ADC is running at 42MHz, which is higher than the
	// specified 36MHz in the data sheet, but it works.
	ADC_CommonInitStructure.ADC_Mode = ADC_TripleMode_RegSimult;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	// Channel-specific settings
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Falling;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T8_CC1;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = HW_ADC_NBR_CONV;

	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_ExternalTrigConv = 0;
	ADC_Init(ADC2, &ADC_InitStructure);
	ADC_Init(ADC3, &ADC_InitStructure);

	// Enable Vrefint channel
	ADC_TempSensorVrefintCmd(ENABLE);

	// Enable DMA request after last transfer (Multi-ADC mode)
	ADC_MultiModeDMARequestAfterLastTransferCmd(ENABLE);

	hw_setup_adc_channels();

	// Enable ADC1
	ADC_Cmd(ADC1, ENABLE);

	// Enable ADC2
	ADC_Cmd(ADC2, ENABLE);

	// Enable ADC3
	ADC_Cmd(ADC3, ENABLE);

	// ------------- Timer8 for ADC sampling ------------- //
	// Time Base configuration
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 500;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;
	TIM_OC1Init(TIM8, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM8, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM8, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM8, ENABLE);
	TIM_CCPreloadControl(TIM8, ENABLE);

	// PWM outputs have to be enabled in order to trigger ADC on CCx
	TIM_CtrlPWMOutputs(TIM8, ENABLE);

	// TIM1 Master and TIM8 slave
	TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update);
	TIM_SelectMasterSlaveMode(TIM1, TIM_MasterSlaveMode_Enable);
	TIM_SelectInputTrigger(TIM8, TIM_TS_ITR0);
	TIM_SelectSlaveMode(TIM8, TIM_SlaveMode_Reset);

	// Enable TIM1 and TIM8
	TIM_Cmd(TIM1, ENABLE);
	TIM_Cmd(TIM8, ENABLE);

	// Main Output Enable
	TIM_CtrlPWMOutputs(TIM1, ENABLE);

	// ADC sampling locations
	stop_pwm_hw();

	// Sample intervals. For now they are fixed with voltage samples in the center of V7
	// and current samples in the center of V0
	TIMER_UPDATE_SAMP(MCPWM_FOC_CURRENT_SAMP_OFFSET);

	// Enable CC1 interrupt, which will be fired in V0 and V7
	TIM_ITConfig(TIM8, TIM_IT_CC1, ENABLE);
	nvicEnableVector(TIM8_CC_IRQn, 6);

	utils_sys_unlock_cnt();

	// Calibrate current offset
	ENABLE_GATE();
	DCCAL_OFF();
	do_dc_cal();

	// Various time measurements
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);

	// Time base configuration
	TIM_TimeBaseStructure.TIM_Period = 0xFFFFFFFF;
	TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t)(((SYSTEM_CORE_CLOCK / 2) / 10000000) - 1);
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM12, &TIM_TimeBaseStructure);

	TIM_Cmd(TIM12, ENABLE);

	// Start threads
	timer_thd_stop = false;
	chThdCreateStatic(timer_thread_wa, sizeof(timer_thread_wa), NORMALPRIO, timer_thread, NULL);

	// WWDG configuration
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);
	WWDG_SetPrescaler(WWDG_Prescaler_1);
	WWDG_SetWindowValue(255);
	WWDG_Enable(100);

	m_init_done = true;
}

void mcpwm_foc_deinit(void) {
	m_init_done = false;

	WWDG_DeInit();

	timer_thd_stop = true;

	while (timer_thd_stop) {
		chThdSleepMilliseconds(1);
	}

	TIM_DeInit(TIM1);
	TIM_DeInit(TIM8);
	TIM_DeInit(TIM12);
	ADC_DeInit();
	DMA_DeInit(DMA2_Stream4);
	nvicDisableVector(ADC_IRQn);
	dmaStreamRelease(STM32_DMA_STREAM(STM32_DMA_STREAM_ID(2, 4)));
}

bool mcpwm_foc_init_done(void) {
	return m_init_done;
}

void mcpwm_foc_set_configuration(volatile mc_configuration *configuration) {
	m_conf = configuration;

	m_control_mode = CONTROL_MODE_NONE;
	m_state = MC_STATE_OFF;
	stop_pwm_hw();
	uint32_t top = SYSTEM_CORE_CLOCK / (int)m_conf->foc_f_sw;
	TIMER_UPDATE_SAMP_TOP(MCPWM_FOC_CURRENT_SAMP_OFFSET, top);
}

mc_state mcpwm_foc_get_state(void) {
	return m_state;
}

bool mcpwm_foc_is_dccal_done(void) {
	return m_dccal_done;
}

/**
 * Switch off all FETs.TIM12->CNT = 0;
 */
void mcpwm_foc_stop_pwm(void) {
	mcpwm_foc_set_current(0.0);
}

/**
 * Use duty cycle control. Absolute values less than MCPWM_MIN_DUTY_CYCLE will
 * stop the motor.
 *
 * @param dutyCycle
 * The duty cycle to use.
 */
void mcpwm_foc_set_duty(float dutyCycle) {
	m_control_mode = CONTROL_MODE_DUTY;
	m_duty_cycle_set = dutyCycle;

	if (m_state != MC_STATE_RUNNING) {
		m_state = MC_STATE_RUNNING;
	}
}

/**
 * Use duty cycle control. Absolute values less than MCPWM_MIN_DUTY_CYCLE will
 * stop the motor.
 *
 * WARNING: This function does not use ramping. A too large step with a large motor
 * can destroy hardware.
 *
 * @param dutyCycle
 * The duty cycle to use.
 */
void mcpwm_foc_set_duty_noramp(float dutyCycle) {
	// TODO: Actually do this without ramping
	mcpwm_foc_set_duty(dutyCycle);
}

/**
 * Use PID rpm control. Note that this value has to be multiplied by half of
 * the number of motor poles.
 *
 * @param rpm
 * The electrical RPM goal value to use.
 */
void mcpwm_foc_set_pid_speed(float rpm) {
	m_control_mode = CONTROL_MODE_SPEED;
	m_speed_pid_set_rpm = rpm;

	if (m_state != MC_STATE_RUNNING) {
		m_state = MC_STATE_RUNNING;
	}
}

/**
 * Use PID position control. Note that this only works when encoder support
 * is enabled.
 *
 * @param pos
 * The desired position of the motor in degrees.
 */
void mcpwm_foc_set_pid_pos(float pos) {
	m_control_mode = CONTROL_MODE_POS;
	m_pos_pid_set = pos;

	if (m_state != MC_STATE_RUNNING) {
		m_state = MC_STATE_RUNNING;
	}
}

/**
 * Use current control and specify a goal current to use. The sign determines
 * the direction of the torque. Absolute values less than
 * conf->cc_min_current will release the motor.
 *
 * @param current
 * The current to use.
 */
void mcpwm_foc_set_current(float current) {
	if (fabsf(current) < m_conf->cc_min_current) {
		m_control_mode = CONTROL_MODE_NONE;
		m_state = MC_STATE_OFF;
		stop_pwm_hw();
		return;
	}

	m_control_mode = CONTROL_MODE_CURRENT;
	m_iq_set = current;

	if (m_state != MC_STATE_RUNNING) {
		m_state = MC_STATE_RUNNING;
	}
}

/**
 * Brake the motor with a desired current. Absolute values less than
 * conf->cc_min_current will release the motor.
 *
 * @param current
 * The current to use. Positive and negative values give the same effect.
 */
void mcpwm_foc_set_brake_current(float current) {
	if (fabsf(current) < m_conf->cc_min_current) {
		m_control_mode = CONTROL_MODE_NONE;
		m_state = MC_STATE_OFF;
		stop_pwm_hw();
		return;
	}

	m_control_mode = CONTROL_MODE_CURRENT_BRAKE;
	m_iq_set = current;

	if (m_state != MC_STATE_RUNNING) {
		m_state = MC_STATE_RUNNING;
	}
}

/**
 * Apply a fixed static current vector in open loop to emulate an electric
 * handbrake.
 *
 * @param current
 * The brake current to use.
 */
void mcpwm_foc_set_handbrake(float current) {
	if (fabsf(current) < m_conf->cc_min_current) {
		m_control_mode = CONTROL_MODE_NONE;
		m_state = MC_STATE_OFF;
		stop_pwm_hw();
		return;
	}

	m_control_mode = CONTROL_MODE_HANDBRAKE;
	m_iq_set = current;

	if (m_state != MC_STATE_RUNNING) {
		m_state = MC_STATE_RUNNING;
	}
}

/**
 * Produce an openloop rotating current.
 *
 * @param current
 * The current to use.
 *
 * @param rpm
 * The RPM to use.
 */
void mcpwm_foc_set_openloop(float current, float rpm) {
	if (fabsf(current) < m_conf->cc_min_current) {
		m_control_mode = CONTROL_MODE_NONE;
		m_state = MC_STATE_OFF;
		stop_pwm_hw();
		return;
	}

	utils_truncate_number(&current, -m_conf->l_current_max, m_conf->l_current_max);

	m_control_mode = CONTROL_MODE_OPENLOOP;
	m_iq_set = current;
	m_openloop_speed = rpm * ((2.0 * M_PI) / 60.0);

	if (m_state != MC_STATE_RUNNING) {
		m_state = MC_STATE_RUNNING;
	}
}

float mcpwm_foc_get_duty_cycle_set(void) {
	return m_duty_cycle_set;
}

float mcpwm_foc_get_duty_cycle_now(void) {
	return m_motor_state.duty_now;
}

float mcpwm_foc_get_pid_pos_set(void) {
	return m_pos_pid_set;
}

float mcpwm_foc_get_pid_pos_now(void) {
	return m_pos_pid_now;
}

/**
 * Get the current switching frequency.
 *
 * @return
 * The switching frequency in Hz.
 */
float mcpwm_foc_get_switching_frequency_now(void) {
	return m_conf->foc_f_sw;
}

/**
 * Get the current sampling frequency.
 *
 * @return
 * The sampling frequency in Hz.
 */
float mcpwm_foc_get_sampling_frequency_now(void) {
#ifdef HW_HAS_PHASE_SHUNTS
	if (m_conf->foc_sample_v0_v7) {
		return m_conf->foc_f_sw;
	} else {
		return m_conf->foc_f_sw / 2.0;
	}
#else
	return m_conf->foc_f_sw / 2.0;
#endif
}

/**
 * Calculate the current RPM of the motor. This is a signed value and the sign
 * depends on the direction the motor is rotating in. Note that this value has
 * to be divided by half the number of motor poles.
 *
 * @return
 * The RPM value.
 */
float mcpwm_foc_get_rpm(void) {
	return m_pll_speed / ((2.0 * M_PI) / 60.0);
}

/**
 * Get the motor current. The sign of this value will
 * represent whether the motor is drawing (positive) or generating
 * (negative) current. This is the q-axis current which produces torque.
 *
 * @return
 * The motor current.
 */
float mcpwm_foc_get_tot_current(void) {
	return SIGN(m_motor_state.vq) * m_motor_state.iq;
}

/**
 * Get the filtered motor current. The sign of this value will
 * represent whether the motor is drawing (positive) or generating
 * (negative) current. This is the q-axis current which produces torque.
 *
 * @return
 * The filtered motor current.
 */
float mcpwm_foc_get_tot_current_filtered(void) {
	return SIGN(m_motor_state.vq) * m_motor_state.iq_filter;
}

/**
 * Get the magnitude of the motor current, which includes both the
 * D and Q axis.
 *
 * @return
 * The magnitude of the motor current.
 */
float mcpwm_foc_get_abs_motor_current(void) {
	return m_motor_state.i_abs;
}

/**
 * Get the magnitude of the motor voltage.
 *
 * @return
 * The magnitude of the motor voltage.
 */
float mcpwm_foc_get_abs_motor_voltage(void) {
	const float vd_tmp = m_motor_state.vd;
	const float vq_tmp = m_motor_state.vq;
	return sqrtf(SQ(vd_tmp) + SQ(vq_tmp));
}

/**
 * Get the filtered magnitude of the motor current, which includes both the
 * D and Q axis.
 *
 * @return
 * The magnitude of the motor current.
 */
float mcpwm_foc_get_abs_motor_current_filtered(void) {
	return m_motor_state.i_abs_filter;
}

/**
 * Get the motor current. The sign of this value represents the direction
 * in which the motor generates torque.
 *
 * @return
 * The motor current.
 */
float mcpwm_foc_get_tot_current_directional(void) {
	return m_motor_state.iq;
}

/**
 * Get the filtered motor current. The sign of this value represents the
 * direction in which the motor generates torque.
 *
 * @return
 * The filtered motor current.
 */
float mcpwm_foc_get_tot_current_directional_filtered(void) {
	return m_motor_state.iq_filter;
}

/**
 * Get the direct axis motor current.
 *
 * @return
 * The D axis current.
 */
float mcpwm_foc_get_id(void) {
	return m_motor_state.id;
}

/**
 * Get the quadrature axis motor current.
 *
 * @return
 * The Q axis current.
 */
float mcpwm_foc_get_iq(void) {
	return m_motor_state.iq;
}

/**
 * Get the input current to the motor controller.
 *
 * @return
 * The input current.
 */
float mcpwm_foc_get_tot_current_in(void) {
	return m_motor_state.i_bus;
}

/**
 * Get the filtered input current to the motor controller.
 *
 * @return
 * The filtered input current.
 */
float mcpwm_foc_get_tot_current_in_filtered(void) {
	return m_motor_state.i_bus; // TODO: Calculate filtered current?
}

/**
 * Read the number of steps the motor has rotated. This number is signed and
 * will return a negative number when the motor is rotating backwards.
 *
 * @param reset
 * If true, the tachometer counter will be reset after this call.
 *
 * @return
 * The tachometer value in motor steps. The number of motor revolutions will
 * be this number divided by (3 * MOTOR_POLE_NUMBER).
 */
int mcpwm_foc_get_tachometer_value(bool reset) {
	int val = m_tachometer;

	if (reset) {
		m_tachometer = 0;
	}

	return val;
}

/**
 * Read the absolute number of steps the motor has rotated.
 *
 * @param reset
 * If true, the tachometer counter will be reset after this call.
 *
 * @return
 * The tachometer value in motor steps. The number of motor revolutions will
 * be this number divided by (3 * MOTOR_POLE_NUMBER).
 */
int mcpwm_foc_get_tachometer_abs_value(bool reset) {
	int val = m_tachometer_abs;

	if (reset) {
		m_tachometer_abs = 0;
	}

	return val;
}

/**
 * Read the motor phase.
 *
 * @return
 * The phase angle in degrees.
 */
float mcpwm_foc_get_phase(void) {
	float angle = m_motor_state.phase * (180.0 / M_PI);
	utils_norm_angle(&angle);
	return angle;
}

/**
 * Read the phase that the observer has calculated.
 *
 * @return
 * The phase angle in degrees.
 */
float mcpwm_foc_get_phase_observer(void) {
	float angle = m_phase_now_observer * (180.0 / M_PI);
	utils_norm_angle(&angle);
	return angle;
}

/**
 * Read the phase from based on the encoder.
 *
 * @return
 * The phase angle in degrees.
 */
float mcpwm_foc_get_phase_encoder(void) {
	float angle = m_phase_now_encoder * (180.0 / M_PI);
	utils_norm_angle(&angle);
	return angle;
}

float mcpwm_foc_get_vd(void) {
	return m_motor_state.vd;
}

float mcpwm_foc_get_vq(void) {
	return m_motor_state.vq;
}

void mcpwm_foc_print_state(void) {
	commands_printf("Mod d:        %.2f", (double)m_motor_state.mod_d);
	commands_printf("Mod q:        %.2f", (double)m_motor_state.mod_q);
	commands_printf("Duty:         %.2f", (double)m_motor_state.duty_now);
	commands_printf("Vd:           %.2f", (double)m_motor_state.vd);
	commands_printf("Vq:           %.2f", (double)m_motor_state.vq);
	commands_printf("Phase:        %.2f", (double)m_motor_state.phase);
	commands_printf("V_alpha:      %.2f", (double)m_motor_state.v_alpha);
	commands_printf("V_beta:       %.2f", (double)m_motor_state.v_beta);
	commands_printf("id:           %.2f", (double)m_motor_state.id);
	commands_printf("iq:           %.2f", (double)m_motor_state.iq);
	commands_printf("id_filter:    %.2f", (double)m_motor_state.id_filter);
	commands_printf("iq_filter:    %.2f", (double)m_motor_state.iq_filter);
	commands_printf("id_target:    %.2f", (double)m_motor_state.id_target);
	commands_printf("iq_target:    %.2f", (double)m_motor_state.iq_target);
	commands_printf("i_abs:        %.2f", (double)m_motor_state.i_abs);
	commands_printf("i_abs_filter: %.2f", (double)m_motor_state.i_abs_filter);
	commands_printf("Obs_x1:       %.2f", (double)m_observer_x1);
	commands_printf("Obs_x2:       %.2f", (double)m_observer_x2);
}

float mcpwm_foc_get_last_inj_adc_isr_duration(void) {
	return last_inj_adc_isr_duration;
}

void mcpwm_foc_tim_sample_int_handler(void) {
	if (m_init_done) {
		// Generate COM event here for synchronization
		TIM_GenerateEvent(TIM1, TIM_EventSource_COM);
	}
}

void mcpwm_foc_adc_int_handler(void *p, uint32_t flags) {
	(void)p;
	(void)flags;

	TIM12->CNT = 0;

	// Reset the watchdog
	WWDG_SetCounter(100);

	int curr0 = ADC_Value[ADC_IND_CURR1];
	int curr1 = ADC_Value[ADC_IND_CURR2];

#ifdef HW_HAS_3_SHUNTS
	int curr2 = ADC_Value[ADC_IND_CURR3];
#endif

	m_curr0_sum += curr0;
	m_curr1_sum += curr1;
#ifdef HW_HAS_3_SHUNTS
	m_curr2_sum += curr2;
#endif

	curr0 -= m_curr0_offset;
	curr1 -= m_curr1_offset;
#ifdef HW_HAS_3_SHUNTS
	curr2 -= m_curr2_offset;
#endif

	m_curr_samples++;

	ADC_curr_norm_value[0] = curr0;
	ADC_curr_norm_value[1] = curr1;
#ifdef HW_HAS_3_SHUNTS
	ADC_curr_norm_value[2] = curr2;
#else
	ADC_curr_norm_value[2] = -(ADC_curr_norm_value[0] + ADC_curr_norm_value[1]);
#endif


	float ia = ADC_curr_norm_value[0] * FAC_CURRENT;
	float ib = ADC_curr_norm_value[1] * FAC_CURRENT;
//	float ic = -(ia + ib);

	const float dt = 1.0 / (m_conf->foc_f_sw / 2.0);

	UTILS_LP_FAST(m_motor_state.v_bus, GET_INPUT_VOLTAGE(), 0.1);

	float enc_ang = 0.0;
	if (encoder_is_configured()) {
		enc_ang = encoder_read_deg();
		float phase_tmp = enc_ang;
		if (m_conf->foc_encoder_inverted) {
			phase_tmp = 360.0 - phase_tmp;
		}
		phase_tmp *= m_conf->foc_encoder_ratio;
		phase_tmp -= m_conf->foc_encoder_offset;
		utils_norm_angle((float*)&phase_tmp);
		m_phase_now_encoder = phase_tmp * (M_PI / 180.0);
	}

	if (m_state == MC_STATE_RUNNING) {
		control_current(&m_motor_state, dt);
	} else {

	}

	// Track position control angle
	// TODO: Have another look at this.
	float angle_now = 0.0;
	if (encoder_is_configured()) {
		angle_now = enc_ang;
	}

	if (m_conf->p_pid_ang_div > 0.98 && m_conf->p_pid_ang_div < 1.02) {
		m_pos_pid_now = angle_now;
	} else {
		static float angle_last = 0.0;
		float diff_f = utils_angle_difference(angle_now, angle_last);
		angle_last = angle_now;
		m_pos_pid_now += diff_f / m_conf->p_pid_ang_div;
		utils_norm_angle((float*)&m_pos_pid_now);
	}

	// Run position control
	if (m_state == MC_STATE_RUNNING) {
		run_pid_control_pos(m_pos_pid_now, m_pos_pid_set, dt);
	}

	// MCIF handler
	mc_interface_mc_timer_isr();

	last_inj_adc_isr_duration = (float) TIM12->CNT / 10000000.0;
}

// Private functions

static THD_FUNCTION(timer_thread, arg) {
	(void)arg;

	chRegSetThreadName("mcpwm_foc timer");

	systime_t time = chVTGetSystemTime();

	for(;;) {

		time += MS2ST(1); //1 ms timing
		const float dt = 0.001;

		if (timer_thd_stop) {
			timer_thd_stop = false;
			return;
		}

		// Samples
		if (m_state == MC_STATE_RUNNING) {
			const volatile float vd_tmp = m_motor_state.vd;
			const volatile float vq_tmp = m_motor_state.vq;
			const volatile float id_tmp = m_motor_state.id;
			const volatile float iq_tmp = m_motor_state.iq;

			m_samples.avg_current_tot += sqrtf(SQ(id_tmp) + SQ(iq_tmp));
			m_samples.avg_voltage_tot += sqrtf(SQ(vd_tmp) + SQ(vq_tmp));
			m_samples.sample_num++;
		}

		run_pid_control_speed(dt);
		chThdSleepUntil(time);
	}

}

static void do_dc_cal(void) {
	DCCAL_ON();

	// Wait max 5 seconds
	int cnt = 0;
	while(IS_DRV_FAULT()){
		chThdSleepMilliseconds(1);
		cnt++;
		if (cnt > 5000) {
			break;
		}
	};

	chThdSleepMilliseconds(1000);
	m_curr0_sum = 0;
	m_curr1_sum = 0;
#ifdef HW_HAS_3_SHUNTS
	m_curr2_sum = 0;
#endif
	m_curr_samples = 0;
	while(m_curr_samples < 4000) {};
	m_curr0_offset = m_curr0_sum / m_curr_samples;
	m_curr1_offset = m_curr1_sum / m_curr_samples;
#ifdef HW_HAS_3_SHUNTS
	m_curr2_offset = m_curr2_sum / m_curr_samples;
#endif
	DCCAL_OFF();
	m_dccal_done = true;
}

/**
 * Run the current control loop.
 *
 * @param state_m
 * The motor state.
 *
 * Parameters that shall be set before calling this function:
 * id_target
 * iq_target
 * max_duty
 * phase
 * i_alpha
 * i_beta
 * v_bus
 *
 * Parameters that will be updated in this function:
 * i_bus
 * i_abs
 * i_abs_filter
 * v_alpha
 * v_beta
 * mod_d
 * mod_q
 * id
 * iq
 * id_filter
 * iq_filter
 * vd
 * vq
 * vd_int
 * vq_int
 * svm_sector
 *
 * @param dt
 * The time step in seconds.
 */
static void control_current(volatile motor_state_t *state_m, float dt) {

	float dutyCycle = fabsf(m_duty_cycle_set);
	utils_truncate_number(&dutyCycle, m_conf->l_min_duty, m_conf->l_max_duty);

	float switching_frequency_now = (float)m_conf->m_bldc_f_sw_min * (1.0 - fabsf(dutyCycle)) +
					m_conf->m_bldc_f_sw_max * fabsf(dutyCycle);

	// Set output (HW Dependent)
	uint32_t duty1, duty2, duty3, top;

	top = SYSTEM_CORE_CLOCK / (int)switching_frequency_now;
	TIMER_UPDATE_SAMP_TOP(MCPWM_FOC_CURRENT_SAMP_OFFSET, top);

	top = TIM1->ARR;
	duty1 = (uint16_t)((float)top * dutyCycle);
	duty2 = 0;
	duty3 = 0;
	TIMER_UPDATE_DUTY(duty1, duty2, duty3);

	if (!m_output_on) {
		start_pwm_hw();
	}
}

static void run_pid_control_pos(float angle_now, float angle_set, float dt) {
	static float i_term = 0;
	static float prev_error = 0;
	float p_term;
	float d_term;

	// PID is off. Return.
	if (m_control_mode != CONTROL_MODE_POS) {
		i_term = 0;
		prev_error = 0;
		return;
	}

	// Compute parameters
	float error = utils_angle_difference(angle_set, angle_now);

	if (encoder_is_configured()) {
		if (m_conf->foc_encoder_inverted) {
			error = -error;
		}
	}

	p_term = error * m_conf->p_pid_kp;
	i_term += error * (m_conf->p_pid_ki * dt);

	// Average DT for the D term when the error does not change. This likely
	// happens at low speed when the position resolution is low and several
	// control iterations run without position updates.
	// TODO: Are there problems with this approach?
	static float dt_int = 0.0;
	dt_int += dt;
	if (error == prev_error) {
		d_term = 0.0;
	} else {
		d_term = (error - prev_error) * (m_conf->p_pid_kd / dt_int);
		dt_int = 0.0;
	}

	// // Filter D
	// static float d_filter = 0.0;
	// UTILS_LP_FAST(d_filter, d_term, m_conf->p_pid_kd_filter);
	// d_term = d_filter;


	// I-term wind-up protection
	utils_truncate_number_abs(&p_term, 1.0);
	utils_truncate_number_abs(&i_term, 1.0 - fabsf(p_term));

	// Store previous error
	prev_error = error;

	// Calculate output
	float output = p_term + i_term + d_term;
	utils_truncate_number(&output, -1.0, 1.0);

	if (encoder_is_configured()) {
		if (encoder_index_found()) {
			m_iq_set = output * m_conf->lo_current_max;
		} else {
			// Rotate the motor with 40 % power until the encoder index is found.
			m_iq_set = 0.4 * m_conf->lo_current_max;
		}
	} else {
		m_iq_set = output * m_conf->lo_current_max;
	}
}

static void run_pid_control_speed(float dt) {
	static float i_term = 0.0;
	static float prev_error = 0.0;
	float p_term;
	float d_term;

	// PID is off. Return.
	if (m_control_mode != CONTROL_MODE_SPEED) {
		i_term = 0.0;
		prev_error = 0.0;
		return;
	}

	const float rpm = mcpwm_foc_get_rpm();
	float error = m_speed_pid_set_rpm - rpm;

	// Too low RPM set. Reset state and return.
	if (fabsf(m_speed_pid_set_rpm) < m_conf->s_pid_min_erpm) {
		i_term = 0.0;
		prev_error = error;
		return;
	}

	// Compute parameters
	p_term = error * m_conf->s_pid_kp * (1.0 / 20.0);
	i_term += error * (m_conf->s_pid_ki * dt) * (1.0 / 20.0);
	d_term = (error - prev_error) * (m_conf->s_pid_kd / dt) * (1.0 / 20.0);

	// // Filter D
	// static float d_filter = 0.0;
	// UTILS_LP_FAST(d_filter, d_term, m_conf->s_pid_kd_filter);
	// d_term = d_filter;

	// I-term wind-up protection
	utils_truncate_number(&i_term, -1.0, 1.0);

	// Store previous error
	prev_error = error;

	// Calculate output
	float output = p_term + i_term + d_term;
	utils_truncate_number(&output, -1.0, 1.0);

	// Optionally disable braking
	if (!m_conf->s_pid_allow_braking) {
		if (rpm > 0.0 && output < 0.0) {
			output = 0.0;
		}

		if (rpm < 0.0 && output > 0.0) {
			output = 0.0;
		}
	}

	m_iq_set = output * m_conf->lo_current_max;
}

static void stop_pwm_hw(void) {
	TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_InActive);
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);

	TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_InActive);
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);

	TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_InActive);
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);

	TIM_GenerateEvent(TIM1, TIM_EventSource_COM);

#ifdef HW_HAS_DRV8313
	DISABLE_BR();
#endif
	m_output_on = false;
}

static void start_pwm_hw(void) {
	TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);

	TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1);
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);

	TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);

	// Generate COM event in ADC interrupt to get better synchronization
//	TIM_GenerateEvent(TIM1, TIM_EventSource_COM);

#ifdef HW_HAS_DRV8313
	ENABLE_BR();
#endif
	m_output_on = true;
}
