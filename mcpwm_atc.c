#include "mcpwm_atc.h"
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

// Structs
typedef struct {
	volatile bool updated;
	volatile unsigned int top;
	volatile unsigned int duty;
	volatile unsigned int val_sample;
	volatile unsigned int curr1_sample;
	volatile unsigned int curr2_sample;
#ifdef HW_HAS_3_SHUNTS
	volatile unsigned int curr3_sample;
#endif
} mc_timer_struct;

// Private variables
static volatile mc_configuration *m_conf;
static volatile mc_state m_state;
static volatile mc_control_mode m_control_mode;
static volatile float switching_frequency_now;
static volatile float m_pos_pid_set;
static volatile float m_pos_pid_now;
static volatile int m_curr0_sum;
static volatile int m_curr1_sum;
static volatile int m_curr_samples;
static volatile int m_curr0_offset;
static volatile int m_curr1_offset;
static volatile bool m_output_on;
static volatile bool m_init_done;
static volatile bool m_dccal_done;
static volatile float last_inj_adc_isr_duration;
static volatile float last_current_sample;
static volatile float last_current_sample_filtered;
static volatile float dutycycle_now;

#ifdef HW_HAS_3_SHUNTS
static volatile int m_curr2_sum;
static volatile int m_curr2_offset;
#endif

// Current FIR filter
#define CURR_FIR_TAPS_BITS		4
#define CURR_FIR_LEN			(1 << CURR_FIR_TAPS_BITS)
#define CURR_FIR_FCUT			0.15
static volatile float current_fir_coeffs[CURR_FIR_LEN];
static volatile float current_fir_samples[CURR_FIR_LEN];
static volatile int current_fir_index = 0;

// Private functions
static void stop_pwm_hw(void);
static void start_pwm_hw(void);
static void do_dc_cal(void);
static float do_pid_pos_control(void);

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

// Public Function definitions

// Initialize motor configuration
void mcpwm_atc_init(volatile mc_configuration *configuration){
	utils_sys_lock_cnt();
	
	m_init_done = false;

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

	// Initialize variables
	m_conf = configuration;
	m_state = MC_STATE_OFF;
	m_control_mode = CONTROL_MODE_NONE;
	m_pos_pid_set = 0.0;
	m_pos_pid_now = 0.0;
	m_output_on = false;
	m_dccal_done = false;
	last_inj_adc_isr_duration = 0;
	switching_frequency_now = m_conf->foc_f_sw;
	last_current_sample = 0;
	last_current_sample_filtered = 0;
	dutycycle_now = 0;

	// Create current FIR filter
	filter_create_fir_lowpass((float*)current_fir_coeffs, CURR_FIR_FCUT, CURR_FIR_TAPS_BITS, 1);

	// Reset timers
	TIM_DeInit(TIM1);
	TIM_DeInit(TIM8);
	TIM1->CNT = 0;
	TIM8->CNT = 0;

	// TIM1 clock enable
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	// Time Base configuration
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	//TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = SYSTEM_CORE_CLOCK / (int)switching_frequency_now;
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
			(stm32_dmaisr_t)mcpwm_atc_adc_int_handler,
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
	TIMER_UPDATE_SAMP(MCPWM_ATC_CURRENT_SAMP_OFFSET);

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
void mcpwm_atc_deinit(void) {
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

bool mcpwm_atc_init_done(void) {
	return m_init_done;
}

void mcpwm_atc_set_configuration(volatile mc_configuration *configuration) {
	m_conf = configuration;

	m_control_mode = CONTROL_MODE_NONE;
	m_state = MC_STATE_OFF;
	stop_pwm_hw();
//	uint32_t top = SYSTEM_CORE_CLOCK / (int)m_conf->foc_f_sw;
//	TIMER_UPDATE_SAMP_TOP(MCPWM_FOC_CURRENT_SAMP_OFFSET, top);
}

mc_state mcpwm_atc_get_state(void) {
	return m_state;
}

bool mcpwm_atc_is_dccal_done(void) {
	return m_dccal_done;
}

/**
 * Switch off all FETs
 */
void mcpwm_atc_stop_pwm(void) {
	m_control_mode = CONTROL_MODE_NONE;
	m_state = MC_STATE_OFF;
	stop_pwm_hw();
}

/**
 * Use PID position control. Note that this only works when encoder support
 * is enabled.
 *
 * @param pos
 * The desired position of the motor in degrees.
 */
void mcpwm_atc_set_pid_pos(float pos) {
	m_control_mode = CONTROL_MODE_POS;
	m_pos_pid_set = pos;

	if (m_state != MC_STATE_RUNNING) {
		m_state = MC_STATE_RUNNING;
	}
}

/**
 * Use duty cycle control. Absolute values less than MCPWM_MIN_DUTY_CYCLE will
 * stop the motor.
 *
 * @param dutyCycle
 * The duty cycle to use.
 */
void mcpwm_atc_set_duty(float dutyCycle) {
	m_control_mode = CONTROL_MODE_DUTY;	

	if (m_state != MC_STATE_RUNNING) {
		m_state = MC_STATE_RUNNING;
	}

	dutyCycle = fabsf(dutyCycle);

	utils_truncate_number(&dutyCycle, m_conf->l_min_duty, m_conf->l_max_duty);

	dutyCycle = 0.5;

	// Adjust switching frequency for good resolution
	switching_frequency_now = (float)m_conf->m_bldc_f_sw_min * (1.0 - dutyCycle) +
		(float)m_conf->m_bldc_f_sw_max * dutyCycle;

	switching_frequency_now = m_conf->foc_f_sw;

	// Set top of counter
	uint16_t top = SYSTEM_CORE_CLOCK / (int)switching_frequency_now;

	// Set switching register for line 1
	uint16_t duty1 = (uint16_t)((float)top * dutyCycle);

	// Update
	utils_sys_lock_cnt();
	TIMER_UPDATE_SAMP_TOP(MCPWM_ATC_CURRENT_SAMP_OFFSET, top);
	TIMER_UPDATE_DUTY(duty1, 0, 0);
	utils_sys_unlock_cnt();
	
// 	uint16_t positive_oc_mode = TIM_OCMode_PWM1;
// 	uint16_t negative_oc_mode = TIM_OCMode_Inactive;

// 	uint16_t positive_highside = TIM_CCx_Enable;
// 	uint16_t positive_lowside = TIM_CCxN_Enable;

// 	uint16_t negative_highside = TIM_CCx_Enable;
// 	uint16_t negative_lowside = TIM_CCxN_Enable;

// #ifdef HW_HAS_DRV8313
// 	DISABLE_BR1();
// 	ENABLE_BR2();
// 	ENABLE_BR3();
// #endif
// 	// 0
// 	TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_Inactive);
// 	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
// 	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);

// 	// +
// 	TIM_SelectOCxM(TIM1, TIM_Channel_2, positive_oc_mode);
// 	TIM_CCxCmd(TIM1, TIM_Channel_2, positive_highside);
// 	TIM_CCxNCmd(TIM1, TIM_Channel_2, positive_lowside);

// 	// -
// 	TIM_SelectOCxM(TIM1, TIM_Channel_3, negative_oc_mode);
// 	TIM_CCxCmd(TIM1, TIM_Channel_3, negative_highside);
// 	TIM_CCxNCmd(TIM1, TIM_Channel_3, negative_lowside);

	if(!m_output_on){
		start_pwm_hw();
	}
	commands_printf("Duty set: %.2f %i %i %i",dutyCycle, duty1,top,m_init_done);
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
void mcpwm_atc_set_duty_noramp(float dutyCycle) {
	commands_printf("Unsupported duty no ramp");
}

/**
 * Use PID rpm control. Note that this value has to be multiplied by half of
 * the number of motor poles.
 *
 * @param rpm
 * The electrical RPM goal value to use.
 */
void mcpwm_atc_set_pid_speed(float rpm) {
	commands_printf("Unsupported set pid speed");
}

/**
 * Use current control and specify a goal current to use. The sign determines
 * the direction of the torque. Absolute values less than
 * conf->cc_min_current will release the motor.
 *
 * @param current
 * The current to use.
 */
void mcpwm_atc_set_current(float current) {
	commands_printf("Unsupported set current");
}

/**
 * Brake the motor with a desired current. Absolute values less than
 * conf->cc_min_current will release the motor.
 *
 * @param current
 * The current to use. Positive and negative values give the same effect.
 */
void mcpwm_atc_set_brake_current(float current) {
	if (fabsf(current) < m_conf->cc_min_current) {
		m_control_mode = CONTROL_MODE_NONE;
		stop_pwm_hw();
		return;
	}
	commands_printf("Unsupported set brake");
	
}

float mcpwm_atc_get_pid_pos_set(void) {
	return m_pos_pid_set;
}

float mcpwm_atc_get_pid_pos_now(void) {
	return m_pos_pid_now;
}

/**
 * Get the current switching frequency.
 *
 * @return
 * The switching frequency in Hz.
 */
float mcpwm_atc_get_switching_frequency_now(void) {
	return switching_frequency_now;
}

/**
 * Get the motor current. The sign of this value will
 * represent whether the motor is drawing (positive) or generating
 * (negative) current.
 *
 * @return
 * The motor current.
 */
float mcpwm_atc_get_tot_current(void) {
	return last_current_sample;
}

/**
 * Get the FIR-filtered motor current. The sign of this value will
 * represent whether the motor is drawing (positive) or generating
 * (negative) current.
 *
 * @return
 * The filtered motor current.
 */
float mcpwm_atc_get_tot_current_filtered(void) {
	return last_current_sample_filtered;
}

/**
 * Get the motor current. The sign of this value represents the direction
 * in which the motor generates torque.
 *
 * @return
 * The motor current.
 */
float mcpwm_atc_get_tot_current_directional(void) {
	const float retval = mcpwm_atc_get_tot_current();
	return dutycycle_now > 0.0 ? retval : -retval;
}

/**
 * Get the filtered motor current. The sign of this value represents the
 * direction in which the motor generates torque.
 *
 * @return
 * The filtered motor current.
 */
float mcpwm_atc_get_tot_current_directional_filtered(void) {
	const float retval = mcpwm_atc_get_tot_current_filtered();
	return dutycycle_now > 0.0 ? retval : -retval;
}

/**
 * Get the input current to the motor controller.
 *
 * @return
 * The input current.
 */
float mcpwm_atc_get_tot_current_in(void) {
	return mcpwm_atc_get_tot_current() * fabsf(dutycycle_now);
}

/**
 * Get the FIR-filtered input current to the motor controller.
 *
 * @return
 * The filtered input current.
 */
float mcpwm_atc_get_tot_current_in_filtered(void) {
	return mcpwm_atc_get_tot_current_filtered() * fabsf(dutycycle_now);
}

float mcpwm_atc_get_duty_cycle_set(void){
	// TODO
	return -1.0;
}

float mcpwm_atc_get_duty_cycle_now(void){
	// TODO
	return -1.0;
}

float mcpwm_atc_get_sampling_frequency_now(void){
	return -1.0;
}

float mcpwm_atc_get_rpm(void){
	return -1.0;
}

int mcpwm_atc_get_tachometer_value(bool reset){
	return -1;
}

int mcpwm_atc_get_tachometer_abs_value(bool reset){
	return 1;
}

float mcpwm_atc_get_last_inj_adc_isr_duration(void){
	return last_inj_adc_isr_duration;
}

// Interrupt handler definitions
void mcpwm_atc_tim_sample_int_handler(void) {
	if (m_init_done) {
		// Generate COM event here for synchronization
		TIM_GenerateEvent(TIM1, TIM_EventSource_COM);
	}
}
void mcpwm_atc_adc_int_handler(void *p, uint32_t flags){
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
	float ic = -(ia + ib);



	filter_add_sample((float*) current_fir_samples, last_current_sample,
			CURR_FIR_TAPS_BITS, (uint32_t*) &current_fir_index);
	last_current_sample_filtered = filter_run_fir_iteration(
			(float*) current_fir_samples, (float*) current_fir_coeffs,
			CURR_FIR_TAPS_BITS, current_fir_index);

	// MCIF handler
	mc_interface_mc_timer_isr();

	last_inj_adc_isr_duration = (float) TIM12->CNT / 10000000.0;
}

// Private function definitions

static THD_FUNCTION(timer_thread, arg) {
	(void)arg;

	chRegSetThreadName("mcpwm_atc timer");

	systime_t time = chVTGetSystemTime();

	for(;;) {
		// Define loop speed
		time += MS2ST(2); //500hz timing

		if (timer_thd_stop) {
			timer_thd_stop = false;
			return;
		}

		// Meat and Potatoes------------------------------------------------
		m_pos_pid_now = do_pid_pos_control();
		
		// End Meat and Potatoes--------------------------------------------

		// Set loop speed
		chThdSleepUntil(time);
	}

}

static float do_pid_pos_control(void){
	float pos = encoder_read_deg();
	utils_norm_angle(&pos);

	return pos;
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
	commands_printf("start");
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