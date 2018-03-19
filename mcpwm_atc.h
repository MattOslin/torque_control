#ifndef MCPWM_ATC_H_
#define MCPWM_ATC_H_

#include "conf_general.h"

// Functions

void mcpwm_atc_init(volatile mc_configuration *configuration);
void mcpwm_atc_deinit(void);
bool mcpwm_atc_init_done(void);
mc_state mcpwm_atc_get_state(void);
void mcpwm_atc_set_configuration(volatile mc_configuration *configuration);
void mcpwm_atc_stop_pwm(void);
void mcpwm_atc_set_pid_pos(float pos);
void mcpwm_atc_set_duty(float dutyCycle);
void mcpwm_atc_set_duty_noramp(float dutyCycle);
void mcpwm_atc_set_pid_speed(float rpm);
void mcpwm_atc_set_current(float current);
void mcpwm_atc_set_brake_current(float current);
float mcpwm_atc_get_duty_cycle_set(void);
float mcpwm_atc_get_duty_cycle_now(void);
float mcpwm_atc_get_pid_pos_set(void);
float mcpwm_atc_get_pid_pos_now(void);
float mcpwm_atc_get_tot_current(void);
float mcpwm_atc_get_tot_current_filtered(void);
float mcpwm_atc_get_tot_current_directional(void);
float mcpwm_atc_get_tot_current_directional_filtered(void);
float mcpwm_atc_get_tot_current_in(void);
float mcpwm_atc_get_tot_current_in_filtered(void);
float mcpwm_atc_get_sampling_frequency_now(void);
float mcpwm_atc_get_rpm(void);
float mcpwm_atc_get_last_inj_adc_isr_duration(void);
int mcpwm_atc_get_tachometer_value(bool reset);
int mcpwm_atc_get_tachometer_abs_value(bool reset);
bool mcpwm_atc_is_dccal_done(void);

// Interrupt handlers
void mcpwm_atc_tim_sample_int_handler(void);
void mcpwm_atc_adc_int_handler(void *p, uint32_t flags);

// Defines
#define MCPWM_ATC_I_FILTER_CONST					0.1 // Filter constant for the current filters
#define MCPWM_ATC_CURRENT_SAMP_OFFSET				(2) // Offset from timer top for injected ADC samples

#endif /* MC_PWM_ATC_H_ */