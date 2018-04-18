#ifndef MCCONF_MULTISTAR_H_
#define MCCONF_MULTISTAR_H_

/*
 * HW: 4.10
 */

/*
 * Parameters
 */
#define MCCONF_DEFAULT_MOTOR_TYPE			MOTOR_TYPE_FOC
#define MCCONF_L_CURRENT_MAX				30.0	// Current limit in Amperes (Upper)
#define MCCONF_L_CURRENT_MIN				-25.0	// Current limit in Amperes (Lower)
#define MCCONF_L_MAX_ABS_CURRENT			100.0	// The maximum absolute current above which a fault is generated
#define MCCONF_L_SLOW_ABS_OVERCURRENT		1		// Use the filtered (and hence slower) current for the overcurrent fault detection
#define MCCONF_L_IN_CURRENT_MAX				25.0	// Input current limit in Amperes (Upper)
#define MCCONF_L_IN_CURRENT_MIN				-20.0	// Input current limit in Amperes (Lower)
#define MCCONF_L_RPM_MAX					10000.0	// The motor speed limit (Upper)
#define MCCONF_L_RPM_MIN					-10000.0	// The motor speed limit (Lower)
#define MCCONF_L_MIN_VOLTAGE				10.0	// Minimum input voltage
#define MCCONF_L_MAX_VOLTAGE				57.0	// Maximum input voltage
#define MCCONF_CC_STARTUP_BOOST_DUTY		0.03	// The lowest duty cycle to use in current control mode (has to be > MCPWM_MIN_DUTY_CYCLE)
#define MCCONF_L_RPM_LIMIT_NEG_TORQUE		0		// Use negative torque to limit the RPM
#define MCCONF_L_CURR_MAX_RPM_FBRAKE		1500	// Maximum electrical RPM to use full brake at

#define MCCONF_L_BATTERY_CUT_START			41.0	// Start limiting the positive current at this voltage
#define MCCONF_L_BATTERY_CUT_END			39.0		// Limit the positive current completely at this voltage

// Deadtime setting
#define HW_DEAD_TIME_VALUE					60 // clock ticks 0x00 to 0xFF

// Sensorless settings
#define MCCONF_SENSOR_MODE				SENSOR_MODE_SENSORLESS // Sensor mode
#define MCCONF_SL_MIN_RPM					250		// Auto-commutate below this RPM
#define MCCONF_SL_MIN_ERPM_CYCLE_INT_LIMIT	1100.0	// Minimum RPM to calculate the BEMF coupling from
#define MCCONF_SL_CYCLE_INT_LIMIT			38.87	// Flux integrator limit 0 ERPM
#define MCCONF_SL_PHASE_ADVANCE_AT_BR	0.8		// Flux integrator limit percentage at MCPWM_CYCLE_INT_START_RPM_BR ERPM
#define MCCONF_SL_BEMF_COUPLING_K		829.22	// Input voltage to bemf coupling constant

// FOC settings
#define MCCONF_FOC_CURRENT_KP			0.03
#define MCCONF_FOC_CURRENT_KI			0.0
#define MCCONF_FOC_F_SW					20000.0
#define MCCONF_FOC_MOTOR_L				0.000007
#define MCCONF_FOC_MOTOR_R				0.015
#define MCCONF_FOC_MOTOR_FLUX_LINKAGE	0.00245
#define MCCONF_FOC_OBSERVER_GAIN		9e7
#define MCCONF_FOC_OPENLOOP_RPM			600.0
#define MCCONF_FOC_SL_OPENLOOP_HYST		0.5
#define MCCONF_FOC_SL_OPENLOOP_TIME		0.5
#define MCCONF_FOC_ENCODER_INVERTED 	0
#define MCCONF_FOC_ENCODER_OFFSET 		327.92
#define MCCONF_FOC_ENCODER_RATIO 		7

// Speed PID parameters
#define MCCONF_S_PID_KP					0.0001	// Proportional gain
#define MCCONF_S_PID_KI					0.002	// Integral gain
#define MCCONF_S_PID_KD					0.0		// Derivative gain
#define MCCONF_S_PID_MIN_RPM			1200.0	// Minimum allowed RPM

// Position PID parameters
#define MCCONF_P_PID_KP                 0.2     // Proportional gain
#define MCCONF_P_PID_KI                 0.1     // Integral gain
#define MCCONF_P_PID_KD                 0.01    // Derivative gain
#define MCCONF_P_PID_ANG_DIV            1.0     // Divide angle by this value

#define MCCONF_M_SENSOR_PORT_MODE       SENSOR_PORT_MODE_AS5047_SPI

#define MCCONF_FOC_TEMP_COMP			true	// True uses anticogging

#endif /* MCCONF_MULTISTAR_H_ */