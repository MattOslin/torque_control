#include "ch.h" // ChibiOS
#include "hal.h" // ChibiOS HAL
#include "mc_interface.h" // Motor control functions
#include "hw.h" // Pin mapping on this hardware
#include "timeout.h" // To reset the timeout
#include "terminal.h" // For terminal commands
#include "commands.h" // To print
#include "utils.h" // For fun
#include "encoder.h"
#include "mcpwm_foc.h" // For anticogging
 
// Threads
static THD_FUNCTION(cog_thread, arg);
static THD_WORKING_AREA(cog_thread_wa, 2048); // 2kb stack for this thread
static THD_FUNCTION(anticogging_thread, arg);
static THD_WORKING_AREA(anticogging_thread_wa, 2048);
static thread_t *anticogging_tp;
 
// Private variables
static volatile int position_step = 0;
static volatile bool stop_now = true;
static volatile bool is_running = false;

// Private functions
static void terminal_cmd_cog(int argc, const char **argv);
static void terminal_cmd_cog_start(int argc, const char **argv);
static void terminal_cmd_cog_stop(int argc, const char **argv);

void app_custom_configure(app_configuration *conf){
	(void)conf;
	
	terminal_register_command_callback(
			"cog",
			"Start anticogging routine",
			0,
			terminal_cmd_cog);
	terminal_register_command_callback(
			"cog_start",
			"Start anticogging app",
			0,
			terminal_cmd_cog_start);
	terminal_register_command_callback(
			"cog_stop",
			"Stop anticogging app",
			0,
			terminal_cmd_cog_stop);
}

void app_custom_start(void){
	// Start the example thread
	stop_now = false;
	chThdCreateStatic(cog_thread_wa, sizeof(cog_thread_wa),
		NORMALPRIO, cog_thread, NULL);
	chThdCreateStatic(anticogging_thread_wa, sizeof(anticogging_thread_wa),
		NORMALPRIO, anticogging_thread, NULL);
}

void app_custom_stop(void){
	stop_now = true;
	while (is_running) {
		chThdSleepMilliseconds(1);
	}
}
 
static THD_FUNCTION(cog_thread, arg) {
	(void)arg;
 
	chRegSetThreadName("APP_COG");
	commands_printf("Restarted cog thread");

	systime_t time = chVTGetSystemTime();
 
	is_running = true;

	for(;;) {
		// Define loop speed
		time += MS2ST(500); //500 ms timing

		// Stop
		if (stop_now) {
			is_running = false;
			return;
		}

		// Meat and Potatoes------------------------------------------------
		//commands_printf("Error: %.2f",(double)utils_angle_difference(mc_interface_get_pid_pos_set(), mc_interface_get_pid_pos_now()));
		//commands_printf("Message: %x", encoder_last_message());
		//commands_printf("Pos: %.2f", (double)mc_interface_get_pid_pos_now());
		//commands_printf("Size: %u", sizeof(float));

		// End Meat and Potatoes--------------------------------------------

		// Set loop speed
		chThdSleepUntil(time);
 
	}
}

static THD_FUNCTION(anticogging_thread, arg){
	(void)arg;

	chRegSetThreadName("ANTICOGGING");

	anticogging_tp = chThdGetSelfX();

	for(;;){
		// Wait for start command
		chEvtWaitAny((eventmask_t) 1);
		commands_printf("Started anticogging routine");

		static mc_configuration mcconf, mcconf_old; // Static to save some stack space
		mcconf = *mc_interface_get_configuration();
		mcconf_old = mcconf;

		mcconf.motor_type = MOTOR_TYPE_FOC;
		mc_interface_set_configuration(&mcconf);

		mcpwm_foc_measure_cogging();
		
		mc_interface_set_configuration(&mcconf_old);

		commands_printf("Done anticogging!");
	}

}

static void terminal_cmd_cog(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	commands_printf("This is cogging");
	chEvtSignal(anticogging_tp, (eventmask_t) 1);
}

static void terminal_cmd_cog_stop(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	commands_printf("Stopping");
	commands_printf(" ");
	app_custom_stop();
}

static void terminal_cmd_cog_start(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	commands_printf("Starting");
	commands_printf(" ");
	app_custom_start();
}