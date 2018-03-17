#include "ch.h" // ChibiOS
#include "hal.h" // ChibiOS HAL
#include "mc_interface.h" // Motor control functions
#include "hw.h" // Pin mapping on this hardware
#include "timeout.h" // To reset the timeout
#include "terminal.h" // For terminal commands
#include "commands.h" // To print
#include "utils.h" // For fun
 
// Example thread
static THD_FUNCTION(cog_thread, arg);
static THD_WORKING_AREA(cog_thread_wa, 2048); // 2kb stack for this thread
 
// Private variables
static volatile int position_step = 0;
static volatile bool stop_now = true;
static volatile bool is_running = false;

// Private functions
static void terminal_cmd_cog(int argc, const char **argv);

void app_custom_configure(app_configuration *conf){

	terminal_register_command_callback(
			"cog",
			"It will do something",
			0,
			terminal_cmd_cog);
}

void app_custom_start(void){
	// Start the example thread
	stop_now = false;
	chThdCreateStatic(cog_thread_wa, sizeof(cog_thread_wa),
		NORMALPRIO, cog_thread, NULL);
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
 
	is_running = true;

	for(;;) {
		// Stop
		if (stop_now) {
			is_running = false;
			return;
		}

		// Meat and Potatoes------------------------------------------------
		commands_printf("Position: %.2f",mc_interface_get_pid_pos_now());

		// End Meat and Potatoes--------------------------------------------

		// Set loop speed
		chThdSleepMilliseconds(1000);
 
	}
}

static void terminal_cmd_cog(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	commands_printf("This is cogging");
	commands_printf(" ");
}