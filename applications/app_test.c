#include "ch.h" // ChibiOS
#include "hal.h" // ChibiOS HAL
#include "mc_interface.h" // Motor control functions
#include "hw.h" // Pin mapping on this hardware
#include "timeout.h" // To reset the timeout
#include "terminal.h" // For terminal commands
#include "commands.h" // To print
#include "utils.h" // For fun
 
// Example thread
static THD_FUNCTION(test_thread, arg);
static THD_WORKING_AREA(test_thread_wa, 2048); // 2kb stack for this thread
 
// Private variables
static volatile bool stop_now = true;
static volatile bool is_running = false;

// Private functions
static void terminal_cmd_test(int argc, const char **argv);
static void terminal_cmd_test_start(int argc, const char **argv);
static void terminal_cmd_test_stop(int argc, const char **argv);

void app_custom_configure(app_configuration *conf){

	terminal_register_command_callback(
			"test",
			"Print some shit",
			0,
			terminal_cmd_test);
	terminal_register_command_callback(
			"test_start",
			"Start test",
			0,
			terminal_cmd_test_start);
	terminal_register_command_callback(
			"test_stop",
			"Stop test",
			0,
			terminal_cmd_test_stop);
}

void app_custom_start(void){
	// Start the example thread
	stop_now = false;
	chThdCreateStatic(test_thread_wa, sizeof(test_thread_wa),
		NORMALPRIO, test_thread, NULL);
}

void app_custom_stop(void){
	stop_now = true;
	while (is_running) {
		chThdSleepMilliseconds(1);
	}
}
 
static THD_FUNCTION(test_thread, arg) {
	(void)arg;
 
	chRegSetThreadName("APP_TEST");
	
	commands_printf("Restarted thread");
 
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
		commands_printf("Position: %.2f",mc_interface_get_pid_pos_now());
 

		// End Meat and Potatoes--------------------------------------------

		// Set loop speed
		chThdSleepUntil(time);
 
		// Reset the timeout
		//timeout_reset();
	}
}

static void terminal_cmd_test(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	commands_printf("This is not a test");
	commands_printf(" ");
}

static void terminal_cmd_test_stop(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	commands_printf("Stopping");
	commands_printf(" ");
	app_custom_stop();
}

static void terminal_cmd_test_start(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	commands_printf("Starting");
	commands_printf(" ");
	app_custom_start();
}