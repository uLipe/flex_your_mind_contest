/**
 *	@brief flow sensor main application file
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "fsl_debug_console.h"
#include "FreeRTOS.h"
#include "task.h"

/** application files list */
#include "motion_estimator.h"
#include "sensor_app.h"

/**
 * @brief Main application entry point...
 *
 */
int main(void)
{
	int err;

	/* starts the motion application, if error occurs, trap it*/
	err = motion_estimator_app_init();
	err = start_sensor_app(NULL);


	/* give to Os the system control */
	vTaskStartScheduler();

	for (;;) {
		/* should not reach here */
	}
}


/**
 * @brief idle task application hook:
 */
void vApplicationIdleHook(void)
{
	/** todo provide code to power management */
}
