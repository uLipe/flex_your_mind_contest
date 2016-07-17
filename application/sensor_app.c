/**
 * @brief sensor data packing and formatting app
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "fsl_debug_console.h"
#include "FreeRTOS.h"
#include "task.h"
#include "motion_estimator.h"
#include "sensor_app.h"

/** sensor packet task parameters list */
#define SENSOR_TASK_PRIO        configMAX_PRIORITIES - 2
#define SENSOR_TASK_STACK_DEEP  256
TaskHandle_t sensor_ctl;

/* forward references */
static void sensor_task(void *args);

/** private functions */
static void sensor_task(void *args)
{
    estimator_param *flow_params;
    (void)args;

    for(;;) {
        flow_params = get_sensor_params();

        /* check if flow detected */
        if(flow_params->has_flow == true) {
            PRINTF("flow detected! With value of: %d [percent] \n\r",flow_params->flow );
        }
                
        vTaskDelay(PACKET_DISPATCH_PERIOD);
    }
}


/** public apis */
int start_sensor_app(void *args)
{
    BaseType_t err;

    /* create the application task */
    err = xTaskCreate(sensor_task,"packet",
                      SENSOR_TASK_STACK_DEEP, args,
                      SENSOR_TASK_PRIO, sensor_ctl);
    if(err != pdPASS) {
        PRINTF("[SENSOR_TASK]:Failed to start sensor app \n\r");
        return(-1);
    }
    else {
        PRINTF("[SENSOR_TASK]:started sensor app! \n\r");
    }

    return 0;
}
