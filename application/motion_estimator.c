/**
 * @brief motion calculation application
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "FreeRTOS.h"
#include "task.h"

#include "flexio_ov7670.h"

#include "board.h"
#include "pin_mux.h"
#include "fsl_port.h"
#include "fsl_debug_console.h"
#include "fsl_device_registers.h"

#include "motion_estimator.h"

/** for fertilizer flow detection */
#define MOTION_THRESHOLD  250

/** defines the camera frame_speed */
#define MOTION_FRAME_RATE 30

/** camera buffer index */
#define CAMERA_NOOF_BUFFER 2

/** task parameters list */
#define MOTION_TASK_PRIO        configMAX_PRIORITIES - 1
#define MOTION_TASK_STACK_DEEP  256
TaskHandle_t motion_task;

/** read only flow sensor parameters */
estimator_param agro_params;

static uint32_t flow_ticker = 0;

/** current buffer index */
int current_buffer = 0;
int capture_buffer = 0;

/** motion detection flag */
bool motion_detected = false;

/** motion estimator current time */
unsigned int motion_current_time;

/** camera buffer */
extern uint8_t g_FlexioCameraFrameBuffer[2][OV7670_FRAME_BYTES + 32];

static volatile __attribute__((aligned(32), section(".camera_ram"))) uint8_t diff_frame[OV7670_FRAME_BYTES];
static volatile __attribute__((aligned(32), section(".camera_ram"))) uint8_t prev_frame[OV7670_FRAME_BYTES];


/* forward references */
static void motion_sensor_task(void *args);
static void motion_frame_diff(uint8_t *dest, uint8_t *op1, uint8_t *op2);
static uint32_t motion_frame_pix_average(uint16_t *mat, uint32_t size);
static void motion_frame_move(uint16_t *dest, uint16_t *src, uint32_t size) ;

/* private functions */
/**
 * @brief take frame difference
 */
static void motion_frame_diff(uint8_t *dest, uint8_t *op1, uint8_t *op2)
{
    int i = 0;

    for(i = 0; i < OV7670_FRAME_BYTES/4; i++){
        uint32_t pix_dest = 0, pix_op1 =0,pix_op2 =0;
        pix_op1 += *((uint32_t*)op1);
        pix_op2  += *((uint32_t*)op2);

        /* subtract using SIMD */
        pix_dest = __UQSUB16(pix_op1, pix_op2);

        /* update the new frame */
        *((uint32_t *)dest) = pix_dest;
        op1+=4;
        op2+=4;
        dest+=4;

    }
}

/**
 * @brief calculates average pixel value on a frame
 */
static uint32_t motion_frame_pix_average(uint16_t *mat, uint32_t size)
{
    int i = 0;
    uint64_t acc = 0;

    for( i = 0; i < size ; i++) {
        acc += (uint32_t)(*mat);
        mat++;
    }
    return((uint32_t)(acc / size));
}

/**
 * @brief move image between frames
 */
static void motion_frame_move(uint16_t *dest, uint16_t *src, uint32_t size)
{
    int i = 0;
    for( i = 0 ; i < size; i++) {
        /* since the frame size is a multiple of 4
         * we can move by 4 and 4 bytes to improve speed
         */
        (*((uint32_t *)dest)) = (*((uint32_t *)src));
        src+=2;
        dest+=2;
    }
}

/**
 * @brief map the flow intensity to percentage
 * @info extracted from arduino.cc
 */
int map(int x, int in_min, int in_max, int out_min, int out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


/**
 * @brief ISR generated each frame captured
 */
void PORTB_IRQHandler(void)
{
    /* Clear the interrupt flag for PTB2 */
    PORT_ClearPinsInterruptFlags(PORTB, 1U << BOARD_FLEXIO_VSYNC_PIN_INDEX);
    FLEXIO_Ov7670AsynCallback();

    /* each execution retrigger the capture fifo */
    if(capture_buffer) {
        capture_buffer--;
        if(capture_buffer == 0) {
            capture_buffer = CAMERA_NOOF_BUFFER;
        }
        FLEXIO_Ov7670StartCapture(capture_buffer);
    }

    BaseType_t ctw = xTaskResumeFromISR(motion_task);
    portYIELD_FROM_ISR(ctw);
}


/**
 * @brief motion calculation task
 */
static void motion_sensor_task(void *args)
{
    uint32_t pixel_average_value =0;
    uint32_t acc = 0;
    uint32_t cnt = 0;

    /* configures the vsync interrupts */
    NVIC_SetPriority((IRQn_Type)PORTB_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
    NVIC_EnableIRQ((IRQn_Type)PORTB_IRQn);


    capture_buffer = CAMERA_NOOF_BUFFER;
    /* starts the video capture */
    FLEXIO_Ov7670StartCapture(capture_buffer);
    vTaskSuspend(NULL);

    for(;;) {
        /* gets the current available buffer */
        current_buffer = (capture_buffer != 0 ? capture_buffer - 1 : 0) ;

        /* compute image diff between frames */
        motion_frame_diff((uint8_t *)&diff_frame[0], (uint8_t *)&g_FlexioCameraFrameBuffer[current_buffer][32],
                (uint8_t *)&prev_frame[0]);

        pixel_average_value = motion_frame_pix_average((uint16_t *)&diff_frame[0], OV7670_FRAME_BYTES / 2);

        //PRINTF("pixel average value: %d \n\r", pixel_average_value);


        /* update frame history */
        motion_frame_move((uint16_t *)&prev_frame[0],
                            (uint16_t *)&g_FlexioCameraFrameBuffer[current_buffer][32],
                            OV7670_FRAME_BYTES / 4);


        /* estimate flow presence */
        if(pixel_average_value > MOTION_THRESHOLD) {
            agro_params.has_flow = true;
        }
        else {
            agro_params.has_flow = false;
        }

        /* estimate flow intensity */
        agro_params.flow = map(pixel_average_value, MOTION_MIN_RANGE, MOTION_MAX_RANGE,
                            0, 100);

        vTaskSuspend(NULL);
    }
}

/* public api */


/*
 *  motion_estimator_app_init()
 */
int motion_estimator_app_init(void)
{
    BaseType_t err;

    /* Init the camera module */
    err = FLEXIO_Ov7670Init();
    if(err != 0) {

        PRINTF("[CAMERA]:Failed to start camera driver \n\r");
        return (-1);
    }

    /* create the application task */
    err = xTaskCreate(motion_sensor_task,"motion",
                      MOTION_TASK_STACK_DEEP, 0,
                      MOTION_TASK_PRIO, &motion_task);
    if(err != pdPASS) {
        PRINTF("[MOTION_TASK]:Failed to start motion app \n\r");
        return(-1);
    }

    PRINTF("[MOTION_TASK]:motion app started \n\r");
    return (0);
}

/*
 *  get_current_params()
 */
const estimator_param *get_sensor_params(void)
{
    return((const estimator_param *) &agro_params);
}

/*
 *  get_camera_buffer()
 */
const unsigned char *get_camera_buffer(void)
{
    if(current_buffer > 0) {
        /* take the current buffer has the full frame */
        return((const unsigned char *)&g_FlexioCameraFrameBuffer[current_buffer][32]);
    }
    else {
        return(NULL);
    }
}
