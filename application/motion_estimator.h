/**
 * @brief motion calculation application
 */

#ifndef __MOTION_ESTIMATOR_H
#define __MOTION_ESTIMATOR_H

/** min max range of detector */
#define MOTION_MIN_RANGE    250
#define MOTION_MAX_RANGE    2500


/** structure wich contains the motion information */
typedef struct {
    int flow;       /* fertilizer / seed flow */
    int has_flow;   /* for fertilizer only */
    int duplicate_seed;/* duplicate seed flag */
    int failure_seed;  /* seed flow stopped */
    unsigned int timestamp;
} estimator_param;


/** @brief starts the motion calculation engine */
int motion_estimator_app_init(void);

/** @brief take snapshot of last measure */
const estimator_param *get_sensor_params(void);

/** @brief take the current full camera buffer */
const unsigned char *get_camera_buffer(void);

#endif
