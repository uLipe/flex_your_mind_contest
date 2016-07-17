/**
 * @brief sensor data packing and formatting app
 */

#ifndef __SENSOR_APP_H
#define __SENSOR_APP_H

/** data packet sending period  in ms*/
#define PACKET_DISPATCH_PERIOD  200


/**
 * @brief Starts the sensor packet application
 */
int start_sensor_app(void *args);


#endif
