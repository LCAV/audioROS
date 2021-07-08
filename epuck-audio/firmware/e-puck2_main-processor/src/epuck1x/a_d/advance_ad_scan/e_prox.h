#ifndef _PROX
#define _PROX

#ifdef __cplusplus
extern "C" {
#endif

/*! \brief To calibrate the IR proximity sensors.
 * \warning Call this function one time before calling e_get_calibrated_prox
 */
void e_calibrate_ir(void);

/*! \brief To get the analog proxy sensor value of a specific IR sensor
 *
 * To estimate the proximity of an obstacle, we do the following things:
 * - measure the ambient light
 * - turn on the IR led of the sensor
 * - measure the reflected light + ambient light
 * - calculate: reflected light = (reflected light + ambient light) - ambient light
 * - turn off the IR led of the sensor
 *
 * The result value of this function is: reflected light. More this value is great,
 * more the obstacle is near.
 * \param sensor_number The proxy sensor's number that you want the value.
 *                      Must be between 0 to 7.
 * \return The analog value of the specified proxy sensor
 */
int e_get_prox(unsigned int sensor_number);

/*! \brief To get the calibrated value of the IR sensor
 *
 * This function return the calibrated analog value of the IR sensor.
 * \warning Before using this function you have to calibrate the IR proximity sensors (only one time)
 * by calling e_calibrate_ir().
 * \param sensor_number The proxy sensor's number that you want the calibrated value.
 *                      Must be between 0 to 7.
 * \return The analog value of the specified proxy sensor
 */
int e_get_calibrated_prox(unsigned int sensor_number);

/*! \brief To get the analog ambient light value of a specific IR sensor
 *
 * This function return the analog value of the ambient light measurement.
 * \param sensor_number The proxy sensor's number that you want the value.
 *                      Must be between 0 to 7.
 * \return The analog value of the specified proxy sensor
 */
int e_get_ambient_light(unsigned int sensor_number); // to get ambient light value

#ifdef __cplusplus
}
#endif

#endif
