#ifndef _MICRO
#define _MICRO

#ifdef __cplusplus
extern "C" {
#endif

#include <hal.h>

// ID of the different captor in their respective array
#define MIC0_BUFFER	0
#define MIC1_BUFFER	1
#define MIC2_BUFFER	2

/*! \brief Get the last value of a given micro
 * \param micro_id		micro's ID (0, 1, or 2)
 *							(use \ref MIC0_BUFFER, \ref MIC1_BUFFER , \ref MIC2_BUFFER  defined in e_micro.h)
 * \return result		last value of the micro
 */
int e_get_micro(unsigned int micro_id);

/*! \brief Get the average on a given number of sample from a micro
 * \param micro_id		micro's ID (0, 1, or 2)
 *							(use \ref MIC0_BUFFER, \ref MIC1_BUFFER , \ref MIC2_BUFFER  defined in e_micro.h)
 * \param filter_size	number of sample to average (max is \ref MIC_SAMP_NB)
 * \return result		average of the last \ref filter_size samples of the micro
 */
int e_get_micro_average(unsigned int micro_id, unsigned int filter_size);

/*! \brief Get the difference between the highest and lowest sample.
 * Beware that from e-puck rev 1.3 the microphone sensitivity resulted a little bit different from the previous hardware revision;
 * some empirical tests show that the difference is about +/-15%.
 *
 * \param micro_id		micro's ID (0, 1, or 2)
 *							(use \ref MIC0_BUFFER, \ref MIC1_BUFFER , \ref MIC2_BUFFER  defined in e_micro.h)
 * \return result		volume
 */
int e_get_micro_volume (unsigned int micro_id);

/*! \brief Write to a given array, the last values of one micro
 *
 * Write to a given array, the last values of one micro. The values are
 * stored with the last one first, and the older one at the end of the array.
 * \n [ t ][ t-1 ][ t-2 ][ t-3 ]...[ t-(samples_nb-1) ][ t-samples_nb ]
 * \param micro_id		micro's ID (0, 1, or 2)
 *							(use \ref MIC0_BUFFER, \ref MIC1_BUFFER , \ref MIC2_BUFFER  defined in e_micro.h)
 * \param *result		pointer on the result array
 * \param samples_nb	size of the result array
 *							(must be between 1 and \ref MIC_SAMP_NB)
 */
void e_get_micro_last_values (int micro_id, int16_t * result, unsigned samples_nb);


#ifdef __cplusplus
}
#endif

#endif /*_MICRO*/

