#ifndef _AD_CONV
#define _AD_CONV

#ifdef __cplusplus
extern "C" {
#endif

#define MIC_SAMP_NB 160 // (MIC_BUFFER_LEN/4) = 160 samples for each microphone

#define MICRO_ONLY 1
#define ALL_ADC 0

/*! \brief Initialize all the peripherals needed to enable the sensors handled by the e-puck1.x ADC.
 */
void e_init_ad_scan(unsigned char only_micro);

/*! \brief Not available in e-puck2.
 * In the e-puck2 the microphones are stored with double-buffering, thus return always true.
 * \return 1
 */
unsigned char e_ad_is_array_filled(void);

/*! \brief Not available in e-puck2.
 * In the e-puck2 the AD conversion is autonomous and thus we cannot clear this flag at the right time, so keep it always true.
 * \return 1
 */
unsigned char e_ad_is_acquisition_completed(void);

/*! \brief Not available in e-puck2. The ADC (proximity sampling) is always running once started.
 */
void e_ad_scan_on(void);

/*! \brief Not available in e-puck2. The ADC (proximity sampling) is always running once started.
 */
void e_ad_scan_off(void);

#ifdef __cplusplus
}
#endif

#endif /*_AD_CONV*/

/* End of File : ad_conv.h */
