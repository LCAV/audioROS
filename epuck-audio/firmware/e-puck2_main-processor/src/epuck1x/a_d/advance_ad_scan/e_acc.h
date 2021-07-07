#ifndef _ACC_DEFS
#define _ACC_DEFS

#ifdef __cplusplus
extern "C" {
#endif

#define CST_RADIAN		(180.0/3.1415)	// used to convert radian in degrees
#define ANGLE_ERROR		666.0			// value returned if an angle can't be defined
#define FILTER_SIZE		5				// define the size of the averaging filter

// ID of the different captor in their respective array
#define ACCX_BUFFER	0
#define ACCY_BUFFER	1
#define ACCZ_BUFFER	2

#define GRAVITY -16384    // Value for -1 g (16 bits accelerometer) @ +-2g range.

/*! \struct TypeAccSpheric
 * \brief struct to store the acceleration vector in spherical coord
 */
typedef struct
{
	float acceleration;		/*!< lenght of the acceleration vector 
							 * = intensity of the acceleration */
	float orientation;		/*!< orientation of the acceleration vector 
							 * in the horizontal plan, zero facing front
							 * - 0° = inclination to the front 
							 * (front part lower than rear part)
							 * - 90° = inclination to the left  
							 * (left part lower than right part)
							 * - 180° = inclination to the rear  
							 * (rear part lower than front part)
							 * - 270° = inclination to the right 
							 * (right part lower than left part) */
	float inclination;		/*!< inclination angle with the horizontal plan
							 * - 0° = e-puck horizontal
							 * - 90° = e-puck vertical
							 * - 180° = e-puck horizontal but up-side-down */
} TypeAccSpheric;

/*! \struct TypeAccRaw
 * \brief struct to store the acceleration raw data
 * in carthesian coord
 */
typedef struct
{
	int acc_x;	/*!< The acceleration on x axis */
	int acc_y;	/*!< The acceleration on y axis */
	int acc_z;	/*!< The acceleration on z axis */
} TypeAccRaw;


/*! \brief Read the last value of a given accelerometer axis
 * \param captor		ID of the axis to read
 *						(must be 0 = x, 1 = y or 2 = z)
 * \return value		last filtered (calibration value removed) axis value.
 */
int e_get_acc(unsigned int captor);

/*! \brief Read the value of a channel, filtered by an averaging filter
 * \param captor		ID of the axis to read (must be 0 to 2)
 * \param filter_size	size of the filter (must be between 1 and ACC_SAMP_NB)
 * \return value		filtered axis value
 */
int e_get_acc_filtered(unsigned int captor, unsigned int filter_size);

/*! \brief Calculate the zero values of all the accelerometer's axes.
 *
 * It reads 50 values and average them to initiate
 * the "zero" value of the accelerometer
 */
void e_acc_calibr(void);

/*! \brief Calculate and return the accel. in spherical coord
 * \return acceleration in spherical coord
 * \sa TypeAccSpheric
 */
TypeAccSpheric e_read_acc_spheric(void);

/*! \brief calculate and return the inclination angle
 * \return inclination angle of the robot
 */
float e_read_inclination(void);

/*! \brief calculate and return the orientation angle
 * \return orientation of the accel vector
 */
float e_read_orientation(void);

/*! \brief calculate and return the intensity of the acceleration
 * \return intensity of the acceleration vector
 */
float e_read_acc(void);

/*! \brief Return acceleration on the x,y,z axis
 * \return acceleration on the x,y,z axis
 * \sa TypeAccRaw
 */
TypeAccRaw e_read_acc_xyz(void);

/*! \brief Return acceleration on the x axis
 * \return acceleration on the x axis
 */
int e_read_acc_x(void);

/*! \brief Return acceleration on the y axis
 * \return acceleration on the y axis
 */
int e_read_acc_y(void);

/*! \brief Return acceleration on the z axis
 * \return acceleration on the z axis
 */
int e_read_acc_z(void);

/*! \brief light the led according to the orientation angle of the robot*/
void e_display_angle(void);

#ifdef __cplusplus
}
#endif

#endif
