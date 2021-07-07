#include "e_prox.h"
#include "../sensors/proximity.h"

void e_calibrate_ir(void)
{
	calibrate_ir();
}

int e_get_prox(unsigned int sensor_number)
{
	if (sensor_number > 7) {
		return 0;
	} else {
		return get_prox(sensor_number);
	}
}

int e_get_calibrated_prox(unsigned int sensor_number)
{
	if (sensor_number > 7) {
		return 0;
	} else {
		return get_calibrated_prox(sensor_number);
	}
}

int e_get_ambient_light(unsigned int sensor_number)
{
	if (sensor_number > 7) {
		return 0;
	} else {
		return get_ambient_light(sensor_number);
	}
}
