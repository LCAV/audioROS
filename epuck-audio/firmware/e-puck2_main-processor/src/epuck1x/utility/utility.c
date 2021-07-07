#include "hal.h"
#include "utility.h"
#include "../sensors/battery_level.h"
#include "../selector.h"

//#define MAX_BATT_VALUE 2560 // corresponds to 4.2 volts
//#define MIN_BATT_VALUE 2070 // corresponds to 3.4 volts
//#define BATT_VALUES_RANGE (MAX_BATT_VALUE-MIN_BATT_VALUE)

uint32_t tickAdcIsr = 0;

void wait(long num) {
	volatile long i;
	for(i=0;i<num;i++);
}

int getselector(void) {
	return get_selector();
}

unsigned int getBatteryValueRaw(void) {
	return get_battery_raw();
}

float getBatteryValueVoltage(void){
	return get_battery_voltage();
}

unsigned int getBatteryValuePercentage(void) {
    return (unsigned int)get_battery_percentage();
}

void resetTime(void) {
    tickAdcIsr = chSysGetRealtimeCounterX();
}

// Based on the Realtime counter.
float getDiffTimeMs(void) {
	return (float)(RTC2US(STM32_SYSCLK, chSysGetRealtimeCounterX()-tickAdcIsr))/1000.0;
}

float getDiffTimeMsAndReset(void) {
	float value = (float)(RTC2US(STM32_SYSCLK, chSysGetRealtimeCounterX()-tickAdcIsr))/1000.0;
    tickAdcIsr = chSysGetRealtimeCounterX();
    return value;
}

