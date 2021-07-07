
#ifndef UTILITY_EPUCK1X
#define UTILITY_EPUCK1X

#ifdef __cplusplus
extern "C" {
#endif

void wait(long num);
int getselector(void);
unsigned int getBatteryValueRaw(void);
float getBatteryValueVoltage(void);
unsigned int getBatteryValuePercentage(void);
void resetTime(void);
float getDiffTimeMs(void);
float getDiffTimeMsAndReset(void);

#ifdef __cplusplus
}
#endif

#endif
