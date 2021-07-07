/**
 * @file	uc_usage.h
 * @brief  	Functions to show the uC usage
 * 
 * @source			http://www.chibios.com/forum/viewtopic.php?f=2&t=138&start=10
 * @modified by  	Eliot Ferragni
 */



/**
 * @brief 			Prints the uC usage on the output specified
 * 					The values returned have a relatively big inertia
 * 					because ChibiOS counts the total time used by each thread.
 * 
 * @param device 	Pointer to the output
 */	
void printUcUsage(BaseSequentialStream* out);