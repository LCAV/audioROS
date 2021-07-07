#include "ch.h"
#include "hal.h"
#include <main.h>
#include <math.h>
#include <fft.h>

#include <arm_math.h>
#include <arm_const_structs.h>

/* Define complex multiplication and its conjugate */
#define  rmul(x,y)      (x.real * y.real - x.imag * y.imag)
#define  imul(x,y)      (x.imag * y.real + x.real * y.imag)
#define rcmul(x,y)      (x.real * y.real + x.imag * y.imag)
#define icmul(x,y)      (x.imag * y.real - x.real * y.imag)

/* 
*
*	FFT written in C
*	Litterally no optimisation at all
*	
*	Processing occurs in-place
*
*/
int fft_c(int lx, complex_float* cx, float signi)
{
	/* Declare all local variables */
	int i, j, k, m, istep;
	float arg;
	complex_float cw;		// Butterfly coefficient
	complex_float ct;		// Temp coefficient

	j = 0;
	k = 1;
	
	// Reorder the coefficients in bit reverse order
	for(i=0; i<lx; i++){
		if (i <= j){
			// Swap coefficients
			ct.real = cx[j].real;
			ct.imag = cx[j].imag;
			cx[j].real = cx[i].real;
			cx[j].imag = cx[i].imag;
			cx[i].real = ct.real;
			cx[i].imag = ct.imag;
		}
		m = lx/2;
		while (j > m-1){
			j = j - m;
			m = m/2;
			if (m < 1)
				break;
		}
		j = j + m;
	}
	
	// Do the FFT
	do{
		istep = 2*k;
		// Do a k points DFT
		for (m=0; m<k; m++){
			// Butterfly coefficients W(n,N)
			arg = 3.14159265*signi*m/k;
			cw.real = (float)(cos((double)(arg)));
			cw.imag = (float)(sin((double)(arg)));
			// Do the butterfly algorithm
			for (i=m; i<lx; i+=istep){
				ct.real = rmul(cw, cx[i+k]);
				ct.imag = imul(cw, cx[i+k]);
				cx[i+k].real = cx[i].real - ct.real;
				cx[i+k].imag = cx[i].imag - ct.imag;
				cx[i].real = cx[i].real + ct.real;
				cx[i].imag = cx[i].imag + ct.imag;
			}
		}
		k = istep;
	}while (k < lx);
	
	return(0);
}

/*
*	Wrapper to call a very optimized fft function provided by ARM
*	which uses a lot of tricks to optimize the computations
*/
void doFFT_optimized(uint16_t size, float* complex_buffer){
	if(size == 1024)
		arm_cfft_f32(&arm_cfft_sR_f32_len1024, complex_buffer, 0, 1);
	
}

/*
*	Wrapper to call the non optimized FFT function
*/
void doFFT_c(uint16_t size, complex_float* complex_buffer){

	fft_c(size, complex_buffer, +1.);
}