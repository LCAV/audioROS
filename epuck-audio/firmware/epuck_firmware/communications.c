#include "ch.h"
#include "hal.h"
#include <main.h>

#include <communications.h>

/*
*	Sends floats numbers to the computer
*/
void SendFrameToComputer(BaseSequentialStream* out, float* data, uint16_t size, uint32_t timestamp)
{	
	chSequentialStreamWrite(out, (uint8_t*)"START", 5);
	chSequentialStreamWrite(out, (uint8_t*)&size, sizeof(size));
	chSequentialStreamWrite(out, (uint8_t*)&timestamp, sizeof(timestamp));
	chSequentialStreamWrite(out, (uint8_t*)data, size * sizeof(float));
	chSequentialStreamWrite(out, (uint8_t*)"END", 3);
}

void SendStart(BaseSequentialStream* out)
{
	chSequentialStreamWrite(out, (uint8_t*)"START", 5);
}


/*
 * Function that is responsible for receiving the frequency of the robot from the computer
 */
uint16_t ReceiveFrequencyFromComputer(BaseSequentialStream* in){

	volatile uint16_t frequency = 0;
	volatile uint8_t c1;

	uint8_t state = 0;
	while(state != 5){

        c1 = chSequentialStreamGet(in);

        //State machine to detect the string EOF\0S in order synchronize
        //with the frame received
        switch(state){
        	case 0:
        		if(c1 == 'D')
        			state = 1;
        		else
        			state = 0;
        		break;
        	case 1:
        		if(c1 == 'R')
        			state = 2;
        		else if(c1 == 'S')
        			state = 1;
        		else
        			state = 0;
        		break;
        	case 2:
        		if(c1 == 'A')
        			state = 3;
        		else if(c1 == 'S')
        			state = 1;
        		else
        			state = 0;
        		break;
        	case 3:
        		if(c1 == 'T')
        			state = 4;
        		else if(c1 == 'S')
        			state = 1;
        		else
        			state = 0;
        		break;
        	case 4:
        		if(c1 == 'E')
        			state = 5;
        		else if(c1 == 'S')
        			state = 1;
        		else
        			state = 0;
        		break;
        }

	}
	//getting the frequency
	for(uint16_t i = 0; i < 4; i++){
		c1 = chSequentialStreamGet(in);
		frequency = frequency | (c1 << (i * 8));
	}
	if(frequency > 0){
		return frequency;
	}
	else{
		return 5000;
	}



}





/*
*	Receives int16 values from the computer and fill a float array with complex values.
*	Puts 0 to the imaginary part. Size is the number of complex numbers.
*	=> data should be of size (2 * size)
*/
uint16_t ReceiveInt16FromComputer(BaseSequentialStream* in, float* data, uint16_t size){

	volatile uint8_t c1, c2;
	volatile uint16_t temp_size = 0;
	uint16_t i=0;

	uint8_t state = 0;
	while(state != 5){

        c1 = chSequentialStreamGet(in);

        //State machine to detect the string EOF\0S in order synchronize
        //with the frame received
        switch(state){
        	case 0:
        		if(c1 == 'S')
        			state = 1;
        		else
        			state = 0;
        		break;
        	case 1:
        		if(c1 == 'T')
        			state = 2;
        		else if(c1 == 'S')
        			state = 1;
        		else
        			state = 0;
        		break;
        	case 2:
        		if(c1 == 'A')
        			state = 3;
        		else if(c1 == 'S')
        			state = 1;
        		else
        			state = 0;
        		break;
        	case 3:
        		if(c1 == 'R')
        			state = 4;
        		else if(c1 == 'S')
        			state = 1;
        		else
        			state = 0;
        		break;
        	case 4:
        		if(c1 == 'T')
        			state = 5;
        		else if(c1 == 'S')
        			state = 1;
        		else
        			state = 0;
        		break;
        }
	}

	c1 = chSequentialStreamGet(in);
	c2 = chSequentialStreamGet(in);

	// The first 2 bytes is the length of the datas
	// -> number of int16_t data
	temp_size = (int16_t)((c1 | c2<<8));

	if((temp_size/2) == size){
		for(i = 0 ; i < (temp_size/2) ; i++){

			c1 = chSequentialStreamGet(in);
			c2 = chSequentialStreamGet(in);

			data[i*2] = (int16_t)((c1 | c2<<8));	        //real
			data[(i*2)+1] = 0;				//imaginary
		}
	}

	return temp_size/2;

}
