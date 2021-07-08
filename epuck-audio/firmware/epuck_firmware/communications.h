#ifndef COMMUNICATIONS_H
#define COMMUNICATIONS_H


void SendFrameToComputer(BaseSequentialStream* out, float* data, uint16_t size, uint32_t timestamp);
void SendStart(BaseSequentialStream* out);

uint16_t ReceiveInt16FromComputer(BaseSequentialStream* in, float* data, uint16_t size);


uint16_t ReceiveFrequencyFromComputer(BaseSequentialStream* in);

#endif /* COMMUNICATIONS_H */
