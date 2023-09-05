#include "stdio.h"

// 16bit, monoral, 16000Hz,  linear PCM
void CreateWavHeader(unsigned char header[], int waveDataSize);  // size of header is 44