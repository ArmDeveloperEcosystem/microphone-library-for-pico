#include "Wav.h"
#include "stdint.h"

// MONO 16bit
void CreateWavHeader(uint8_t header[], int waveDataSize){
  header[0] = 'R';
  header[1] = 'I';
  header[2] = 'F';
  header[3] = 'F';
  unsigned int fileSizeMinus8 = waveDataSize + 44 - 8;
  header[4] = (uint8_t)(fileSizeMinus8 & 0xFF);
  header[5] = (uint8_t)((fileSizeMinus8 >> 8) & 0xFF);
  header[6] = (uint8_t)((fileSizeMinus8 >> 16) & 0xFF);
  header[7] = (uint8_t)((fileSizeMinus8 >> 24) & 0xFF);
  header[8] = 'W';
  header[9] = 'A';
  header[10] = 'V';
  header[11] = 'E';
  header[12] = 'f';
  header[13] = 'm';
  header[14] = 't';
  header[15] = ' ';
  header[16] = 0x10;  // linear PCM
  header[17] = 0x00;
  header[18] = 0x00;
  header[19] = 0x00;
  header[20] = 0x01;  // linear PCM
  header[21] = 0x00;
  header[22] = 0x01;  // monoral
  header[23] = 0x00;
  header[24] = 0x80;  // sampling rate 16000
  header[25] = 0x3E;
  header[26] = 0x00;
  header[27] = 0x00;
  header[28] = 0x00;  // Byte/sec = 16000x2x1 = 32000
  header[29] = 0x7D;
  header[30] = 0x00;
  header[31] = 0x00;
  header[32] = 0x02;  // 16bit Mono
  header[33] = 0x00;
  header[34] = 0x10;  // 16bit
  header[35] = 0x00;
  header[36] = 'd';
  header[37] = 'a';
  header[38] = 't';
  header[39] = 'a';
  header[40] = (uint8_t)(waveDataSize & 0xFF);
  header[41] = (uint8_t)((waveDataSize >> 8) & 0xFF);
  header[42] = (uint8_t)((waveDataSize >> 16) & 0xFF);
  header[43] = (uint8_t)((waveDataSize >> 24) & 0xFF);
}

/* STEREO 32bit*/
/* 
  void CreateWavHeader(uint8_t header[], int waveDataSize){
  header[0] = 'R';
  header[1] = 'I';
  header[2] = 'F';
  header[3] = 'F';
  unsigned int fileSizeMinus8 = waveDataSize + 44 - 8;
  header[4] = (uint8_t)(fileSizeMinus8 & 0xFF);
  header[5] = (uint8_t)((fileSizeMinus8 >> 8) & 0xFF);
  header[6] = (uint8_t)((fileSizeMinus8 >> 16) & 0xFF);
  header[7] = (uint8_t)((fileSizeMinus8 >> 24) & 0xFF);
  header[8] = 'W';
  header[9] = 'A';
  header[10] = 'V';
  header[11] = 'E';
  header[12] = 'f';
  header[13] = 'm';
  header[14] = 't';
  header[15] = ' ';
  header[16] = 0x10;  // linear PCM
  header[17] = 0x00;
  header[18] = 0x00;
  header[19] = 0x00;
  header[20] = 0x01;  // linear PCM
  header[21] = 0x00;
  header[22] = 0x02;  // STEREO
  header[23] = 0x00;
  header[24] = 0x80;  // sampling rate 16000
  header[25] = 0x3E;
  header[26] = 0x00;
  header[27] = 0x00;
  header[28] = 0x00;  // Byte/sec = 16000x4x1 = 64000
  header[29] = 0xFA;
  header[30] = 0x00;
  header[31] = 0x00;
  header[32] = 0x04;  // 32bit Stereo
  header[33] = 0x00;
  header[34] = 0x20;  // 32bit
  header[35] = 0x00;
  header[36] = 'd';
  header[37] = 'a';
  header[38] = 't';
  header[39] = 'a';
  header[40] = (uint8_t)(waveDataSize & 0xFF);
  header[41] = (uint8_t)((waveDataSize >> 8) & 0xFF);
  header[42] = (uint8_t)((waveDataSize >> 16) & 0xFF);
  header[43] = (uint8_t)((waveDataSize >> 24) & 0xFF);
}
*/