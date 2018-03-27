#ifndef __CONTROL_H__
#define __CONTROL_H__

#include <stdint.h>

#define MAX_DATA_SIZE 128
// This packet has 1 + 1 + 2 + 32 + 16 * 4 = 100 bytes
typedef struct comms
{
	uint32_t magic;
	uint32_t dataSize;
	char command;
	char packetType;
	char buffer[MAX_DATA_SIZE];
	unsigned char checksum;
	uint32_t params[16];
	char dummy[1];
} TComms;


#endif
