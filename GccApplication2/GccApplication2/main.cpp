#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <math.h>
#include "motor.h"
#include "buffer.c"
#include "constants.h"
#include "serialize.cpp"


#define PACKET_SIZE    140
#define MAX_DATA_SIZE       128
#define XMIT_SIZE     128
#define RECV_SIZE     128

static TBuffer _recvBuffer, _xmitBuffer;

int hear(unsigned char *);
TResult readPacket(TComms*);
void sendResponse(TComms *);
void say(const unsigned char *, int);
#include "response.h"

ISR(USART_UDRE_vect)
{
	unsigned char data;
	TBufferResult result = readBuffer(&_xmitBuffer, &data);
	
	if(result == BUFFER_OK)
	UDR0 = data;
	else
	if(result == BUFFER_EMPTY)
	UCSR0B &= 0b11011111;
	
}
// ISR for receive interrupt. Any data we get from UDR0 is written to the receive buffer.
ISR(USART_RX_vect)
{
	unsigned char data = UDR0;
	
	// Note: This will fail silently and data will be lost if recvBuffer is full.
	writeBuffer(&_recvBuffer, data);
}

TResult readPacket(TComms *packet)
{
	// Reads in data from the serial port and
	// deserializes it.Returns deserialized
	// data in "packet".
	
	unsigned char buffer[PACKET_SIZE];
	int len;

	len = hear(buffer);

	if(len == 0)
	return PACKET_INCOMPLETE;
	else
	return deserialize(buffer, len, packet);
	
}

int hear(unsigned char line[])
{
	int count=0;
	
	TBufferResult result;
	do
	{
		result = readBuffer(&_recvBuffer, &line[count]);
		
		if(result == BUFFER_OK)
		count++;
	} while (result == BUFFER_OK);
	
	return count;
}

void say(const unsigned char *line, int size)
{
	TBufferResult result = BUFFER_OK;
	
	int i;
	
	for(i=1; i<size && result == BUFFER_OK; i++)
	{
		result = writeBuffer(&_xmitBuffer, line[i]);
	}
	
	// Send the first character
	UDR0=line[0];
	
	// Enable the UDRE interrupt. The enable bit is bit 5 of UCSR0B.
	UCSR0B |= 0b0010000;
}

void initBuffers()
{
	// Initialize the receive and transmit buffers.
	initBuffer(&_recvBuffer, RECV_SIZE);
	initBuffer(&_xmitBuffer, XMIT_SIZE);
}

void setupSerial()
{
	// To replace later with bare-metal.
	// Serial.begin(9600);
	
	UCSR0C = 0x06; // 8N1
	UCSR0A = 0;
	UBRR0L = 103; // setting baud rate for 9600?
	UBRR0H = 0;

}

void startSerial()
{
	// Empty for now. To be replaced with bare-metal code
	// later on.
	UCSR0B = 0b10111000; // interrupt mode
	
}

void waitForHello()
{
	int exit=0;

	while(!exit)
	{
		TComms hello;
		TResult result;
		
		do
		{
			result = readPacket(&hello);
		} while (result == PACKET_INCOMPLETE);

		if(result == PACKET_OK)
		{
			if(hello.packetType == PACKET_TYPE_HELLO)
			{
				

				sendOK();
				exit=1;
			}
			else
			sendBadResponse();
		}
		else
		if(result == PACKET_BAD)
		{
			sendBadPacket();
		}
		else
		if(result == PACKET_CHECKSUM_BAD)
		sendBadChecksum();
	} // !exit
}

void setup() {
	// put your setup code here, to run once:

	cli();
	setupEINT();
	setupSerial();
	setupMotors();
	startMotors();
	enablePullups();
	initializeState();
	sei();

	//compute vincent diagonal
	vincentDiagonal = sqrt((VINCENT_LENGTH * VINCENT_LENGTH) + (VINCENT_BREADTH *VINCENT_BREADTH));
	vincentCirc = PI * vincentDiagonal;
	
}

void handleCommand(TComms *command)
{
  switch(command->command)
  {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
        sendOK();
        forward((float) command->params[0], (float) command->params[1]);
      break;

    /*
     * Implement code for other commands here.
     * 
     */
        
    default:
      sendBadCommand();
  }
}

void handlePacket(TComms *packet)
{
	switch(packet->packetType)
	{
		case PACKET_TYPE_COMMAND:
		handleCommand(packet);
		break;

		case PACKET_TYPE_RESPONSE:
		break;

		case PACKET_TYPE_ERROR:
		break;

		case PACKET_TYPE_MESSAGE:
		break;

		case PACKET_TYPE_HELLO:
		break;
	}
}



int main() {
	TComms recvPacket; // This holds commands from the Pi

	TResult result = readPacket(&recvPacket);
	
	if(result == PACKET_OK)
	handlePacket(&recvPacket);
	else
	if(result == PACKET_BAD)
	{
		sendBadPacket();
	}
	else
	if(result == PACKET_CHECKSUM_BAD)
	{
		sendBadChecksum();
	}
	
	if(deltaDist > 0){
		if(dir==FORWARD){
			if(forwardDist > newDist){
				deltaDist=0;
				newDist=0;
				stop();
			}
		}
		else
		if(dir == BACKWARD){
			if(reverseDist > newDist){
				deltaDist=0;
				newDist=0;
				stop();
			}
		}
		else
		if(dir == STOP){
			deltaDist=0;
			newDist=0;
			stop();
		}
	}
}