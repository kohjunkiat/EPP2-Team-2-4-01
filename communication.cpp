#include "serialize.cpp"
#include "constants.h"
#include "serialize .h"
static TBuffer _recvBuffer, _xmitBuffer;

/*
 * 
 * Vincent Communication Routines.
 * 
 */
 
TResult readPacket(TComms *packet)
{
    // Reads in data from the serial port and
    // deserializes it.Returns deserialized
    // data in "packet".
    
    char buffer[PACKET_SIZE];
    int len;

    len = readSerial(buffer);

    if(len == 0)
      return PACKET_INCOMPLETE;
    else
      return deserialize(buffer, len, packet);
    
}

void sendStatus()
{
  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.
  //
}

void sendMessage(const char *message)
{
  // Sends text messages back to the Pi. Useful
  // for debugging.
  
  TComms messagePacket;
  messagePacket.packetType=PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void sendBadPacket()
{
  // Tell the Pi that it sent us a packet with a bad
  // magic number.
  
  TComms badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);
  
}

void sendBadChecksum()
{
  // Tell the Pi that it sent us a packet with a bad
  // checksum.
  
  TComms badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);  
}

void sendBadCommand()
{
  // Tell the Pi that we don't understand its
  // command sent to us.
  
  TComms badCommand;
  badCommand.packetType=PACKET_TYPE_ERROR;
  badCommand.command=RESP_BAD_COMMAND;
  sendResponse(&badCommand);

}

void sendBadResponse()
{
  TComms badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK()
{
  TComms okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);  
}

void sendResponse(TComms *packet)
{
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TComms));
  writeSerial(buffer, len);
}

ISR(USART_UDRE_vect)
{
  unsigned char data;
  TResult result = readBuffer(&_xmitBuffer, &data);
  
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

// Write a string to the UART.
// Will silently truncate string if the buffer is full
void say(const unsigned char *line, int size)
{
  TResult result = BUFFER_OK;
  
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

// Read from the UART.
int hear(unsigned char *line)
{
  int count=0;
  
  TResult result;
  do 
  {
    result = readBuffer(&_recvBuffer, &line[count]);
    
    if(result == BUFFER_OK)
      count++;
  } while (result == BUFFER_OK);
  
  return count;
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

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial()
{
  // Empty for now. To be replaced with bare-metal code
  // later on.
  UCSR0B = 0b10111000; // interrupt mode
  
}


