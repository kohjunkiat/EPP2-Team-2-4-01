void sendResponse(TComms *packet)
{
	// Takes a packet, serializes it then sends it out
	// over the serial port.
	unsigned char buffer[PACKET_SIZE];
	int len;

	len = serialize(buffer, packet, sizeof(TComms));
	say(buffer, len);
}

void sendStatus()
{
	TComms statusPacket;
	statusPacket.packetType = PACKET_TYPE_RESPONSE;
	statusPacket.command = RESP_STATUS;
	statusPacket.params[0] = leftForwardTicks;
	statusPacket.params[1] = rightForwardTicks;
	statusPacket.params[2] = leftReverseTicks;
	statusPacket.params[3] = rightReverseTicks;
	statusPacket.params[4] = leftForwardTicksTurns;
	statusPacket.params[5] = rightForwardTicksTurns;
	statusPacket.params[6] = leftReverseTicksTurns;
	statusPacket.params[7] = rightReverseTicksTurns;
	statusPacket.params[8] = forwardDist;
	statusPacket.params[9] = reverseDist;
	sendResponse(&statusPacket);
}

void sendMessage(const char *message)
{
	// Sends text messages back to the Pi. Useful
	// for debugging.
	
	TComms messagePacket;
	messagePacket.packetType=PACKET_TYPE_MESSAGE;
	strncpy(messagePacket.buffer, message, MAX_DATA_SIZE);
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