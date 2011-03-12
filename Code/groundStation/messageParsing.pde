

// Used for parsing message data
byte[] message = new byte[128];
int messageIndex = 0;
int messageState = 0;

/**
 * Parses another character from the serialport
 */
void buildAndCheckMessage(byte characterIn) {

	// This contains the function's state of whether
	// it is currently building a message.
	// 0 - Awaiting header byte 0 (%)
	// 1 - Awaiting header byte 1 (&)
	// 2 - Building message
	// 3 - Awaiting header byte 0 (^)
	// 4 - Awaiting header byte 1 (&)
	// 5 - Reading checksum character
	
	// We start recording a new message if we see the header
	if (messageState == 0) {
		if (characterIn == '%') {
			message[0] = characterIn;
			messageIndex = 1;
			messageState = 1;
		}
	} else if (messageState == 1) {
		// If we don't find the necessary ampersand we start over
		// waiting for a new sentence
		if (characterIn == '&') {
			message[1] = characterIn;
			messageIndex = 2;
			messageState = 2;
		} else {
			messageIndex = 0;
			messageState = 0;
		}
	} else if (messageState == 2) {
		// Record every character that comes in now that we're building a sentence.
		// Only stop if we run out of room or an asterisk is found.
		message[messageIndex++] = characterIn;
		if (characterIn == '^') {
			messageState = 3;
		} else if (messageIndex == 126) {
			// If we've filled up the buffer, ignore the entire message as we can't store it all
			messageState = 0;
			messageIndex = 0;
		}
	} else if (messageState == 3) {
		// If we don't find the necessary ampersand we continue
		// recording data as we haven't found the footer yet until
		// we've filled up the entire message (ends at 124 characters
		// as we need room for the 2 footer chars).
		message[messageIndex++] = characterIn;
		if (characterIn == '&') {
			messageState = 4;
		} else if (messageIndex == 127) {
			messageState = 0;
			messageIndex = 0;
		} else {
			messageState = 3;
		}
	} else if (messageState == 4) {
		// Record the second ASCII-hex character of the checksum byte.
		message[messageIndex] = characterIn;

		// The checksum is now verified and if successful the message
		// is stored in the appropriate struct.
		if (message[messageIndex] == calculateChecksum(subset(message, 2, messageIndex-4))) {
			// We now memcpy all the data into our global data structs.
			// NOTE: message[3] is used to skip the header & message ID info
			switch (message[2]) {
				case 1:
					break;
				case 2:
					break;
				case 3:
                                        updateStateData(subset(message, 3, messageIndex-4));
					break;
				case 4:
					break;
			}
		}
		
		// We clear all state variables here regardless of success.
		messageIndex = 0;
		messageState = 0;
		int b;
		for (b = 0;b < 64;b++) {
			message[b] = 0;
		}
	}
}

void updateStateData(byte message[]) {  
  // Wrap the message in a DataInputStream so that readFloat() can be used
  InputStream in = new ByteArrayInputStream(message);
  DataInputStream din = new DataInputStream(in);
  
  // Save the last set of data into an array list
  if (recording) {
    L2List.add(L2.array().clone());
    globalPositionList.add(globalPosition.array().clone());
    headingList.add(heading);
    localPositionList.add(localPosition.array().clone());
    velocityList.add(velocity.array().clone());
    waypoint0List.add(waypoint0.array().clone());
    waypoint1List.add(waypoint1.array().clone());
    rudderPotList.add(rudderPot);
    rudderPortLimitList.add((byte)(rudderPortLimit?1:0));
    rudderSbLimitList.add((byte)(rudderSbLimit?1:0));
    gpsYearList.add(gpsYear);
    gpsMonthList.add(gpsMonth);
    gpsDayList.add(gpsDay);
    gpsHourList.add(gpsHour);
    gpsMinuteList.add(gpsMinute);
    gpsSecondList.add(gpsSecond);
    gpsCourseList.add(gpsCourse);
    gpsSpeedList.add(gpsSpeed);
    gpsHdopList.add(gpsHdop);
    gpsFixList.add(gpsFix);
    gpsSatellitesList.add(gpsSatellites);
    
    recordedMessages++;
  }
  
  // Import new data from a complete StateData message
  try {
    L2.x = din.readFloat();
    L2.y = din.readFloat();
    L2.z = din.readFloat();
    globalPosition.x = din.readFloat();
    globalPosition.y = din.readFloat();
    globalPosition.z = din.readFloat();
    heading = din.readFloat();
    localPosition.x = din.readFloat();
    localPosition.y = din.readFloat();
    localPosition.z = din.readFloat();
    velocity.x = din.readFloat();
    velocity.y = din.readFloat();
    velocity.z = din.readFloat();
    waypoint0.x = din.readInt();
    waypoint0.y = din.readInt();
    waypoint0.z = din.readInt();
    waypoint1.x = din.readInt();
    waypoint1.y = din.readInt();
    waypoint1.z = din.readInt();
    rudderPot = din.readUnsignedShort();
    rudderPortLimit = din.readBoolean();
    rudderSbLimit = din.readBoolean();
    gpsLatitude = din.readFloat();
    gpsLongitude = din.readFloat();
    gpsAltitude = din.readFloat();
    gpsYear = din.readByte();
    gpsMonth = din.readByte();
    gpsDay = din.readByte();
    gpsHour = din.readByte();
    gpsMinute = din.readByte();
    gpsSecond = din.readByte();
    gpsCourse = din.readFloat();
    gpsSpeed = din.readFloat();
    gpsHdop = din.readFloat();
    gpsFix = din.readByte();
    gpsSatellites = din.readByte();
  } catch (Exception e) {
    e.printStackTrace();
    println("Crap, failed to extract the data");
    exit();
  }
}

/**
 * This function calculates the checksum of some bytes in an
 * array by XORing all of them.
 */
byte calculateChecksum(byte sentence[]) {

  byte checkSum = 0;
  for (int i = 0; i < sentence.length; i++) {
    checkSum ^= sentence[i];
  }
	
  return checkSum;
}
