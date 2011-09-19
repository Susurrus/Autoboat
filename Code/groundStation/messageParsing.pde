

// Used for parsing message data
byte[] message = new byte[128];
int messageIndex = 0;
int messageState = 0;

/**
 * Parses another character from the serialport
 */
boolean buildAndCheckMessage(byte characterIn) {

        boolean success = false;
  
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
		} else if (characterIn != '%'){
			messageIndex = 0;
			messageState = 0;
		}
	} else if (messageState == 2) {
		// Record every character that comes in now that we're building a sentence.
		// Only stop if we run out of room or an asterisk is found.
		message[messageIndex++] = characterIn;
		if (messageIndex == message[3] + 5) {
                  if (characterIn == '^') {
			messageState = 3;
                  } else {
                         messageIndex = 0;
                         messageState = 0;
                  }
		} else if (messageIndex == message.length - 3) {
			// If we've filled up the buffer, ignore the entire message as we can't store it all
			messageState = 0;
			messageIndex = 0;
		}
	} else if (messageState == 3) {
		// If we find an ampersand, then we've found the footer. If we've run out of space at 127 bytes,
                // error out, if we've found the first footer char, keep waiting for the second one, otherwise
                // return to the previous state to read and record characters.
		message[messageIndex++] = characterIn;
		if (characterIn == '&') {
			messageState = 4;
		} else if (messageIndex == message.length - 2) {
			messageState = 0;
			messageIndex = 0;
		}
	} else if (messageState == 4) {
		// Record the second ASCII-hex character of the checksum byte.
		message[messageIndex] = characterIn;
		// The checksum is now verified and if successful the message
		// is stored in the appropriate struct.
		if (message[messageIndex] == calculateChecksum(subset(message, 2, messageIndex-4)) && message[3] == messageIndex - 6) {
			// NOTE: message[2] is used to skip the header, message ID.
			if (message[2] == 3) {
                                updateStateData(subset(message, 4, messageIndex-6));
                                success = true;
			}
		}
		
		// We clear all state variables here regardless of success.
		messageIndex = 0;
		messageState = 0;
		int b;
		for (b = 0;b < messageIndex;b++) {
			message[b] = 0;
		}
	}

        return success;
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
    resetList.add(reset);
    loadList.add(load);
    rudderAngleList.add(rudderAngle);
    propRpmList.add(propRpm);
    statusBitsList.add(statusBits);
    orderingList.add(ordering);
    rudderAngleCommandList.add(rudderAngleCommand);
    throttleCommandList.add(throttleCommand);
    batteryVoltageList.add(batteryVoltage);
    batteryAmperageList.add(batteryAmperage);
    lowRudderCalLimitList.add(lowRudderCalLimit);
    highRudderCalLimitList.add(highRudderCalLimit);
    
    recordedMessages++;
  }
  
  // Import new data from a complete StateData message
  try {
    L2.x = din.readFloat();
    L2.y = din.readFloat();
    globalPosition.x = din.readFloat();
    globalPosition.y = din.readFloat();
    heading = din.readFloat();
    localPosition.x = din.readFloat();
    localPosition.y = din.readFloat();
    velocity.x = din.readFloat();
    velocity.y = din.readFloat();
    waypoint0.x = din.readInt();
    waypoint0.y = din.readInt();
    waypoint1.x = din.readInt();
    waypoint1.y = din.readInt();
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
    reset = din.readByte();
    load = din.readByte();
    rudderAngle = din.readFloat();
    propRpm = din.readShort();
    statusBits = din.readByte();
    ordering = din.readByte();
    rudderAngleCommand = din.readFloat();
    throttleCommand = din.readShort();
    batteryVoltage = din.readFloat();
    batteryAmperage = din.readFloat();
    lowRudderCalLimit = din.readShort();
    highRudderCalLimit = din.readShort();
    
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
