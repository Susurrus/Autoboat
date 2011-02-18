import controlP5.*;
import processing.serial.*;
import gnu.io.PortInUseException;

ControlP5 controlP5;

DropdownList serialPortsList;

Serial myPort;

// Used for parsing message data
byte[] message = new byte[64];
int messageIndex = 0;
int messageState = 0;
byte[] inBuffer = new byte[255];

// Boat state data
float[] L2 = new float[3];
float[] globalPosition = new float[3];
float heading = 0;
float[] localPosition = new float[3];
float[] velocity = new float[3];

void setup() {
  size(800, 600);
  frameRate(30);
  controlP5 = new ControlP5(this);
  serialPortsList = controlP5.addDropdownList("serialPortsList",100,100,100,120);
  customizeSerialPortsList(serialPortsList);
}

void draw() {
  // Restore the cursor to the regular arrow.
  cursor(ARROW);
  
  // Redraw the background at every timestep
  // (Necessary for clearing things like the dropdown)
  background(128);
  
  // Grab some data from the serial port
  // TODO: Grab serial data faster.
  if (myPort != null && myPort.available() > 0) {
    //println(myPort.available());
    fill(0,255,0); 
    buildAndCheckMessage((byte)myPort.read());
    //for (int i=0;i<myPort.readBytes(inBuffer);i++) {
    //  buildAndCheckMessage(inBuffer[i]);
    //}
      //println(char(inBuffer));
  } else {
    fill(0,100,0);
  }
  // Draw the RX status
  arc(90, 90, 10, 10, 0, TWO_PI);
  // Draw the TX status
  fill(100,0,0);
  arc(75, 90, 10, 10, 0, TWO_PI);
  
  // Draw the L2 vector values
  text("L2 Vector", 300, 290);
  text(L2[0], 300, 300);
  text(L2[1], 300, 310);
  text(L2[2], 300, 320);
  
  // Draw the velocity vector values
  text("Velocity", 200, 290);
  text(velocity[0], 200, 300);
  text(velocity[1], 200, 310);
  text(velocity[2], 200, 320);
  
  // Draw the local position values
  text("Local position", 200, 390);
  text(localPosition[0], 200, 400);
  text(localPosition[1], 200, 410);
  text(localPosition[2], 200, 420);
  
  // Draw the local position values
  text("Global position", 300, 390);
  text(globalPosition[0], 300, 400);
  text(globalPosition[1], 300, 410);
  text(globalPosition[2], 300, 420);
  
  // Display the boat heading
  text("Heading", 400, 290);
  text(heading, 400, 300);
  pushMatrix();
  translate(420,350);
  rotate(heading);
  triangle(10,30,0,-30,-10,30);
  popMatrix();
}

void controlEvent(ControlEvent theEvent) {
  // PulldownMenu is of type ControlGroup.
  // A controlEvent will be triggered from within the ControlGroup.
  // therefore you need to check the originator of the Event with
  // if (theEvent.isGroup())
  // to avoid an error message from controlP5.

  if (theEvent.isGroup()) {
    // check if the Event was triggered from a ControlGroup
    cursor(WAIT);
    if (myPort != null) {
      myPort.stop();
    }
    
    // TODO: This try/catch statement needs to be fixed to properly suppress the error
    // warning from gnu.io.PortInUseException and inform the user.
    try {
      myPort = new Serial(this, theEvent.group().stringValue(), 57600);
    }
    catch (Exception e) {
      println("Port in use or otherwise unavailable. Please select another.");
    }
  }
}

void customizeSerialPortsList(DropdownList ddl) {
  ddl.setBackgroundColor(color(190));
  ddl.setItemHeight(20);
  ddl.setBarHeight(15);
  
  ddl.captionLabel().set("Select serial port");
  ddl.captionLabel().style().marginTop = 3;
  ddl.captionLabel().style().marginLeft = 3;
  ddl.valueLabel().style().marginTop = 3;
  
  String[] ports = Serial.list();
  for(int i=0;i<ports.length;i++) {
    ddl.addItem(ports[i],i);
  }
  
  ddl.setColorBackground(color(60));
  ddl.setColorActive(color(255,128));
}

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
	
print(char(characterIn));
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
		} else if (messageIndex == 62) {
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
		} else if (messageIndex == 63) {
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
                  println("Yay! Successfully parsed a new data!");
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
  InputStream in = new ByteArrayInputStream(message);
  DataInputStream din = new DataInputStream(in);
  try {
    L2[0] = din.readFloat();
    L2[1] = din.readFloat();
    L2[2] = din.readFloat();
    globalPosition[0] = din.readFloat();
    globalPosition[1] = din.readFloat();
    globalPosition[2] = din.readFloat();
    heading = din.readFloat();
    localPosition[0] = din.readFloat();
    localPosition[1] = din.readFloat();
    localPosition[2] = din.readFloat();
    velocity[0] = din.readFloat();
    velocity[1] = din.readFloat();
    velocity[2] = din.readFloat();
  } catch (Exception e) {
    println("Crap, failed to extract the data");
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
