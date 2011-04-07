import com.jmatio.types.*;
import com.jmatio.io.*;

// GUI Toolkit
import controlP5.*;

// Serial port interface
import processing.serial.*;

import java.util.Calendar;

// Used for parsing message data
byte[] message = new byte[64];
int messageIndex = 0;
int messageState = 0;

// Internal program state
boolean recording = false;
Long recordedMessages = new Long(0);
Serial myPort;
ControlTimer recordTimer;
ControlP5 controlP5;
DropdownList serialPortsList;
Textlabel recordTimerLabel;
Textlabel recordedMessagesLabel;
Toggle recordToggle;
Button selectMatFile;


// Reference
PVector trueNorth = new PVector(0, 1, 0);

// Current timestep boat state data
PVector L2 = new PVector(0,0,0);
PVector globalPosition = new PVector(0,0,0);
float heading = 0;
PVector localPosition = new PVector(0,0,0);
PVector velocity = new PVector(0,0,0);

// Boat state data recording
ArrayList<float[]> L2List = new ArrayList<float[]>(255);
ArrayList<float[]> globalPositionList = new ArrayList<float[]>(255);
ArrayList<Float> headingList = new ArrayList<Float>(255);
ArrayList<float[]> localPositionList = new ArrayList<float[]>(255);
ArrayList<float[]> velocityList = new ArrayList<float[]>(255);

void setup() {
  size(800, 600);
  frameRate(30);
  controlP5 = new ControlP5(this);
  serialPortsList = controlP5.addDropdownList("serialPortsList",100,100,100,120);
  serialPortsList.setId(0);
  recordToggle = controlP5.addToggle("record",false,100,100,20,20);
  recordToggle.setId(1);
  recordTimer = new ControlTimer();
  recordTimer.setSpeedOfTime(1);
  recordTimerLabel = controlP5.addTextlabel("recordTimer",recordTimer.toString(),20,100);
  recordedMessagesLabel = controlP5.addTextlabel("recordedMessages","Messages: " + recordedMessages.toString(),20,120);
  selectMatFile = controlP5.addButton("Select .mat file",0,400,50,100,20);
  selectMatFile.setId(2);
  customizeSerialPortsList(serialPortsList);
}

void draw() {
  // Redraw the background at every timestep
  // (Necessary for clearing things like the dropdown)
  background(0);
  
  // Restore the cursor to the regular arrow.
  cursor(ARROW);
  
  // Grab some data from the serial port
  // TODO: Grab serial data faster.
  if (myPort != null && myPort.available() > 0) {
    fill(0,255,0);
    while(myPort.available() > 0) {
      byte[] inBuffer = new byte[7];
      int bytesToRead = myPort.readBytes(inBuffer);
      for (int i=0;i<bytesToRead;i++) {
        buildAndCheckMessage(inBuffer[i]);
      }
    }
  } else {
    fill(0,100,0);
  }
  // Draw the RX status
  arc(90, 90, 10, 10, 0, TWO_PI);
  // Draw the TX status
  fill(100,0,0);
  arc(75, 90, 10, 10, 0, TWO_PI);
  
  // Reset fill color to white
  fill(255);
  
  // Draw the L2 vector values
  text("L2 Vector", 300, 290);
  text(L2.x, 300, 300);
  text(L2.y, 300, 310);
  text(L2.z, 300, 320);
  
  pushMatrix();
  fill(0,0,255);
  translate(320,220);
  float rotation = PVector.angleBetween(L2, trueNorth);
  if (!Float.isNaN(rotation)) {
    rotate(rotation);
  }
  scale(L2.mag()/3);
  triangle(10,30,0,-30,-10,30);
  popMatrix();
  
  // Reset fill color to white
  fill(255);
  
  // Draw the velocity vector values
  text("Velocity", 200, 290);
  text(velocity.x, 200, 300);
  text(velocity.y, 200, 310);
  text(velocity.z, 200, 320);
  
  // Draw the local position values
  text("Local position", 200, 390);
  text(localPosition.x, 200, 400);
  text(localPosition.y, 200, 410);
  text(localPosition.z, 200, 420);
  
  // Draw the local position values
  text("Global position", 300, 390);
  text(globalPosition.x, 300, 400);
  text(globalPosition.y, 300, 410);
  text(globalPosition.z, 300, 420);
  
  // Display the boat heading
  text("Heading", 400, 290);
  text(heading, 400, 300);
  pushMatrix();
  fill(155,155,0);
  translate(420,220);
  rotate(heading);
  triangle(10,30,0,-30,-10,30);
  popMatrix();
  
  // Reset fill color to white
  fill(255);
  
  if (recording) {
    recordTimerLabel.setValue(recordTimer.toString());
    recordedMessagesLabel.setValue("Messages: " + recordedMessages.toString());
  }
}

void controlEvent(ControlEvent theEvent) {
  // PulldownMenu is of type ControlGroup.
  // A controlEvent will be triggered from within the ControlGroup.
  // therefore you need to check the originator of the Event with
  // if (theEvent.isGroup())
  // to avoid an error message from controlP5.

  switch (theEvent.id()) {
    case(0):
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
      break;
    case(1):
      println("toggle "+theEvent.value());
      if (theEvent.value() == 0) {
        stopRecordingAndSave();
      } else if (theEvent.value() == 1) {
        startRecording();
      } 
      break;
    case(2):
      println(".mat");
      break;
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

public void startRecording() {
  recording = true;
  recordTimer.reset();
}

public void stopRecordingAndSave() {
  recording = false;
  float[][] t = new float[L2List.size()][3];
  L2List.toArray(t);
  double[][] output = new double[L2List.size()][3];
  for (int i = 0; i < L2List.size(); i++){
    for (int j=0;j<3;j++) {
      output[i][j] = (double)t[i][j];
    }
  }
  MLDouble L2Double = new MLDouble("L2", output);
  globalPositionList.toArray(t);
  output = new double[L2List.size()][3];
  for (int i = 0; i < L2List.size(); i++){
    for (int j=0;j<3;j++) {
      output[i][j] = (double)t[i][j];
    }
  }
  
  MLDouble globalPositionDouble = new MLDouble("globalPosition", output);
  localPositionList.toArray(t);
  output = new double[L2List.size()][3];
  for (int i = 0; i < L2List.size(); i++){
    for (int j=0;j<3;j++) {
      output[i][j] = (double)t[i][j];
    }
  }
  
  Float[] g = new Float[L2List.size()];
  double[] gg = new double[L2List.size()];
  headingList.toArray(g);
  for (int i = 0; i < L2List.size(); i++){
    gg[i] = (double)g[i];
  }
  MLDouble headingDouble = new MLDouble("heading", gg, L2List.size());
  
  MLDouble localPositionDouble = new MLDouble("localPosition", output);
  velocityList.toArray(t);
  output = new double[L2List.size()][3];
  for (int i = 0; i < L2List.size(); i++){
    for (int j=0;j<3;j++) {
      output[i][j] = (double)t[i][j];
    }
  }
  
  MLDouble velocityDouble = new MLDouble("velocity", output);
  
  ArrayList matList = new ArrayList();
  matList.add(L2Double);
  matList.add(headingDouble);
  matList.add(globalPositionDouble);
  matList.add(localPositionDouble);
  matList.add(velocityDouble);
  
  try {
    Calendar cal = Calendar.getInstance();
    SimpleDateFormat sdf = new SimpleDateFormat("yyyyMMdd-hhmmss");
    new MatFileWriter(sketchPath(sdf.format(cal.getTime())+".mat"), matList);
  }
  catch (Exception e){
    println("Failed to write output to a .mat file");
  }
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
  } catch (Exception e) {
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
