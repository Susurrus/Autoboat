import com.jmatio.types.*;
import com.jmatio.io.*;

// GUI Toolkit
import controlP5.*;

// Serial port interface
import processing.serial.*;

import java.util.Calendar;

// Internal program state
boolean recording = false;
boolean playing = false;
int playbackIndex = 0;
int lastPlaybackTime = 0;
MatFileReader inputMatFileReader;
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
float heading = 0.0;
PVector localPosition = new PVector(0,0,0);
PVector velocity = new PVector(0,0,0);
PVector waypoint0 = new PVector(0,0,0);
PVector waypoint1 = new PVector(0,0,0);
int rudderPot = 0;
boolean rudderPortLimit = false;
boolean rudderSbLimit = false;

// Boat state data recording
ArrayList<float[]> L2List = new ArrayList<float[]>(255);
ArrayList<float[]> globalPositionList = new ArrayList<float[]>(255);
ArrayList<Float> headingList = new ArrayList<Float>(255);
ArrayList<float[]> localPositionList = new ArrayList<float[]>(255);
ArrayList<float[]> velocityList = new ArrayList<float[]>(255);

// Boat state data playback
double[][] L2Playback;
double[][] globalPositionPlayback;
double[] headingPlayback;
double[][] localPositionPlayback;
double[][] velocityPlayback;

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
  if (myPort != null && myPort.available() > 0 && !playing) {
    fill(0,255,0);
    while(myPort.available() > 0) {
      byte[] inBuffer = new byte[7];
      int bytesToRead = myPort.readBytes(inBuffer);
      for (int i=0;i<bytesToRead;i++) {
        buildAndCheckMessage(inBuffer[i]);
      }
    }
  } else if (playing) {
    if (playbackIndex < headingPlayback.length) {
      if (millis() - lastPlaybackTime >= 10) { // Assume .01s sampletime
        L2.x = (float)L2Playback[playbackIndex][0];
        L2.y = (float)L2Playback[playbackIndex][1];
        L2.z = (float)L2Playback[playbackIndex][2];
        globalPosition.x = (float)globalPositionPlayback[playbackIndex][0];
        globalPosition.y = (float)globalPositionPlayback[playbackIndex][1];
        globalPosition.z = (float)globalPositionPlayback[playbackIndex][2];
        heading = (float)headingPlayback[playbackIndex];
        localPosition.x = (float)localPositionPlayback[playbackIndex][0];
        localPosition.y = (float)localPositionPlayback[playbackIndex][1];
        localPosition.z = (float)localPositionPlayback[playbackIndex][2];
        velocity.x = (float)velocityPlayback[playbackIndex][0];
        velocity.y = (float)velocityPlayback[playbackIndex][1];
        velocity.z = (float)velocityPlayback[playbackIndex][2];
        
        playbackIndex++;
        lastPlaybackTime = millis();
      }
    } else {
      playing = false;
    }
    fill(0,100,0);
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
  
  // Add rudder information
  text("Rudder", 400, 390);
  text(rudderPot, 400, 400);
  text(rudderPortLimit?"true":"false", 400, 410);
  text(rudderSbLimit?"true":"false", 400, 420);
  
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
  
  // Draw the current and next waypoints
  text("Waypoint0", 50, 290);
  text(waypoint0.x, 50, 300);
  text(waypoint0.y, 50, 310);
  text(waypoint0.z, 50, 320);
  text("Waypoint1", 50, 390);
  text(waypoint1.x, 50, 400);
  text(waypoint1.y, 50, 410);
  text(waypoint1.z, 50, 420);
  
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
        myPort = new Serial(this, theEvent.group().stringValue(), 115200);
      }
      catch (Exception e) {
        println("Port in use or otherwise unavailable. Please select another.");
      }
      break;
    case(1):
      if (theEvent.value() == 0) {
        stopRecordingAndSave();
      } else if (theEvent.value() == 1) {
        startRecording();
      } 
      break;
    case(2):
      String inputMatFileName = selectInput();
      cursor(WAIT);
      if (inputMatFileName != null) {
        try {
          inputMatFileReader = new MatFileReader(inputMatFileName);
          L2Playback = ((MLDouble)inputMatFileReader.getMLArray("L2")).getArray().clone();
          globalPositionPlayback = ((MLDouble)inputMatFileReader.getMLArray("globalPosition")).getArray().clone();
          double[][] data = ((MLDouble)inputMatFileReader.getMLArray("heading")).getArray();
          headingPlayback = new double[data.length];
          for (int i=0;i<data.length;i++) {
            headingPlayback[i] = data[i][0];
          }
          localPositionPlayback = ((MLDouble)inputMatFileReader.getMLArray("localPosition")).getArray().clone();
          velocityPlayback = ((MLDouble)inputMatFileReader.getMLArray("velocity")).getArray().clone();
        } catch (IOException e) {
          e.printStackTrace();
          exit();
        }
        
        if (inputMatFileReader != null) {
          playing = true;
          playbackIndex = 0;
        }
      }
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

