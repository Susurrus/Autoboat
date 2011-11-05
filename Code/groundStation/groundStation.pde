// GUI Toolkit
import controlP5.*;

// Serial port interface
import processing.serial.*;

import java.util.Calendar;

// Internal program state
boolean recording = false;
int playbackIndex = 0;
int lastPlaybackTime = 0;
Long recordedMessages = new Long(0);
Serial myPort;
ControlTimer recordTimer;
ControlP5 controlP5;
DropdownList serialPortsList;
Textlabel recordTimerLabel;
Textlabel recordedMessagesLabel;
Toggle recordToggle;
PrintWriter csvOutput;

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
float gpsLatitude = 0.0;
float gpsLongitude = 0.0;
float gpsAltitude = 0.0;
byte gpsYear = 0;
byte gpsMonth = 0;
byte gpsDay = 0;
byte gpsHour = 0;
byte gpsMinute = 0;
byte gpsSecond = 0;
float gpsCourse = 0.0;
float gpsSpeed = 0.0;
float gpsHdop = 0.0;
byte gpsFix = 0;
byte gpsSatellites = 0;
byte reset = 0;
byte load = 0;
float rudderAngle = 0.0;
int propRpm = 0;
byte statusBits = 0;
byte ordering = 0;
float rudderAngleCommand = 0.0;
int throttleCommand = 0;
float batteryVoltage = 0.0;
float batteryAmperage = 0.0;
int lowRudderCalLimit;
int highRudderCalLimit;
float revoHeading;
byte revoMagStatus;
float revoPitch;
byte revoPitchStatus;
float revoRoll;
byte revoRollStatus;
float revoDip;
int revoMagneticMagnitude;
float windSpeed;
float windDirection;
float airTemp;
float airPressure;
float airHumidity;
float waterSpeed;
float waterTemp;
float waterDepth;

// Rendering variables
PFont regularFont;
PFont boldFont;

void setup() {
  size(1000, 600);
  frameRate(50);
  
  regularFont = loadFont("DejaVuSansMono-15.vlw");
  textFont(regularFont);
  boldFont = loadFont("DejaVuSansMono-Bold-15.vlw");
  
  controlP5 = new ControlP5(this);
  serialPortsList = controlP5.addDropdownList("serialPortsList",100,100,100,120);
  serialPortsList.setId(0);
  recordToggle = controlP5.addToggle("record",false,100,100,20,20);
  recordToggle.setId(1);
  recordTimer = new ControlTimer();
  recordTimer.setSpeedOfTime(1);
  recordTimerLabel = controlP5.addTextlabel("recordTimer",recordTimer.toString(),20,100);
  recordedMessagesLabel = controlP5.addTextlabel("recordedMessages",String.format("Messages: %d", recordedMessages),20,120);
  customizeSerialPortsList(serialPortsList);
}

int lastDrawTime = 0;

void draw() {
  // Redraw the background at every timestep
  // (Necessary for clearing things like the dropdown)
  background(0);
  
  // Restore the cursor to the regular arrow.
  cursor(ARROW);
  
  // Reset fill to green (used for drawing the RX status circle)
  fill(0,100,0);
  
  // Grab some data from the serial port
  if (myPort != null && myPort.available() > 0) {
    int bytesProcessed = 0;
    while(myPort.available() > 0) {
      byte[] inBuffer = new byte[127];
      int bytesToRead = myPort.readBytes(inBuffer);
      for (int i = 0; i < bytesToRead; i++) {
        if (buildAndCheckMessage(inBuffer[i])) {
              fill(0,255,0);
        }
      }
    }
  }

  // Draw the RX status
  arc(90, 90, 10, 10, 0, TWO_PI);
  // Draw the TX status
  fill(100,0,0);
  arc(75, 90, 10, 10, 0, TWO_PI);
  
  // Reset fill color to white
  fill(255);
  
  // Add text section headers
  textFont(boldFont);
  text("Rudder", 50, 490);
  text("GPS", 400, 390);
  text("Velocity", 200, 270);
  text("Throttle command", 200, 340);
  text("Local position", 200, 390);
  text("Global position", 200, 450);
  text("Waypoint0", 50, 290);
  text("Waypoint1", 50, 350);
  text("Power", 50, 420);
  text("Heading", 400, 290);
  text("Rudder angle command", 400, 340);
  text("Load", 570, 290);
  text("Reset status", 500, 70);
  text("Rudder angle", 630, 290);
  text("Prop speed", 630, 350);
  text("Mode:", 630, 400);
  text("Status:", 20, 170);
  textFont(regularFont);
  text("(press SPACE to toggle)", 100, 150);
  
  // Draw the rudder angle in degrees
  text(String.format("%3.1f\u00B0", rudderAngle * 180 / 3.14159), 630, 310);
  
  // Draw the prop speed in RPM
  text(String.format("%3d RPM", propRpm), 630, 370);

  // Draw the operational mode of the boat
  if ((statusBits & 0x01) != 0) {
    fill(0, 200, 0);
    textFont(boldFont);
    text("auto", 680, 400);
    textFont(regularFont);
    fill(255);
  } else {
    fill(200, 0, 0);
    textFont(boldFont);
    text("manual", 680, 400);
    textFont(regularFont);
    fill(255);
  }
  
  // Plot the relative position of the boat and the current waypoints.
  // First draw the field
  noFill();
  stroke(255);
  strokeWeight(1);
  rect(550, 410, 180, 180);
  noStroke();

  PVector origin = new PVector(0, 0);
  
  // Create a bounding box around the four points.
  float minX = origin.x; // Start with the origin.
  float minY = origin.y;
  float maxX = origin.x;
  float maxY = origin.y;
  if (localPosition.x < minX) {
    minX = localPosition.x;
  } else if (localPosition.x > maxX) {
    maxX = localPosition.x;
  }
  if (localPosition.y < minY) {
    minY = localPosition.y;
  } else if (localPosition.y > maxY) {
    maxY = localPosition.y;
  }
  if (waypoint0.x < minX) {
    minX = waypoint0.x;
  } else if (waypoint0.x > maxX) {
    maxX = waypoint0.x;
  }
  if (waypoint0.y < minY) {
    minY = waypoint0.y;
  } else if (waypoint0.y > maxY) {
    maxY = waypoint0.y;
  }
  if (waypoint1.x < minX) {
    minX = waypoint1.x;
  } else if (waypoint1.x > maxX) {
    maxX = waypoint1.x;
  }
  if (waypoint1.y < minY) {
    minY = waypoint1.y;
  } else if (waypoint1.y > maxY) {
    maxY = waypoint1.y;
  }
  
  // And then add a 5% buffer.
  minX *= 1.05;
  maxX *= 1.05;
  minY *= 1.05;
  maxY *= 1.05;
  
  // Generate an X- and Y- scaling factor, and use the biggest one for both axes to keep a square aspect ratio
  float plotScale = max((maxX - minX), (maxY - minY));
  text(String.format("Scale\n%2.1fm", plotScale), 550 + 185, 410 + 155);
  plotScale = 180/plotScale;
  
  // Determine the center of the bounding box
  float centerX = (maxX - minX)/2 + minX;
  float centerY = (maxY - minY)/2 + minY;
  
  // Remap points relative to this center, scale them into the new coordinate system and plot them.
  float waypoint0XRemapped = -plotScale * (waypoint0.x - centerX) + (410+180/2);
  float waypoint0YRemapped = plotScale * (waypoint0.y - centerY) + (550+180/2);
  float waypoint1XRemapped = -plotScale * (waypoint1.x - centerX) + (410+180/2);
  float waypoint1YRemapped = plotScale * (waypoint1.y - centerY) + (550+180/2);
  float localPositionXRemapped = -plotScale * (localPosition.x - centerX) + (410+180/2);
  float localPositionYRemapped = plotScale * (localPosition.y - centerY) + (550+180/2);
  float originXRemapped = -plotScale * (origin.x - centerX) + (410+180/2);
  float originYRemapped = plotScale * (origin.y - centerY) + (550+180/2);
  
  // Plot all the points within the alloted 180x180 square and display the scaling.
  // First scale the points appropriately and then subtract the position of the lower-left point from their position in essence creating a new origin.
  noStroke();
  rectMode(CENTER);
  fill(0, 128, 0);
  rect(waypoint0YRemapped, waypoint0XRemapped, 5, 5);
  
  fill(128, 0, 0);
  rect(waypoint1YRemapped, waypoint1XRemapped, 5, 5);
  
  stroke(128);
  line(waypoint0YRemapped, waypoint0XRemapped, waypoint1YRemapped, waypoint1XRemapped);
  
  noStroke();
  fill(128, 128, 0);
  rect(localPositionYRemapped, localPositionXRemapped, 5, 5);
  
  fill(128);
  rect(originYRemapped, originXRemapped, 5, 5);
  
  rectMode(CORNER);

  fill(255);
  // Draw the various possible states within the status bit.
  // First we output the state of the HIL actuator sensor override line
  int nextStatusLineStart = 190;
  if ((statusBits & 0x02) != 0) {
    text("HIL actuator sensor override", 20, nextStatusLineStart);
    nextStatusLineStart += 20;
  }
  if ((statusBits & 0x04) != 0) {
    text("RC receiver disconnected", 20, nextStatusLineStart);
    nextStatusLineStart += 20;
  }
  if ((statusBits & 0x08) != 0) {
    text("CAN transmission error", 20, nextStatusLineStart);
    nextStatusLineStart += 20;
  }
  if ((statusBits & 0x10) != 0) {
    text("CAN reception error", 20, nextStatusLineStart);
    nextStatusLineStart += 20;
  }
  
  // Draw the reset bits
  int vertical = 90;
  if ((reset & 0x01) != 0) {
    text("Starting up...", 500, vertical);
    vertical += 20;
  }
  if ((reset & 0x02) != 0) {
    text("HIL mode changed", 500, vertical);
    vertical += 20;
  }
  if ((reset & 0x04) != 0) {
    text("HIL disconnected", 500, vertical);
    vertical += 20;
  }
  if ((reset & 0x08) != 0) {
    text("GPS unavailable", 500, vertical);
    vertical += 20;
  }
  if ((reset & 0x10) != 0) {
    text("Resetting after switching track", 500, vertical);
    vertical += 20;
  }
  if ((reset & 0x20) != 0) {
    text("Undergoing rudder calibration", 500, vertical);
    vertical += 20;
  }
  if ((reset & 0x40) != 0) {
    text("Rudder uncalibrated", 500, vertical);
    vertical += 20;
  }
  if ((reset & 0x80) != 0) {
    text("E-stop activated", 500, vertical);
  }
  
  // Draw the battery voltage and power draw in amps
  text(String.format("%2.1f V", batteryVoltage), 50, 440);
  text(String.format("%2.1f A", batteryAmperage), 50, 460);
  
  // Draw the load %
  text(String.format("%3d%%",load), 570, 310);
  
  // Reset fill color to white
  fill(255);
  
  // Add rudder sensor information
  text(rudderPot, 50, 505);
  text("Pt: " + (rudderPortLimit?"true":"false"), 50, 520);
  text("Sb: " + (rudderSbLimit?"true":"false"), 50, 535);
  text("LoLim: " + lowRudderCalLimit, 50, 550);
  text("HiLim: " + highRudderCalLimit, 50, 565);
  
  // Add GPS sensor information
  text(String.format("%3.8f", gpsLatitude), 400, 405);
  text(String.format("%3.8f", gpsLongitude), 400, 420);
  text(gpsAltitude, 400, 435);
  text(String.format("%02d/%02d/%04d", gpsDay, gpsMonth, 2000+gpsYear), 400, 450);
  text(String.format("%02d:%02d:%02d", gpsHour, gpsMinute, gpsSecond), 400, 465);
  text(gpsCourse, 400, 480);
  text(gpsSpeed, 400, 495);
  text(gpsHdop, 400, 510);
  text(gpsFix, 400, 525);
  text(gpsSatellites, 400, 540);
  
  // Add Revolution GS sensor information
  text(String.format("Psi: %3.2f", revoHeading), 30, 50);
  text(String.format("Status: %c", revoMagStatus), 30, 65);
  text(String.format("Theta: %3.2f", revoPitch), 130, 50);
  text(String.format("Status: %c", revoPitchStatus), 130, 65);
  text(String.format("Phi: %3.2f", revoRoll), 230, 50);
  text(String.format("Status: %c", revoRollStatus), 230, 65);
  text(String.format("Dip: %3.2f", revoDip), 330, 50);
  text(String.format("MM: %d", revoMagneticMagnitude), 330, 65);
  
  // Add WSO100 air/wind data
  text(String.format("Wind speed: %2.2f m/s", windSpeed), 780, 50);
  text(String.format("Wind dir: %2.2f \u00B0", windDirection * 180 / Math.PI), 780, 80);
  text(String.format("Air temp: %2.2f \u00B0C", airTemp), 780, 110);
  text(String.format("Air pressure: %2.2f kPa", airPressure), 780, 140);
  text(String.format("Air humidity: %2.2f %%", airHumidity), 780, 170);
  text(String.format("Water speed: %2.2f m/s", waterSpeed), 780, 200);
  text(String.format("Water temp: %2.2f \u00B0C", waterTemp), 780, 230);
  text(String.format("Water depth: %2.2f m", waterDepth), 780, 260);
  
  // Draw the velocity vector values
  text(String.format("%2.1f m/s", velocity.mag()), 200, 285);
  text(String.format("%2.1f knots", velocity.mag()*1.944), 200, 300);
  text(String.format("%2.1f mph", velocity.mag()*2.237), 200, 315);
  
  // Display the commanded throttle values. Since the throttle is a commanded current
  // and each step is 1/1024 of the maximum current I map the value into Amperes before
  // display.
  text(String.format("%3.1f A", float(throttleCommand)/1024*15), 200, 360);
  
  // Draw the local position values
  text(localPosition.x, 200, 405);
  text(localPosition.y, 200, 420);
  text(localPosition.z, 200, 435);
  
  // Draw the global position values
  text(globalPosition.x, 200, 465);
  text(globalPosition.y, 200, 480);
  text(globalPosition.z, 200, 495);
  
  // Draw the current and next waypoints
  text(waypoint0.x, 50, 305);
  text(waypoint0.y, 50, 320);
  text(waypoint0.z, 50, 335);
  text(waypoint1.x, 50, 365);
  text(waypoint1.y, 50, 380);
  text(waypoint1.z, 50, 395);
  
  // Display the boat heading converted to degrees
  text(String.format("%2.1f\u00B0", heading*57.2958), 400, 305);
  
  // Draw the commanded rudder angle in degrees
  text(String.format("%3.1f\u00B0", rudderAngleCommand * 180 / 3.14159), 400, 360);
  
  // Draw the boat and rudder
  fill(155,155,0);
  pushMatrix();
  translate(420,200);
  rotate(heading);
  pushMatrix();
  translate(0, 49);
  rotate(rudderAngle);
  rect(-2.5, 0, 5, 18);
  popMatrix();
  rect(-20, -50, 40, 100);
  popMatrix();
  
  // Draw the velocity vector on top of the boat
  if (velocity.mag() > 0) {
    // Prepare the vector from a NED to an ESD vector draw call
    drawVector(new PVector(velocity.y, -velocity.x, velocity.z), 420, 200, 40, 2, new PVector(34, 139, 34));
  }
  
  // Draw the L2 vector on top of the boat also
  if (L2.mag() > 0) {
    // Prepare the vector from a NED to an ESD vector draw call
    drawVector(new PVector(L2.y, -L2.x, L2.z), 420, 200, 10, 2, new PVector(34, 34, 139));
  }
  
  // Reset fill color to white
  fill(255);
  stroke(255);
  
  if (recording) {
    recordTimerLabel.setValue(recordTimer.toString());
    recordedMessagesLabel.setValue(String.format("Messages: %d", recordedMessages));
  }
}

/**
 * Implements the built-in callback keyReleased() that is provided by Processing
 * for handling keyboard events. This function is used to check for the spacebar
 * being pressed. If it was we toggle the recording state.
 */
void keyReleased() {
  if (key == ' ') {
      if (recording) {
        recording = false;
        stopRecordingAndSave();
      } else {
        recording = true;
        startRecording();
      }
      recordToggle.setState(recording);
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
      if (theEvent.value() == 0) {
        recording = false;
        stopRecordingAndSave();
      } else if (theEvent.value() == 1) {
        recording = true;
        startRecording();
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
  recordTimer.reset();
    
  try {
    Calendar cal = Calendar.getInstance();
    SimpleDateFormat sdf = new SimpleDateFormat("yyyyMMdd-hhmmss");
    csvOutput = createWriter(sketchPath(sdf.format(cal.getTime())+".csv"));
    csvOutput.print("% Units: m, m, m, m, rad east from north, m, m, m, m, m, m, m, m, none, none, none, rad, rad, m, none, none, none, none, none, none, rad, m/s, none, none, none, none, none, rads, RPM, none, none, rad, rpm, V, A, none, none, rad, none, rad, none, rad, none, none, none, m/s, rad east from north, deg C, kPa, %, m/s, deg C, m\n");
    csvOutput.print("L2.north, L2.east, globalPosition.lat, globalPosition.lon, heading, localPosition.north, localPosition.east, velocity.north, velocity.east, waypoint0.north, waypoint0.east, waypoint1.north, waypoint1.east, rudderPot, rudderPortLimit, rudderSbLimit, gpsLatitude, gpsLongitude, gpsAltitude, gpsYear, gpsMonth, gpsDay, gpsHour, gpsMinute, gpsSecond, gpsCourse, gpsSpeed, gpsHdop, gpsFix, gpsSatellites, reset, load, rudderAngle, propRpm, statusBits, ordering, rudderAngleCommand, throttleCommand, batteryVoltage, batteryAmperage, lowRudderCalLimit, highRudderCalLimit, revoHeading, revoMagStatus, revoPitch, revoPitchStatus, revoRoll, revoRollStatus, revoDip, revoMagneticMagnitude, windSpeed, windDirection, airTemp, airPressure, airHumidity, waterSpeed, waterTemp, waterDepth\n");
  }
  catch (Exception e){
    println("Failed to write output to a .csv file");
    recording = false;
  }
  
  // Reset the messages counter
  recordedMessages = new Long(0);
}

public void stopRecordingAndSave() {
  csvOutput.close();
}

