import controlP5.*;
import processing.serial.*;


ControlP5 controlP5;

DropdownList serialPortsList;

Serial myPort;

void setup() {
  size(800, 600);
  frameRate(30);
  controlP5 = new ControlP5(this);
  serialPortsList = controlP5.addDropdownList("myList-p1",100,100,100,120);
  customizeSerialPortsList(serialPortsList);
}

void draw() {
  if (myPort != null && myPort.available() > 0) {  // If data is available,
    print(myPort.readChar());         // read it and store it in val
  }
  
}

void controlEvent(ControlEvent theEvent) {
  // PulldownMenu is of type ControlGroup.
  // A controlEvent will be triggered from within the ControlGroup.
  // therefore you need to check the originator of the Event with
  // if (theEvent.isGroup())
  // to avoid an error message from controlP5.

  if (theEvent.isGroup()) {
    // check if the Event was triggered from a ControlGroup
    myPort = new Serial(this, theEvent.group().stringValue(), 57600);
  }
}

void customizeSerialPortsList(DropdownList ddl) {
  ddl.setBackgroundColor(color(190));
  ddl.setItemHeight(20);
  ddl.setBarHeight(15);
  
  ddl.captionLabel().set("pulldown");
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
