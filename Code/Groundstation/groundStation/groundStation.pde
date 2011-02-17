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
  
  myPort = new Serial(this, "COM8", 57600);
}

void draw() {
  if ( myPort.available() > 0) {  // If data is available,
    print(myPort.readChar());         // read it and store it in val
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
