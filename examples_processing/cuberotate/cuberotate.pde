import processing.serial.*;
import java.awt.datatransfer.*;
import java.awt.Toolkit;
import processing.opengl.*;
import saito.objloader.*;
import g4p_controls.*;

float roll  = 0.0F;
float pitch = 0.0F;
float yaw   = 0.0F;
float temp  = 0.0F;
float alt   = 0.0F;

OBJModel model;

// Serial port state.
Serial       port;
final String serialConfigFile = "serialconfig.txt";
boolean      printSerial = false;

// UI controls.
GPanel    configPanel;
GDropList serialList;
GLabel    serialLabel;
GLabel    calLabel;
GCheckbox printSerialCheckbox;

void setup()
{
  size(640, 480, OPENGL);
  frameRate(30);
  model = new OBJModel(this);
  model.load("bunny.obj");
  model.scale(20);
  
  // Serial port setup.
  // Grab list of serial ports and choose one that was persisted earlier or default to the first port.
  int selectedPort = 0;
  String[] availablePorts = Serial.list();
  if (availablePorts == null) {
    println("ERROR: No serial ports available!");
    exit();
  }
  String[] serialConfig = loadStrings(serialConfigFile);
  if (serialConfig != null && serialConfig.length > 0) {
    String savedPort = serialConfig[0];
    // Check if saved port is in available ports.
    for (int i = 0; i < availablePorts.length; ++i) {
      if (availablePorts[i].equals(savedPort)) {
        selectedPort = i;
      } 
    }
  }
  // Build serial config UI.
  configPanel = new GPanel(this, 10, 10, width-20, 90, "Configuration (click to hide/show)");
  serialLabel = new GLabel(this,  0, 20, 80, 25, "Serial port:");
  configPanel.addControl(serialLabel);
  serialList = new GDropList(this, 90, 20, 200, 200, 6);
  serialList.setItems(availablePorts, selectedPort);
  configPanel.addControl(serialList);
  calLabel = new GLabel(this, 300, 20, 350, 25, "Calibration: Sys=? Gyro=? Accel=? Mag=?");
  configPanel.addControl(calLabel); 
  printSerialCheckbox = new GCheckbox(this, 5, 50, 200, 20, "Print serial data");
  printSerialCheckbox.setSelected(printSerial);
  configPanel.addControl(printSerialCheckbox);
  // Set serial port.
  setSerialPort(serialList.getSelectedText());
}
 
void draw()
{
  background(0,0,0);

  // Set a new co-ordinate space
  pushMatrix();

  // Simple 3 point lighting for dramatic effect.
  // Slightly red light in upper right, slightly blue light in upper left, and white light from behind.
  pointLight(255, 200, 200,  400, 400,  500);
  pointLight(200, 200, 255, -400, 400,  500);
  pointLight(255, 255, 255,    0,   0, -500);
  
  // Move bunny from 0,0 in upper left corner to roughly center of screen.
  translate(300, 380, 0);
  
  // Rotate shapes around the X/Y/Z axis (values in radians, 0..Pi*2)
  //rotateZ(radians(roll));
  //rotateX(radians(pitch)); // extrinsic rotation
  //rotateY(radians(yaw));
  float c1 = cos(radians(roll));
  float s1 = sin(radians(roll));
  float c2 = cos(radians(pitch)); // intrinsic rotation
  float s2 = sin(radians(pitch));
  float c3 = cos(radians(yaw));
  float s3 = sin(radians(yaw));
  applyMatrix( c2*c3, s1*s3+c1*c3*s2, c3*s1*s2-c1*s3, 0,
               -s2, c1*c2, c2*s1, 0,
               c2*s3, c1*s2*s3-c3*s1, c1*c3+s1*s2*s3, 0,
               0, 0, 0, 1);

  pushMatrix();
  noStroke();
  model.draw();
  popMatrix();
  popMatrix();
  //print("draw");
}

void serialEvent(Serial p) 
{
  String incoming = p.readString();
  if (printSerial) {
    println(incoming);
  }
  
  if ((incoming.length() > 8))
  {
    String[] list = split(incoming, " ");
    if ( (list.length > 0) && (list[0].equals("Orientation:")) ) 
    {
      roll  = float(list[3]); // Roll = Z
      pitch = float(list[2]); // Pitch = Y 
      yaw   = float(list[1]); // Yaw/Heading = X
    }
    if ( (list.length > 0) && (list[0].equals("Alt:")) ) 
    {
      alt  = float(list[1]);
    }
    if ( (list.length > 0) && (list[0].equals("Temp:")) ) 
    {
      temp  = float(list[1]);
    }
    if ( (list.length > 0) && (list[0].equals("Calibration:")) )
    {
      int sysCal   = int(list[1]);
      int gyroCal  = int(list[2]);
      int accelCal = int(list[3]);
      int magCal   = int(trim(list[4]));
      calLabel.setText("Calibration: Sys=" + sysCal + " Gyro=" + gyroCal + " Accel=" + accelCal + " Mag=" + magCal);
    }
  }
}

// Set serial port to desired value.
void setSerialPort(String portName) {
  // Close the port if it's currently open.
  if (port != null) {
    port.stop();
  }
  try {
    // Open port.
    port = new Serial(this, portName, 115200);
    port.bufferUntil('\n');
    // Persist port in configuration.
    saveStrings(serialConfigFile, new String[] { portName });
  }
  catch (RuntimeException ex) {
    // Swallow error if port can't be opened, keep port closed.
    port = null; 
  }
}

// UI event handlers

void handlePanelEvents(GPanel panel, GEvent event) {
  // Panel events, do nothing.
}

void handleDropListEvents(GDropList list, GEvent event) { 
  // Drop list events, check if new serial port is selected.
  if (list == serialList) {
    setSerialPort(serialList.getSelectedText()); 
  }
}

void handleToggleControlEvents(GToggleControl checkbox, GEvent event) { 
  // Checkbox toggle events, check if print events is toggled.
  if (checkbox == printSerialCheckbox) {
    printSerial = printSerialCheckbox.isSelected(); 
  }
}
