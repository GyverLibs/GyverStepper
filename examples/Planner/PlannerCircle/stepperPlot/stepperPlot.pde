String port = "COM6";   // имя порта

import processing.serial.*;
Serial myPort;

void setup() {
  size(500, 500);
  myPort = new Serial(this, port, 115200);
  fill(0);
  noStroke();
}

void draw() {  
  if (myPort.available() > 0) {
    String str = myPort.readStringUntil('\n').trim();
    if (str != null) {      
      String[] pos = split(str, ',');
      int x = int(pos[0]);
      int y = int(pos[1]);
      fill(10);
      circle(x+10, y+10, 4);
      fill(200, 1);
      rect(0, 0, width, height);
    }
  }
}
