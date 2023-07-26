/*
    HMC5883L Triple Axis Digital Compass.
    Processing for HMC5883L_calibrate.ino
    Processing for HMC5883L_calibrate_MPU6050.ino
    Read more: http://www.jarzebski.pl/arduino/czujniki-i-sensory/3-osiowy-magnetometr-hmc5883l.html
    GIT: https://github.com/jarzebski/Arduino-HMC5883L
    Web: http://www.jarzebski.pl
    (c) 2014 by Korneliusz Jarzebski
*/

import processing.serial.*;

Serial myPort;

// Data samples
float x = 0;
float y = 0;
float z = 0;

float minX = 0;
float maxX = 0;
float minY = 0;
float maxY = 0;
float minZ = 0;
float maxZ = 0;
float offX = 0;
float offY = 0;
float offZ = 0;
float scaleX = 0;
float scaleY = 0;
float scaleZ = 0;

int ws = 900;

void setup ()
{
  size(900, 900, P2D);
  background(0);
  stroke(255);
  
  strokeWeight(2);

  line(ws, 0, ws/2, ws);
  line(0, ws/2, ws, 40);

  strokeWeight(3);
  textSize(12);
  myPort = new Serial(this, Serial.list()[2], 9600);
  myPort.bufferUntil(10);
}

void draw() 
{
  stroke(255);
  strokeWeight(0);
  fill(0);  // Set fill to white
  rect(0, 0, 240, 65); 

  strokeWeight(2);
  fill(255);  // Set fill to white
  text(minX+" "+maxX+" = "+offX, 10, 20);
  text(minY+" "+maxY+" = "+offY, 10, 35);
  text(minZ+" "+maxZ+" = "+offZ, 10, 50);
  text(scaleX+" "+scaleY+" "+scaleZ, 10, 65);
  point((x*0.05)+ws/2, (y*0.05)+ws/2);
  
  //strokeWeight(2);
  stroke(120);
  point(ws/2, (z*0.05)+ws/2);
}

void serialEvent (Serial myPort)
{
  String inString = myPort.readStringUntil(10);

  if (inString != null)
  {
    inString = trim(inString);
    String[] list = split(inString, ':');
    String testString = trim(list[0]);

    if (list.length != 15) return;

    x = (float(list[0]));
    y = (float(list[1]));
    z = (float(list[2]));
    
    minX = (float(list[3]));
    maxX = (float(list[4]));
    minY = (float(list[5]));
    maxY = (float(list[6]));   
    minZ = (float(list[7]));
    maxZ = (float(list[8]));   
    offX = (float(list[9]));
    offY = (float(list[10]));  
    offZ = (float(list[11]));  
    scaleX = (float(list[12]));
    scaleY = (float(list[13]));
    scaleZ = (float(list[14]));
  }
}
