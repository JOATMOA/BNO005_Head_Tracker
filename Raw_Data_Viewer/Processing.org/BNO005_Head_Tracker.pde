// Need G4P library
import g4p_controls.*;
import processing.serial.*;

String[] rawPortNames = Serial.list();
String[] portNames;
Serial serialPort;
boolean serialPortOpen=false;
int lf = 10;      // ASCII linefeed


float trueAngleX=0.0f;
float calculatedAngleX=0.0f;
float offsetAngleX=0.0;

int trueBradX=0;
int calculatedBradX=0;
int offsetBradX=0;

float trueAngleZ=0.0f;
float calculatedAngleZ=0.0f;
float offsetAngleZ=0.0;

int trueBradZ=0;
int calculatedBradZ=0;
int offsetBradZ=0;



public void setup(){
  size(800, 400, JAVA2D);
  
  //enumerate serial ports
 portNames=new String[rawPortNames.length];
  int portNamesPointer=0;
  for (int i=0; i<rawPortNames.length; i++)
  {
    if( rawPortNames[i].indexOf("ACM") != -1 )
    {
      portNames[portNamesPointer]=rawPortNames[i];
      portNamesPointer++;
    }
  }
  for (int i=0; i<rawPortNames.length; i++)
  {
    if( rawPortNames[i].indexOf("USB") != -1 )
    {
      portNames[portNamesPointer]=rawPortNames[i];
      portNamesPointer++;
    }
  }
  for (int i=0; i<rawPortNames.length; i++)
  {
    if( rawPortNames[i].indexOf("ACM") == -1 && rawPortNames[i].indexOf("USB") == -1)
    {
      portNames[portNamesPointer]=rawPortNames[i];
      portNamesPointer++;
    }
  }
  //end of sort
  
  
  
  createGUI();
  customGUI();
  // Place your setup code here
  
}

public void draw(){
  background(0,0,0);
  
  if (serialPortOpen==true)
  {
    if (serialPort.available() > 0) 
    {
      String myString = serialPort.readStringUntil(lf);
      if (myString != null) 
      {
        //textarea1.appendText(myString+"\r\n");
        myString=myString.trim();
        //println(myString);
        if (myString.indexOf("Data,")==0)
        {
          try
          {
            String[] q=splitTokens(myString,",");
            //printArray(q);
            float eulerX=parseFloat(q[3]);
            float eulerY=parseFloat(q[5]);
            float eulerZ=parseFloat(q[7]);
            
            EulerX.setText(str(eulerX));
            EulerY.setText(str(eulerY));
            EulerZ.setText(str(eulerZ));
            
            int calSys=parseInt(q[19]);
            int calGyro=parseInt(q[21]);
            int calAccel=parseInt(q[23]);
            int calMag=parseInt(q[25]);
            
            calSysTxt.setText(str(calSys));
            calGyroTxt.setText(str(calGyro));
            calAccelTxt.setText(str(calAccel));
            calMagTxt.setText(str(calMag));
            
            ///X
            trueAngleX=eulerX;
            trueBradX=angle2BRAD(eulerX);
            
            calculatedAngleX=(eulerX+360+offsetAngleX)%360.0f;
            angDegXTxt.setText(nf(calculatedAngleX,4,2));
            offsetXTxt.setText(str(offsetAngleX));
            
            calculatedBradX=(trueBradX+65535+offsetBradX)& 0xFFFF;;//angle2BRAD(calculatedAngleX);
            angBRADXTxt.setText(str(calculatedBradX));
            

            ///Z
            trueAngleZ=eulerZ;
            trueBradZ=angle2BRAD(eulerZ);
            
            calculatedAngleZ=(eulerZ+360+offsetAngleZ)%360.0f;
            angDegZTxt.setText(nf(calculatedAngleZ,4,2));
            offsetZTxt.setText(str(offsetAngleZ));
            
            calculatedBradZ=(trueBradZ+65535+offsetBradZ)& 0xFFFF;;//angle2BRAD(calculatedAngleX);
            angBRADZTxt.setText(str(calculatedBradZ));
            
            
          }
          catch (Exception e) 
          {
          
          }
        }
        //EulerX.setText("x");
      }//not null
    }//serialPortAvailable>0
  }//serialPortOpen==true
  
}

// Use this method to add additional statements
// to customise the GUI controls
public void customGUI(){
 dropList1.setItems(portNames,0);
}


float BRAD2Angle(int theBRAD)
{
  float retV=0.0f;
  
  retV=(360.0f/65535.0f)*theBRAD;
  retV=retV % 360;
  
  return retV;
}


int angle2BRAD(float theAngle)
{
  int retV=0;
  
  retV=(int)(theAngle/(360.0f/65535.0f));
  retV=retV & 0xFFFF;
  
  return retV;
}