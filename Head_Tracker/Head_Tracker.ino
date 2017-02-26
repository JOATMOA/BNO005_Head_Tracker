#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055();

#include <Servo.h>

Servo panServo;
int panServoPin=9;
int panServoMax=170;
int panServoMin=10;
int panServoPosition=90;//stores the current panServoPosition


Servo tiltServo;
int tiltServoPin=10;
int tiltServoMax=140;
int tiltServoMin=40;
int tiltServoPosition=90;

unsigned long previousMillis = 0;        // will store last time LED was updated
unsigned long currentMillis =0;
unsigned long displayInterval=1000;

int statusLEDPin=12;
int statusLEDToggle=0;

int enableHeadTracking=0;

//float trueAngleX=0.0f;
unsigned int trueBradX=0;
unsigned int calculatedBradX=0;
unsigned int offsetBradX=32768;
unsigned int bradXMin=21845;
unsigned int bradXMax=43690;


unsigned int trueBradZ=0;
unsigned int calculatedBradZ=0;
unsigned int offsetBradZ=32768;
unsigned int bradZMin=23665;
unsigned int bradZMax=41869;



int zeroButtonPin=2;


void setup(void)
{
  
  pinMode(statusLEDPin,OUTPUT);
  pinMode(zeroButtonPin,INPUT_PULLUP);
  
  Serial.begin(115200);
  Serial.println("Head Tracker v0.1"); Serial.println("");

  //Initialise the sensor 
  if(!bno.begin())
  {
    //There was a problem detecting the BNO055 ... check your connections
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  Serial.println("BNO055 Ready");

  delay(3000);

//  for (int lop=-180; lop<360+180; lop++)
//  {
//    unsigned int theBrad=angle2BRAD(lop);
//    Serial.print("Original Angle="); Serial.print(lop);
//    Serial.print(" theBrad="); Serial.print(theBrad);
//    float theAngle=BRAD2Angle(theBrad);
//    Serial.print(" theAngle="); Serial.print(theAngle);
//    Serial.println("");
//    
//  }


  //for (;;)
  {
    digitalWrite(statusLEDPin,HIGH);
    delay(500);
    digitalWrite(statusLEDPin,LOW);
    delay(500);
  }


  panServo.attach(panServoPin);
  tiltServo.attach(tiltServoPin);

  for (int servoPanTest=panServoMin; servoPanTest<panServoMax; servoPanTest++)
  {
    panServo.write(servoPanTest);
    delay(10);
    if (statusLEDToggle==0) statusLEDToggle=1;
    else statusLEDToggle=0;
    digitalWrite(statusLEDPin,statusLEDToggle);
    
  }
  for (int servoPanTest=panServoMax; servoPanTest>90; servoPanTest--)
  {
    panServo.write(servoPanTest);
    delay(10);
  }

  Serial.println("Pan Servo Ready.");


  for (int servoTiltTest=tiltServoMin; servoTiltTest<tiltServoMax; servoTiltTest++)
  {
    tiltServo.write(servoTiltTest);
    delay(10);
  }
  for (int servoTiltTest=tiltServoMax; servoTiltTest>90; servoTiltTest--)
  {
    tiltServo.write(servoTiltTest);
    delay(10);
  }

  Serial.println("Tilt Servo Ready.");



  //Display the current temperature
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
}

void loop(void)
{
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);


  imu::Quaternion quat = bno.getQuat();

  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);


  currentMillis = millis();
  if (currentMillis - previousMillis >= displayInterval) 
  {
    previousMillis = currentMillis;
 

    /* Display the floating point data */
    Serial.print("Data,");
    Serial.print("Euler");
    Serial.print(",X,");
    Serial.print(euler.x());
    Serial.print(",Y,");
    Serial.print(euler.y());
    Serial.print(",Z,");
    Serial.print(euler.z());
  
//    // Quaternion data
//    Serial.print(",Quaternion,qW,");
//    Serial.print(quat.w(), 4);
//    Serial.print(",qX,");
//    Serial.print(quat.y(), 4);
//    Serial.print(",qY,");
//    Serial.print(quat.x(), 4);
//    Serial.print(",qZ,");
//    Serial.print(quat.z(), 4);
//  
    /* Display calibration status for each sensor. */
    Serial.print(",CALIBRATION,Sys,");
    Serial.print(system, DEC);
    Serial.print(",Gyro,");
    Serial.print(gyro, DEC);
    Serial.print(",Accel,");
    Serial.print(accel, DEC);
    Serial.print(",Mag,");
    Serial.print(mag, DEC);
  
  
    Serial.print(",time,");
    Serial.println(millis());
    if (mag==3) 
    {
      digitalWrite(statusLEDPin,HIGH);
      enableHeadTracking=1;

    }
    else 
    {
      digitalWrite(statusLEDPin,LOW);
      enableHeadTracking=0;
    }
    Serial.print("trueBradX="); Serial.print(trueBradX,DEC);
    Serial.print(" calculatedBradX="); Serial.print(calculatedBradX,DEC);
    Serial.print(" panServoPosition="); Serial.print(panServoPosition);
    Serial.println("");
    
    Serial.print("trueBradZ="); Serial.print(trueBradZ,DEC);
    Serial.print(" calculatedBradZ="); Serial.print(calculatedBradZ,DEC);
    Serial.print(" tiltServoPosition="); Serial.print(tiltServoPosition);
    Serial.println("");
    

  }//if displayinterval

  if (digitalRead(zeroButtonPin)==LOW)
  {
    Serial.println("Recalibrating Zero");
    offsetBradX=32768-trueBradX;
    offsetBradZ=32768-trueBradZ;
    delay(1000);
  }


  if (enableHeadTracking==1) //only when mag clibration == 3
  {

      trueBradX=angle2BRAD(euler.x());
      calculatedBradX=(trueBradX+65535+offsetBradX)& 0xFFFF;;//angle2BRAD(calculatedAngleX);
      if (calculatedBradX>bradXMax) calculatedBradX=bradXMax;
      if (calculatedBradX<bradXMin) calculatedBradX=bradXMin;
      panServoPosition=map(calculatedBradX,bradXMin,bradXMax,panServoMax,panServoMin);
      panServo.write(panServoPosition);

      trueBradZ=angle2BRAD(euler.z());
      calculatedBradZ=(trueBradZ+65535+offsetBradZ)& 0xFFFF;;//angle2BRAD(calculatedAngleX);
      if (calculatedBradZ>bradZMax) calculatedBradZ=bradZMax;
      if (calculatedBradZ<bradZMin) calculatedBradZ=bradZMin;
      tiltServoPosition=map(calculatedBradZ,bradZMin,bradZMax,tiltServoMax,tiltServoMin);
      tiltServo.write(tiltServoPosition);



//      printVar("CalculatedBradX",calculatedBradX);
//      printVar("bradXMin",bradXMin);
//      printVar("bradXMax",bradXMax);
//      printVar("panServoMin",panServoMin);
//      printVar("panServoMax",panServoMax);
//      
//      delay(1000);
    
  }

  delay(BNO055_SAMPLERATE_DELAY_MS);
}


float BRAD2Angle(int theBRAD)
{
  float retV=0.0f;
  
  retV=(360.0f/65535.0f)*theBRAD;
  //retV=retV % 360.0f;
  
  return retV;
}


unsigned int angle2BRAD(float theAngle)
{
  unsigned int retV=0;
  
  retV=(unsigned int)(theAngle/(360.0f/65535.0f));
  //retV=retV & 0xFFFF;
  
  return retV;
}
void printVar(String printVar,unsigned int printVal)
{
  Serial.print(printVar);
  Serial.print("=");
  Serial.println(printVal,DEC);
}
