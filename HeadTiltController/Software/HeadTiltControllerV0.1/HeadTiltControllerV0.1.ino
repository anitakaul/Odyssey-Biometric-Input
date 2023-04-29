/*
Pot 0 - X (65-158) (right-left)
Pot 1 - Y (90-178) (bottom-top)

*/

#include <MCP4251.h>
#include "MCP4251Enable.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


#define chipSelectPin 37
#define pot0ResistanceRmax 98600 // These resistance values may vary
#define pot0ResistanceRmin 113.5
#define pot1ResistanceRmax 98600
#define pot2ResistanceRmin 130
MCP4251 digitalPot(chipSelectPin, pot0ResistanceRmax, pot0ResistanceRmin, pot1ResistanceRmax, pot2ResistanceRmin);

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

double headLeftValue=-45.0, headRightValue=45.0, headUpValue=45.0, headDownValue=-45.0, playerLeftValue=158.0, playerRightValue=65.0, playerUpValue=178.0, playerDownValue=90.0;

//prototypes
// double getIMU_X(sensors_event_t* event);
// double getIMU_Y(sensors_event_t* event);


void setup()
{

  // Serial.begin(9600);
  digitalPot.begin();

  init_player_as_writeable(&p1_spot);

  Serial.begin(115200);

  setupIMU();

  digitalPot.DigitalPotSetWiperPosition(0, 110);
  digitalPot.DigitalPotSetWiperPosition(1, 110);

  calibrate();
  
}


void loop()
{
  // for(int i=0; i<255; i++){
  //   pos = i;
  //   digitalPot.DigitalPotSetWiperPosition(0, 70);
  //   digitalPot.DigitalPotSetWiperPosition(1, 150);
  //   delay(100);
  //   Serial.print("Looping: ");
  //   Serial.println(pos);
  // }
  // delay(1000);


  sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

  
  int xWrite = map(getIMU_X(&orientationData), headLeftValue, headRightValue, playerLeftValue, playerRightValue);
  int yWrite = map(getIMU_Y(&orientationData), headDownValue, headUpValue, playerDownValue, playerUpValue);

  // digitalPot.DigitalPotSetWiperPosition(0, 65);
  // digitalPot.DigitalPotSetWiperPosition(1, 90);
  // delay(1000);

  // digitalPot.DigitalPotSetWiperPosition(0, 158);
  // digitalPot.DigitalPotSetWiperPosition(1, 178);
  // delay(1000);

  // digitalPot.DigitalPotSetWiperPosition(0, 65);
  // digitalPot.DigitalPotSetWiperPosition(1, 178);
  // delay(1000);

  // digitalPot.DigitalPotSetWiperPosition(0, 158);
  // digitalPot.DigitalPotSetWiperPosition(1, 90);
  // delay(5000);

  digitalPot.DigitalPotSetWiperPosition(0, xWrite);
  digitalPot.DigitalPotSetWiperPosition(1, yWrite);
  Serial.println("Reached Bottom");


}

//===================================Calibration Methods===================================

void calibrate(){
  //setup varibales for control loops and input pins
  bool isRightCalibrated = false, isLeftCalibrated = false, isUpCalibrated = false, isDownCalibrated = false;
  int upLeftBtn = A5, downRightBtn = A6, confirmBtn = A7;
  int position;

  //setup inputs
  pinMode(upLeftBtn, INPUT);
  pinMode(downRightBtn, INPUT);
  pinMode(confirmBtn, INPUT);


  //set the player to defaut position
  digitalPot.DigitalPotSetWiperPosition(0, 110);
  digitalPot.DigitalPotSetWiperPosition(1, 110);
  position=127;
  while(!isRightCalibrated){
     Serial.println("Inside cal loop");
    if(digitalRead(upLeftBtn)){
      delay(50);
      position++;
    }else if(digitalRead(downRightBtn)){
      delay(50);
      position--;
    }else if(digitalRead(confirmBtn)){
      sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
      bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

      isRightCalibrated=true;
      playerRightValue=position;
      headRightValue=getIMU_X(&orientationData);
      delay(200);
    }else{
      digitalPot.DigitalPotSetWiperPosition(0, position);      
    }
  }

  //set the player to defaut position
  digitalPot.DigitalPotSetWiperPosition(0, 110);
  digitalPot.DigitalPotSetWiperPosition(1, 110);
  position=127;
  while(!isLeftCalibrated){
     Serial.println("Inside cal loop");
    if(digitalRead(upLeftBtn)){
      delay(50);
      position++;
    }else if(digitalRead(downRightBtn)){
      delay(50);
      position--;
    }else if(digitalRead(confirmBtn)){
      sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
      bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

      isLeftCalibrated=true;
      playerLeftValue=position;
      headLeftValue=getIMU_X(&orientationData);
      delay(200);
    }else{
      digitalPot.DigitalPotSetWiperPosition(0, position);      
    }
  }

  //set the player to defaut position
  digitalPot.DigitalPotSetWiperPosition(0, 110);
  digitalPot.DigitalPotSetWiperPosition(1, 110);
  position=127;
  while(!isUpCalibrated){
     Serial.println("Inside cal loop");
    if(digitalRead(upLeftBtn)){
      delay(50);
      position++;
    }else if(digitalRead(downRightBtn)){
      delay(50);
      position--;
    }else if(digitalRead(confirmBtn)){
      sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
      bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

      isUpCalibrated=true;
      playerUpValue=position;
      headUpValue=getIMU_Y(&orientationData);
      delay(200);
    }else{
      digitalPot.DigitalPotSetWiperPosition(1, position);      
    }
  }

  //set the player to defaut position
  digitalPot.DigitalPotSetWiperPosition(0, 110);
  digitalPot.DigitalPotSetWiperPosition(1, 110);
  position=127;
  while(!isDownCalibrated){
     Serial.println("Inside cal loop");
    if(digitalRead(upLeftBtn)){
      delay(50);
      position++;
    }else if(digitalRead(downRightBtn)){
      delay(50);
      position--;
    }else if(digitalRead(confirmBtn)){
      sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
      bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

      isDownCalibrated=true;
      playerDownValue=position;
      headDownValue=getIMU_Y(&orientationData);
      delay(200);
    }else{
      digitalPot.DigitalPotSetWiperPosition(1, position);      
    }
  }

}

//===============================End Calibration Methods===================================



//=======================================IMU Methods=======================================
void setupIMU()
{
  while (!Serial) delay(10);  // wait for serial port to open!

  // Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);  
}

// void getIMUData()
// {
//   sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
//   bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
//   bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
//   bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
//   bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
//   bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
//   bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);  
// }

double getIMU_X(sensors_event_t* event)
{
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orient:");
    x = event->orientation.y;
  }
  return x;
}

double getIMU_Y(sensors_event_t* event)
{
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orient:");
    y = event->orientation.z;
  }
  return y;
}
//===================================End IMU Methods=======================================

