/*
Pot 0 - X (65-158) (right-left)
Pot 1 - Y (90-178) (bottom-top)

*/

#include <MCP4251.h>
// #include "MCP4251Enable.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


#define chipSelectPin 7
#define pot0ResistanceRmax 98600 // These resistance values may vary
#define pot0ResistanceRmin 113.5
#define pot1ResistanceRmax 98600
#define pot2ResistanceRmin 130
MCP4251 digitalPot(chipSelectPin, pot0ResistanceRmax, pot0ResistanceRmin, pot1ResistanceRmax, pot2ResistanceRmin);



double leftValue=-45.0, rightValue=45.0, upValue=45.0, downValue=-45.0, playerLeftValue=158.0, playerRightValue=65.0, playerUpValue=178.0, playerDownValue=90.0;
int muscleOffset = 0, muscleScale = 1, muscleValue=127;

//prototypes
// double getIMU_X(sensors_event_t* event);
// double getIMU_Y(sensors_event_t* event);


void setup()
{

  // Serial.begin(9600);
  digitalPot.begin();

  // init_player_as_writeable(&p1_spot);

  Serial.begin(115200);

  digitalPot.DigitalPotSetWiperPosition(0, 110);
  digitalPot.DigitalPotSetWiperPosition(1, 110);

  calibrate();
  
}


void loop()
{

  muscleScale = map(analogRead(A6), 0, 1023, 0, 5);
  muscleOffset = map(analogRead(A7), 0, 1023, 0, 255);
  muscleValue=getMuscle();

  
  int xWrite = map((muscleValue*muscleScale)+muscleOffset, 0, 1023, 0, 255);
  int yWrite = map(0, downValue, upValue, playerDownValue, playerUpValue);

  xWrite = muscleOffset + xWrite*muscleScale;
  Serial.println(xWrite);
  yWrite=110;

  //TODO: Add negative scale and offset

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


}

//===================================Calibration Methods===================================

void calibrate(){
  
}

//===============================End Calibration Methods===================================



//==================================Heart Rate Methods=====================================
void setupHeartRate(){
  //do setup actionhere  
  
}
int getHeartRate(){
  //get heart rate here
  return 0;
}
//==============================End Heart Rate Methods=====================================


//====================================Muscle Methods=======================================
void setupMuscle(){
  //do setup actionhere  
  muscleScale = analogRead(A6);
  muscleOffset = analogRead(A7);
}
int getMuscle(){
  //get heart rate here
    // read the input on analog pin 0:
  int sensorValue = analogRead(A0);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  float voltage = sensorValue * (5.0 / 1023.0);
  return sensorValue;  
}
//================================End Muscle Methods=======================================
