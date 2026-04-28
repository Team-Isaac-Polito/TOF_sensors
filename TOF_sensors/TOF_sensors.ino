
/*
  This is the code for the RP2040-Zero used as master to TOF VL53L1X sensors by ST Microelectronics via I2C0.
  The zero is then connected to a Raspberry Pico via I2C1 as a slave sending the data collected by the TOFs. 
  
  The idea is to use multiple sensors to create a depth map of the environment for the robot autonomous movement.
  In order to achieve this goal, it is essential to:
  -> Make fast/simultaneous meausrements through TOFs ID management
  -> Divide the single photon avalanche diode (SPAD) of the TOF in multiple Regions of Interest (ROI) whose centers can be programmed.  

  For the definition of the center of ROI we find the SPAD configuration on both the pololu library and the ST user manual.
  Having defined a ROI size of 4, the center falls in between the 4 SPADs. The manual tells to choose as a center the upper-right position!

   Table of SPAD locations. Each SPAD has a number which is not obvious.
          * Pin 1 *                                                          -> 128 nearest to Pin 1 of the breakout board
          * 128,136,144,152,160,168,176,184, 192,200,208,216,224,232,240,248
          * 129,137,145,153,161,169,177,185, 193,201,209,217,225,233,241,249
          * 130,138,146,154,162,170,178,186, 194,202,210,218,226,234,242,250
          * 131,139,147,155,163,171,179,187, 195,203,211,219,227,235,243,251
          * 132,140,148,156,164,172,180,188, 196,204,212,220,228,236,244,252
          * 133,141,149,157,165,173,181,189, 197,205,213,221,229,237,245,253
          * 134,142,150,158,166,174,182,190, 198,206,214,222,230,238,246,254
          * 135,143,151,159,167,175,183,191, 199,207,215,223,231,239,247,255
          * 127,119,111,103, 95, 87, 79, 71, 63, 55, 47, 39, 31, 23, 15, 7
          * 126,118,110,102, 94, 86, 78, 70, 62, 54, 46, 38, 30, 22, 14, 6
          * 125,117,109,101, 93, 85, 77, 69, 61, 53, 45, 37, 29, 21, 13, 5
          * 124,116,108,100, 92, 84, 76, 68, 60, 52, 44, 36, 28, 20, 12, 4
          * 123,115,107, 99, 91, 83, 75, 67, 59, 51, 43, 35, 27, 19, 11, 3
          * 122,114,106, 98, 90, 82, 74, 66, 58, 50, 42, 34, 26, 18, 10, 2
          * 121,113,105, 97, 89, 81, 73, 65, 57, 49, 41, 33, 25, 17, 9, 1
          * 120,112,104, 96, 88, 80, 72, 64, 56, 48, 40, 32, 24, 16, 8, 0
          
    For more info on SPAD and ROI centers and definitions, look up at the user manual UM2555
  
*/              

// Libraries which are used in the code: 
#include <Arduino.h>
#include <Wire.h>
#include "VL53L1X.h"
/* 
  Other Libraries That might be usefull in the future:

#include <SPI.h>
#include "mcp2515.h"
#include "CanWrapper.h"
#include "include/definitions.h"
#include "include/mod_config.h"
#include "include/communication.h"
#include "Adafruit_VL53L1X.h"
*/

// PINs for I2C1 communication with the Pico
#define SCL_PIN_PICO 3
#define SDA_PIN_PICO 2

// Comunications' clock
#define SENSOR_BAUDRATE 400000    // at least 400 kHz for comunication with VL53L1X
#define BUS_BAUDRATE 115200       // To increase more use pull-up resistors

// Definition of the ROI  -> minimum ROI size of 4 
#define ROI_SIZE 4
#define MATRIX_SIZE 4

// Definition of the dimension of the mean matrix used to calculate mean values and filtering data. Higher values mean more stability,
// but at the cost of speed of detection of a big change in the distances!
#define MEAN_DIMENSION 5

/*
  Matrix 4x4  -> useful to measure as much area of the SPAD as possible

int centers_matrix[MATRIX_SIZE][MATRIX_SIZE] = {
        {145, 177, 209, 241}, // Riga 0
        {149, 181, 213, 245},    // Riga 1
        {110, 78, 46, 14},    // Riga 2
        {106, 74, 42, 10}     // Riga 3
};
*/

// Matrice 4x1  ->  useful to measure just an array of distances on a line   ->   case used for the robot. 
int centers_matrix[MATRIX_SIZE] = {193, 197, 62, 58};

//  SENSOR COUNT change only this number when adding or removing physical sensors (will be recalled for sensorCount later on).
//  Allowed range: from 1 to MAX_AVAILABLE_SENSORS (should be 7: from 7, 8, 14, 15, 16, 17, 18).
#define NUM_SENSORS 3

// All GPIO pins available for XSHUT, in priority order.
// The first NUM_SENSORS entries are used; the rest are ignored.
// Order: 7, 8, 14, 15, 16, 17, 18 
// if the number of max sensor available on board needs to be changed, remember to change both MAX_SENSOR and xshutPinPool (with the index of said pins)
#define MAX_AVAILABLE_SENSORS 7
const uint8_t xshutPinPool[MAX_AVAILABLE_SENSORS] = { 7, 8, 14, 15, 16, 17, 18 };

#if NUM_SENSORS < 1
  #error "NUM_SENSORS must be at least 1."
#endif
#if NUM_SENSORS > MAX_AVAILABLE_SENSORS
  #error "NUM_SENSORS exceeds the number of available XSHUT pins in xshutPinPool."
#endif

// sensorCount mirrors NUM_SENSORS. All previously existing code that references sensorCount continues to work without any other modifications.
const uint8_t sensorCount = NUM_SENSORS;
int distance_matrix[sensorCount][MATRIX_SIZE];

// Number of init attempts per sensor before halting:
#define MAX_INIT_RETRIES 3    

// Mean matrix for filtering:
int mean_matrix[sensorCount][MATRIX_SIZE][MEAN_DIMENSION];

// Counters and averages used in the loop for filtering
int counter = 0;
int average = 0;

// Definition of bytes to send to Master:
const size_t BYTES_TO_SEND = sizeof(distance_matrix);

// Address of the Slave  (rp2040-zero):
const int SLAVE_ADDRESS = 0x42;

// construct of the sensor through Pololu VL53L1X library:
VL53L1X sensors[sensorCount];


// Request Event from Master  ->  Slave writes the required bytes with matrix of distances. 
void requestEvent() {
    Wire1.write((const byte*)distance_matrix, BYTES_TO_SEND);
}


void setup(){
  Serial.begin(115200);
  //while (!Serial);      // Waits for Serial Monitor - Used for debugging
  Wire.setSDA(4);         // GP4
  Wire.setSCL(5);         // GP5
  Wire.begin();           // I2C0 for TOF comunication  ->  MASTER
  Wire.setClock(SENSOR_BAUDRATE); 
  Serial.println("Master Bus Started");
  //pinMode(SDA_PIN_PICO, INPUT_PULLUP);      // Internal Pull up resistance
  //pinMode(SCL_PIN_PICO, INPUT_PULLUP);      // not working very well -> better use lower baudrate

  Wire1.setSDA(SDA_PIN_PICO);  
  Wire1.setSCL(SCL_PIN_PICO);  
  Wire1.begin(SLAVE_ADDRESS);       // I2C1 comunication wih Raspberry pi pico ->  SLAVE
  Wire1.setClock(BUS_BAUDRATE);
  Wire1.onRequest(requestEvent);    // Slave setted on request waiting for Master

  Serial.println("Slave Bus started");

  // Disable/reset all sensors by driving their XSHUT pins low. This way we are able to turn them on one by one and assign an ID.
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    pinMode(xshutPinPool[i], OUTPUT);
    digitalWrite(xshutPinPool[i], LOW);
  }

  // Enable, initialize, and start each sensor, one by one.
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    pinMode(xshutPinPool[i], INPUT);
    delay(10);
    sensors[i].setTimeout(500);

    // Retry init up to MAX_INIT_RETRIES times before halting.
    bool initialized = false;
    for (uint8_t attempt = 0; attempt < MAX_INIT_RETRIES; attempt++) {
      if (sensors[i].init()) {
        initialized = true;
        break;
      }
      Serial.print("[WARNING] Sensor ");
      Serial.print(i);
      Serial.print(" init failed, retrying (");
      Serial.print(attempt + 1);
      Serial.print("/");
      Serial.print(MAX_INIT_RETRIES);
      Serial.println(" attempts)...");
      delay(100);
    }

    if (!initialized) {
      Serial.print("[ERROR] Sensor ");
      Serial.print(i);
      Serial.print(" failed all retries on XSHUT pin ");
      Serial.println(xshutPinPool[i]);
      Serial.println("Halting. Check wiring and power for this sensor.");
      while (1);
    }

    // Each sensor is assigned with a univoque ID.
    // The default ID is 0x29 (= 41). We set IDs from 0x2A (=42): 
    sensors[i].setAddress(0x2A + i); 

    // Setting sensor in continuous acquisition mod with a time budget of 40 ms. The sensor will acquire coninuously data with a timeout of 40 ms.
    // At least 30ms should be granted as time budget to ensure correct and stable measurments!
    sensors[i].startContinuous(40);

    // Setting ROI size (minimum size of 4 cells):
    sensors[i].setROISize(ROI_SIZE,ROI_SIZE);

    // Confirmation Status Message:
    Serial.print("  Sensor ");
    Serial.print(i);
    Serial.print(" OK - address 0x");
    Serial.print(0x2A + i, HEX);
    Serial.print(", XSHUT pin ");
    Serial.println(xshutPinPool[i]);
  }
  Serial.println("All sensors initialized. Starting measurements.");
}


void loop(){
  // The idea is to minimize the amount of time spent for each meausurment of each ROI for each sensor.
  // After setting the ROI center a delay IS REQUIRED before the measurment.
  // -> the best idea is to set first the roi center of each sensor, place a delay and THEN measure the distances.
  //    ->  measure time almost independent from the number of sensors used!!!
  // Unfortunately the Pololu library does not support fully syncronized readings, however, the continuous modality 
  // makes the TOF constantly make measures, so we are able to read the latest data from all sensors almost simultaneously, with a delay
  // of few ms!  
  // In total we will be under 100ms for a whole measurment of all sensors!
  
  // In this filtered version a filtering mean matrix has been added. Each distance measured with the sensor is then filtered by 
  // calculating the mean with MEAN_DIMENSION-1 previous readings. 

  for (int r=0; r < MATRIX_SIZE; r++) {
    for (int i=0; i< sensorCount; i++) {
      int center_spad = centers_matrix[r];
      sensors[i].setROICenter(center_spad);
    }
    delay(20);
    for (int i=0; i< sensorCount; i++) {
      mean_matrix[i][r][counter] = sensors[i].read();
      //distance_matrix[i][r] = sensors[i].read();
      for (int k=0; k< MEAN_DIMENSION; k++){
        average = average + mean_matrix[i][r][k];
      }
      average = average/MEAN_DIMENSION;
      distance_matrix[i][r] = average;
      average = 0;
    }
    delay(2); 
  }
    
  if (counter < MEAN_DIMENSION-1){
    counter++;
  }
  else {
    counter = 0;
  }

  // Printing the matrix of measurments - Used for debugging
/*  
  for (int r = 0; r < sensorCount; r++) {
    Serial.print("Lettura sensore:");
    Serial.print(r);
    Serial.print('\t');
    for (int c = 0; c < MATRIX_SIZE; c++) {
      Serial.print(distance_matrix[r][c]);
      Serial.print('\t'); 
    }
    Serial.println();
  }
*/

}
