/*
  This is is the Pico low level code for communicating with the RP2040-zero via I2C.
  The data is then sent to Serial for graphical visualization via python code. 
*/

// Used Libraries
#include <Arduino.h>
#include <Wire.h>
#include "VL53L1X.h"

/* Possibly useful libraries 
#include <SPI.h>
#include "mcp2515.h"
#include "CanWrapper.h"
#include "include/definitions.h"
#include "include/mod_config.h"
#include "include/communication.h"
#include "Adafruit_VL53L1X.h"
*/

// Definition of constants for comunication and data dimension:
#define MATRIX_SIZE 4
#define SDA_PIN_ZERO 2
#define SCL_PIN_ZERO 3

// Number of sensors used:
const uint8_t sensorCount = 2;

int received_matrix[sensorCount][MATRIX_SIZE];

// Dimension of received matrix:
const size_t BYTES_FROM_ZERO = sizeof(received_matrix);

// Comunication constants:
const int ZERO_ADDRESS = 0x42;
const long BAUD_RATE = 100000;

void setup(){
  //while (!Serial) {}    // Just for debugging
  Serial.begin(115200);
 
  // Possible pull up internal resistors for higher clock comunication:
  //pinMode(SDA_PIN_ZERO, INPUT_PULLUP);
  //pinMode(SCL_PIN_ZERO, INPUT_PULLUP);

  // Initialization of I2C1 for comunication with RP2040-zero:
  Wire1.setSDA(2);  // GP4
  Wire1.setSCL(3);  // GP5
  Wire1.begin();    // initialized as Master
  Wire1.setClock(BAUD_RATE);

  Serial.println("Pico Master Initialized");
  delay(1000);
}

void loop(){
  // Requesting bytes from RP:
  Wire1.requestFrom(ZERO_ADDRESS, BYTES_FROM_ZERO);
  
  // Setting timeous and checking for available line of comunication:
  unsigned long timeout = millis();
  while (Wire1.available() < BYTES_FROM_ZERO && millis() - timeout < 100) {
    delay(1);
  }
  
  // Using temporary buffer to save data:
  if (Wire1.available()>= BYTES_FROM_ZERO) {
    int tof_matrix[sensorCount][MATRIX_SIZE];
    byte temp_buffer[BYTES_FROM_ZERO];

    for (size_t i=0; i<BYTES_FROM_ZERO; i++) {
      temp_buffer[i] = Wire1.read();
    }

    // Converting raw data on buffer in a matrix of measured distances:
    memcpy(received_matrix, temp_buffer, BYTES_FROM_ZERO);
  
    // Definition of an header for Python visualizer syncronization
    // When the high level code receives the header, knows whose sensor the data are!
    byte header[4] = {0xAA, 0x55, 0xAA, 0x55};
    Serial.write(header, 4);
    Serial.write((const byte*)received_matrix, sizeof(received_matrix));  // This is just used to visualize it on python like depth camera
    Serial.flush();

    

    
    // Used for debugging:

    for (size_t i = 0; i < sensorCount; i++) {
      for (size_t r = 0; r < MATRIX_SIZE; r++) {
        Serial.print(received_matrix[i][r]);
        Serial.print(" ");
      }
      Serial.println();
    } 
  }
  
  else {
    Serial.println("Master: Incomplete data received.");
  }
  
 

  // Small delay that can be setted without much restriction.
  delay(50);
}