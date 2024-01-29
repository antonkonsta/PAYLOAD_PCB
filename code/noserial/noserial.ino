#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SD.h>
#include "Adafruit_BMP3XX.h"
#include <Servo.h>  // Include the Servo library


#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

Servo myservo;
Adafruit_BMP3XX bmp;

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 1;
float altitude = 0;

const int chipSelect = BUILTIN_SDCARD; // Change this if you have an external SD card module

// Functionality Settings
const int printData = 0; // Change this to 1 to see live data printed to monitor
int recordMeasurements = 0;
unsigned long startTime = 0; // Track when the current file was created
unsigned long recordingDuration = 200000; // 200 seconds in milliseconds

int fileCounter = 1;
File dataFile; // Declare the File object globally

// Declare the Adafruit_BNO055 object at a global scope
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire2);

void setup(void)
{
  Serial.begin(115200);
  myservo.attach(23);
  myservo.write(0);
  // Check if SD card is present
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // Don't do anything more:
    while (1);
  }

  openNewFile(); // Open the first file

  /* Initialise the sensor */
  if (!bno.begin()) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    /* There was a problem detecting the BNO055 ... check your connections */
    while (1);
  }

  if (!bmp.begin_I2C(0x77, &Wire1)) {
    Serial.print("Ooops, no BMP detected ... Check your wiring or I2C ADDR!");
    // Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  delay(1000);
}

void openNewFile() {
  // Close the previous file if it's open
  if (dataFile) {
    dataFile.close();
  }

  // Keep trying different file names until an available one is found
  String fileName;
  while (true) {
    fileName = "flightData" + String(fileCounter) + ".csv";
    if (!SD.exists(fileName.c_str())) { // Convert fileName to const char*
      break; // Exit the loop when an available file name is found
    }
    fileCounter++;
  }

  dataFile = SD.open(fileName.c_str(), FILE_WRITE); // Convert fileName to const char*

  if (dataFile) {
    dataFile.println("milliseconds,orientationx,orientationy,orientationz,angVelocityx,angVelocityy,angVelocityz,linAccelerationx,linAccelerationy,linAccelerationz,magneticx,magneticy,magneticz,accelerationx,accelerationy,accelerationz,gravityx,gravityy,gravityz,altitude");
    startTime = millis(); // Update the start time
  }
}

void loop(void)
{
  // could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER, VECTOR_GRAVITY...
  sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

  if (!bmp.performReading()) {
    // Serial.println("Failed to perform reading :(");
    return;
  }

  if (millis() - startTime >= recordingDuration) {
    // Time to open a new file
    fileCounter++;
    openNewFile();
  }

  if (recordMeasurements == 1) {
    altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    Serial.println(altitude);
    if (altitude < 235.95){
      myservo.write(120);
    }
    else if (altitude > 236.1){
      myservo.write(0);
    }

    // Writing data to the open file
    dataFile.print(String(millis()) + ",");
    dataFile.print(String(orientationData.orientation.x) + ",");
    dataFile.print(String(orientationData.orientation.y) + ",");
    dataFile.print(String(orientationData.orientation.z) + ",");
    dataFile.print(String(angVelocityData.gyro.x) + ",");
    dataFile.print(String(angVelocityData.gyro.y) + ",");
    dataFile.print(String(angVelocityData.gyro.z) + ",");
    dataFile.print(String(linearAccelData.acceleration.x) + ",");
    dataFile.print(String(linearAccelData.acceleration.y) + ",");
    dataFile.print(String(linearAccelData.acceleration.z) + ",");
    dataFile.print(String(magnetometerData.magnetic.x) + ",");
    dataFile.print(String(magnetometerData.magnetic.y) + ",");
    dataFile.print(String(magnetometerData.magnetic.z) + ",");
    dataFile.print(String(accelerometerData.acceleration.x) + ",");
    dataFile.print(String(accelerometerData.acceleration.y) + ",");
    dataFile.print(String(accelerometerData.acceleration.z) + ",");
    dataFile.print(String(gravityData.acceleration.x) + ",");
    dataFile.print(String(gravityData.acceleration.y) + ",");
    dataFile.print(String(gravityData.acceleration.z) + ",");
    dataFile.println(String(bmp.readAltitude(SEALEVELPRESSURE_HPA)));

    dataFile.flush(); // Flush the data to the file
  }

  if (recordMeasurements != 1) {
    recordMeasurements += 1;
  }
}
