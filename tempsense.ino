/* tempAndPressureBrakeSubsystem
 *  @description: This does stuff
 *  @author: Jon Barrett
 *  @contact: jobarret@tcd.ie
 *  @date: June 2018
 */

#include <SoftwareSerial.h>
#include <WireKinetis.h>
#include <Wire.h> // I2C library, required for MLX90614
#include <SparkFunMLX90614.h> // SparkFunMLX90614 Arduino library

// Threshold definitions
#define TEMPTHRESHOLDMAX 40.0       //max threshold before a fault is called for temperature sensors

// Address definitions
#define FRONTLEFTSENSORADDRESS 0x2F   //Front Left Temperature Sensor Address
#define FRONTRIGHTSENSORADDRESS 0x2E  //Front Right Temperature Sensor Address

// Timer definitions
#define CALIBRATIONTIME 15000 //Time before sensors become calibrated to their environment

// Software Serial pin definitions
#define rxPin 6
#define txPin 2

// Chip select pin defintions
#define chipSelectSend 11
#define chipSelectRecieve 12
#define BAUDRATE 9600

IRTherm frontLeftTempSensor; //mlx1 Create an IRmlx1 object for the front left temperature sensor to interact with throughout
IRTherm frontRightTempSensor; // Create an IRmlx1 object for the front right temperature sensor to interact with throughout

//SoftwareSerial mySerial = SoftwareSerial(rxPin, txPin);

int state = 1; // Initialize state to IDLE


/* Values to smooth the data
 */
const int numberOfReadings = 5;  // constant for number of values to sample for smoothing
float readingsFL[numberOfReadings]; // Array for readings to smooth at sample size for Front Left Sensor readings (Sample size above)
float readingsFR[numberOfReadings]; // Array for readings to smooth at sample size for Front Right Sensor readings (Sample size above)
//float readingsBL[numberOfReadings]; // Array for readings to smooth at sample size for Back Left Sensor readings (Sample size above)
//float readingsBR[numberOfReadings]; // Array for readings to smooth at sample size for Back Right Sensor readings (Sample size above)
//float readingsPressure[numberOfReadings]; // Array for readings to smooth at sample size for Pressure Sensor readings (Sample size above)

int readIndex = 0; //the read index for the number to iterate

//Smoothened values calculated via average
float smoothenedFL = 0; 
float smoothenedFR = 0;

float totalFL = 0;
float totalFR = 0;

void setup() 
{
  Serial.begin(BAUDRATE); // Initialize Serial to log output

  /*pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);

  pinMode(chipSelectSend, OUTPUT);
  pinMode(chipSelectRecieve, INPUT);

  mySerial.begin(BAUDRATE); */

  frontLeftTempSensor.begin(FRONTLEFTSENSORADDRESS); // Initialize front left IR sensor to address
  frontRightTempSensor.begin(FRONTRIGHTSENSORADDRESS); // Initialize front left IR sensor to address
  
  frontLeftTempSensor.setUnit(TEMP_C); // Set the library's units to Celsius
  frontRightTempSensor.setUnit(TEMP_C); // Set the library's units to Celsius


  /* Initialize all values in each Array for the smoothing to 0
   */
  for (int thisReading = 0; thisReading < numberOfReadings; thisReading++) {
    readingsFL[thisReading] = 0.0;
    readingsFR[thisReading] = 0.0;
  }
  
}



void loop() 
{ 
  
  unsigned long currentMillis = millis(); //timer
  

  /*
   * Determining state for IDLE and READY determining whether the pod has passed the calibration period time (determined in milliseconds by CALIBRATIONTIME)
   * Also determined if each sensor can be read.
   */
  if (frontLeftTempSensor.read() && frontRightTempSensor.read() && (currentMillis > CALIBRATIONTIME)) // On success, read() will return 1, on fail 0.
  {
    if(state != 0){ //if it hasn't set the state to error previously
      state = 2; //Set the state to READY
    }
  }
  else{
    if(state != 0){
      state = 1; //State set to IDLE if it doesn't read or timer isn't above the Calibration Period
    }
  }

  /*  Turning from sensor reading string to floats with 2 points of precision for calculating smoothing
   */
  float frontLeftTemp = String(frontLeftTempSensor.object(),2).toFloat();
  float frontRightTemp = String(frontRightTempSensor.object(),2).toFloat();

  
  //--------------[ Smoothing Data ]------------------//

  //Set total to minus the read index for each sensor to make up for end data once read index is reset
  totalFL = totalFL - readingsFL[readIndex];
  totalFR = totalFR - readingsFR[readIndex];

  // read converted float data to array of each @ read index
  readingsFL[readIndex] = frontLeftTemp;
  readingsFR[readIndex] = frontRightTemp;

  
  // add the reading to the total:
  totalFL = totalFL + readingsFL[readIndex];
  totalFR = totalFR + readingsFR[readIndex];
  
  // advance to the next position in the array:
  readIndex = readIndex + 1;
  // if we're at the end of the array...
  if (readIndex >= numberOfReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the average for each of the sesnsors to get the smoothened value
  smoothenedFL = totalFL / numberOfReadings;
  smoothenedFR = totalFR / numberOfReadings;

  //--------------/ End Smoothing Data /------------------//
  

  
   //--------------[ Ensure Data in Range ]------------------//
  /*  Check if the Temperature is not within the threshold and set the state to 0 if it isn't
   */
  if (!isTempWithinThreshold(smoothenedFL) 
  || (!isTempWithinThreshold(smoothenedFR))) {
       sendData(smoothenedFL, smoothenedFR, state);
      }
  
   //--------------/ End Ensure Data in Range /------------------//

  /*
   * Reads if the pin for chip selection is set to HIGH and if so, sends data as requested.
   * This prevents over-reading in network with multiple Teensys.
   */
  /*if(digitalRead(chipSelectRecieve) == HIGH){
    sendData(smoothenedFL, smoothenedFR, smoothenedBL, smoothenedBR, smoothenedPressure, state);
    while(digitalRead(chipSelectRecieve) == HIGH){
      //do nothing
    }
  } */

  //Debugging Comments vv
  //digitalWrite(chipSelectRecieve, HIGH);
  sendData(smoothenedFL, smoothenedFR, state);
  //delay(50);
}


/* sendData
 *  @description: 
 *            Will convert data to String values for sending via serial and subsequently send said data over serial
 *  @param smoothenedFL:
 *            The smoothened value for the front left temperature sensor
 *  @param smoothenedFR:
 *            The smoothened value for the front right temperature sensor
 *  @param smoothenedBL:
 *            The smoothened value for the back left temperature sensor         
 *  @param smoothenedBR:
 *            The smoothened value for the back right temperature sensor
 *  @param smoothenedPressure:
 *            The smoothened value for the pressure sensor
 *  @param state:
 *            The current state of the pod
 *  @returns void
 */
void sendData(float smoothtenedFL, float smoothenedFR,int state){
  String tempOutFL = String(smoothenedFL,2);
  String tempOutFR = String(smoothenedFR,2);
  Serial.println(tempOutFL + "," + tempOutFR + "," + state + "]");
  //mySerial.print(tempOutFL + "," + tempOutFR + "," + tempOutBL + "," + tempOutBR + "," + pressureOut + "," + state + "]");
}

/* isTempWithinThreshold
 *  @description:
 *            Will determine whether the temperature range is within the specified maximum threshold.
 *  @param tempValue:
 *            The temperature value from a sensor
 *  @returns boolean:
 *            True if the temperature is within the range
 *            False if the temperature is not within the range
 */
boolean isTempWithinThreshold(float tempValue){
  if ( (tempValue < TEMPTHRESHOLDMAX) ){ 
    return true;
  }
  else{
    return false;
  }
}

/* isPressureWithinThreshold
 *  @description:
 *            Will determine whether the pressure is within the upper and lower nominal ranges for the braking system.
 *  @param tempValue:
 *            The pressure value from the analog pressure sensor
 *  @returns boolean:
 *            True if the pressure is within the range
 *            False if the pressure is not within the range         
 */


/* mapFromAnalog: 
 *  @description:
 *            A function that maps the sensor values from analog to pressure bar values.
 *  @param rawValue:
 *            The raw value read in via analog to the address
 *  @param inMin:
 *            The minimum value in from analog
 *  @param inMax:
 *            The maximum value in from analog
 *  @param outMin:
 *            The minimum out value for pressure in bar via the datasheet of sensor
 *  @param outMax:
 *            The maximum out value for pressure in bar via the datasheet of sensor
 *  @returns float:
 *            The returned float value after conversion from analog to pressure value in bar;
 */
/*float mapFromAnalog(float rawValue, float inMin, float inMax, float outMin, float outMax){
  
  return ((rawValue-inMin)*(outMax-outMin)/(inMax-inMin)+outMin);
} */
