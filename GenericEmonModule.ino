#include <SPI.h>

// USER SETTINGS:

// EmonTH temperature RFM12B node ID - should be unique on network
#define NETWORK_NODE_ID 13

// EmonTH RFM12B wireless network group - needs to be same as emonBase and emonGLCD
#define NETWORK_GROUP 210               

// How often to take and transmit readings (seconds)
#define SENSOR_SAMPLE_PERIOD 1.0

// How much to jitter the output by to avoid repeated network collisions (milliseconds)
#define TRANSMIT_RANDOM_JITTER 50

// How many samples to take and average per transmitted reading
#define OVERSAMPLING 200

// How many channels of data are we transmitting?
#define SENSOR_COUNT 8

// Emit a bunch of debugging text to the serial port?
#define SERIAL_DEBUG 1

// Emit debugging text for every sample that is taken?
#define SERIAL_DEBUG_SAMPLES 0

// Set to 1 if using RFM69CW or 0 if using RFM12B
#define RF69_COMPAT 1

// END OF USER SETTINGS

#include <JeeLib.h>

// Frequency of RF12B module can be RF12_433MHZ, RF12_868MHZ or RF12_915MHZ. You should use the one matching the module you have.
#define RF_FREQ RF12_433MHZ 

#include <avr/power.h>

int sensorData[SENSOR_COUNT];
long sensorAccumulators[SENSOR_COUNT];
int lastCycleTime = 0;

void setup() {
  Serial.begin(115200);
  if (SERIAL_DEBUG) Serial.println("GenericEmonModule");
  
  if (SERIAL_DEBUG) Serial.println("Calling setupSensors()");
  setupSensors();
  
  if (SERIAL_DEBUG) Serial.println("Setting Up RFM");
  rf12_initialize(NETWORK_NODE_ID, RF_FREQ, NETWORK_GROUP);

  if (SERIAL_DEBUG) Serial.println("Running\n");
}

void readSensors() {
  // INSERT CODE TO READ YOUR SENSORS HERE
  sensorData[0] = analogRead(A0);
  sensorData[1] = analogRead(A1);
  sensorData[2] = analogRead(A2);
  sensorData[3] = analogRead(A3);
  sensorData[4] = analogRead(A4);
  sensorData[5] = analogRead(A5);
  sensorData[6] = analogRead(A6);
  sensorData[7] = analogRead(A7);
}

void setupSensors() {
  // INSERT CODE TO SET UP YOUR SENSORS HERE:
}

void loop() {
  unsigned long startTime = millis();
  
  int sensorSampleDelay = (SENSOR_SAMPLE_PERIOD * 1000.0 - (TRANSMIT_RANDOM_JITTER / 2)) / OVERSAMPLING;

  resetAccumulators();
  
  for (int i=0; i<OVERSAMPLING; i++) {
    readSensors();
    debugSensorSamples();
    addSensorsToAccumulators();    
    delay(sensorSampleDelay);
  }

  computeSensorAverages();
  
  debugSensorValues();

  transmitSensorData();

  randomDelay(millis() - startTime);
}

void resetAccumulators() {
  for (int i=0; i<SENSOR_COUNT; i++) {
    sensorAccumulators[i] = 0;
  }
}

void addSensorsToAccumulators() {
  for (int i=0; i<SENSOR_COUNT; i++) {
    sensorAccumulators[i] += sensorData[i];
  }
}

void computeSensorAverages() {
  for (int i=0; i<SENSOR_COUNT; i++) {
    sensorData[i] = (float)sensorAccumulators[i] / OVERSAMPLING;  
  }
}

void debugSensorSamples() {
  if (SERIAL_DEBUG_SAMPLES) {
    for (int i=0; i < SENSOR_COUNT; i++) {
      Serial.print(sensorData[i]);
      Serial.print("\t");
    }
    Serial.print("\n");
    Serial.flush();
  }
}

void debugSensorValues() {
  if (SERIAL_DEBUG) {
    for (int i=0; i < SENSOR_COUNT; i++) {
      Serial.print("sensorData["); Serial.print(i); Serial.print("]="); Serial.println(sensorData[i]);
    }
    Serial.flush();
  }
}

void transmitSensorData() {
  rf12_sendNow(0, &sensorData, sizeof sensorData);
  if (SERIAL_DEBUG) { Serial.print("sending "); Serial.print(sizeof sensorData); Serial.print(" bytes on group "); Serial.print(NETWORK_GROUP); Serial.print(", node "); Serial.println(NETWORK_NODE_ID); }
  if (SERIAL_DEBUG) { Serial.flush(); }
}

void randomDelay(unsigned long lastCycleTime) {
  // delay until next cycle.  randomize delay slightly to avoid constant radio conflicts
  int extra = random(0, TRANSMIT_RANDOM_JITTER);
  if (SERIAL_DEBUG) { Serial.print("Cycle time was: "); Serial.println(lastCycleTime); }
  if (SERIAL_DEBUG) { Serial.print("Delaying for "); Serial.println(extra); Serial.print("\n"); Serial.flush(); }
  delay(extra);
}

