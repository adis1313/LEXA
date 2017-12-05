#include <Wire.h>
#include <iBUStelemetry.h>

#define DEBUG
#define GPS_DEBUG

#define SERIAL_SPEED          115200

#define SLAVE_I2C_ADDRESS     8

typedef struct //Telemetry values - 24 Bytes
{
      volatile uint16_t temp;   //Temperature           0x01       
      volatile uint16_t head;   //Heading               0x08
      volatile uint16_t crate;  //Climb rate            0x09
      volatile uint16_t spd;    //Speed                 0x13
      volatile uint16_t stat;   //GPS status            0x0b
      volatile uint16_t sat;    //Number of satellites  0x16
            
      volatile int32_t lat;    //Latitude               0x80
      volatile int32_t lon;    //Longitude              0x81
      volatile int32_t alt;    //Altitude               0x82
} tTelemetryValues;

tTelemetryValues telemetryValues;

iBUStelemetry telemetry(4);

void setup() 
{
  initSerial();
  initI2C();
  initTelemetry();
}

void loop() 
{
  requestTelemetryValues();
  delay(20);

  //telemetry.setSensorMeasurement(1, i++);
}

void initSerial() 
{
#if defined (DEBUG)
  Serial.begin(SERIAL_SPEED);
#endif  
}

void initI2C()
{
  Wire.begin();
}

void initTelemetry()
{
  telemetry.begin(115200);
  
  telemetry.addSensor(0x01);    //Temperature
  telemetry.addSensor(0x08);    //Heading
  telemetry.addSensor(0x09);    //Climb rate
  telemetry.addSensor(0x13);    //Speed
  telemetry.addSensor(0x0b);    //GPS status
  telemetry.addSensor(0x16);    //Number of satellites
  telemetry.addSensor(0x80);    //Latitude
  telemetry.addSensor(0x81);    //Longitude
  telemetry.addSensor(0x82);    //Altitude
}

void requestTelemetryValues() 
{
  int i = 0;
  byte telemetrySize = sizeof(telemetryValues);
  
  byte telemetryValuesArray[telemetrySize];
  Wire.requestFrom(SLAVE_I2C_ADDRESS, telemetrySize);

  while (Wire.available() && i < telemetrySize){
    telemetryValuesArray[i] = Wire.read();
    i++;
  }
  memcpy(&telemetryValues, telemetryValuesArray, sizeof(telemetryValues));
  
#if defined (GPS_DEBUG) && defined (DEBUG)
  Serial.print("Temp: "); Serial.print(telemetryValues.temp);
  Serial.print(" Lat: "); Serial.print(telemetryValues.lat);
  Serial.print(" Lon: "); Serial.print(telemetryValues.lon);
  Serial.print(" Alt: "); Serial.print(telemetryValues.alt);
  Serial.print(" Climb rate: "); Serial.print(telemetryValues.crate);
  Serial.print(" Speed: "); Serial.print(telemetryValues.spd);
  Serial.print(" Fix type: "); Serial.print(telemetryValues.stat);      
  Serial.print(" Sat num: "); Serial.print(telemetryValues.sat);  
  Serial.print(" Heading: "); Serial.println(telemetryValues.head);  
#endif  
}

