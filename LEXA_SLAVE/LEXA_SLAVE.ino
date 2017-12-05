#include <NazaDecoderLib.h>
#include <Wire.h>

#define DEBUG
#define GPS_DEBUG
//#define TELEMETRY_ARRAY_DEBUG

#define SERIAL_SPEED          115200
#define GPS_SERIAL_SPEED      115200
#define IBUS_SERIAL_SPEED     115200

#define I2C_ADDRESS           8

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

byte telemetryValuesSize = sizeof(telemetryValues);

void setup()
{
  initSerial();
  initI2C();
}

void loop()
{
  readTelemetryValues();
}

void initSerial() 
{
#if defined (DEBUG)
  Serial.begin(SERIAL_SPEED);
#endif
  Serial2.begin(GPS_SERIAL_SPEED); //GPS serial
  Serial3.begin(IBUS_SERIAL_SPEED); //iBus serial
}

void initI2C()
{
  Wire.begin(I2C_ADDRESS);
  Wire.onRequest(sendTelemetryValues);
}

void readTelemetryValues() 
{
  if(Serial2.available())
  {
    uint8_t decodedMessage = NazaDecoder.decode(Serial2.read());
    switch (decodedMessage)
    {
      case NAZA_MESSAGE_GPS:

        telemetryValues.lat = (int32_t)(NazaDecoder.getLat() * 10000000);
        telemetryValues.lon = (int32_t)(NazaDecoder.getLon() * 10000000);      
        telemetryValues.alt = (int32_t)(NazaDecoder.getGpsAlt() * 100);        
        
        telemetryValues.crate = (uint16_t)(NazaDecoder.getGpsVsi() * 100);
        telemetryValues.spd = (uint16_t)(NazaDecoder.getSpeed() * 100);
        telemetryValues.stat = (uint16_t)(NazaDecoder.getFixType());
        telemetryValues.sat = NazaDecoder.getFixType();

#if defined (GPS_DEBUG) && defined (DEBUG)
        Serial.print("Lat: "); Serial.print(telemetryValues.lat);
        Serial.print(" Lon: "); Serial.print(telemetryValues.lon);
        Serial.print(" Alt: "); Serial.print(telemetryValues.alt);
        Serial.print(" Climb rate: "); Serial.print(telemetryValues.crate);
        Serial.print(" Speed: "); Serial.print(telemetryValues.spd);
        Serial.print(" Fix type: "); Serial.print(telemetryValues.stat);      
        Serial.print(" Sat num: "); Serial.print(telemetryValues.sat);  
        Serial.print(" Heading: "); Serial.println(telemetryValues.head);  
#endif

        break;
      case NAZA_MESSAGE_COMPASS:
        
        telemetryValues.head = (uint16_t)(NazaDecoder.getHeadingNc());      
      
        break;
    }
  }
}

void sendTelemetryValues() 
{  
  tTelemetryValues telemetryValuesCopy = telemetryValues;

  byte telemetryValuesArray[telemetryValuesSize]; 
  memcpy(telemetryValuesArray, &telemetryValuesCopy, telemetryValuesSize); //Convert struct to byte array

  for(int i = 0; i < telemetryValuesSize; i++) {
    Wire.write(telemetryValuesArray[i]);

#if defined (TELEMETRY_ARRAY_DEBUG) && defined (DEBUG)
    Serial.print(i);
    Serial.print(": ");
    Serial.println(telemetryValuesArray[i]);
#endif
  }  
}

