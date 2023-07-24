#define DHTTYPE           DHT11     // DHT 11
// #define DHTTYPE           DHT22     // DHT 22 (AM2302)
//#define DHTTYPE           DHT21     // DHT 21 (AM2301)

#define OB_BH1750_ADDRESS       (0x23)
#define OB_BME280_ADDRESS       (0x77)
#define OB_SHT3X_ADDRESS        (0x44)

#define I2C_SDA                 (25)
#define I2C_SCL                 (26)
#define I2C1_SDA                (21)
#define I2C1_SCL                (22)
#define DS18B20_PIN             (21)

#define DHT1x_PIN               (16)
#define BAT_ADC                 (33)
#define SALT_PIN                (34)
#define SOIL_PIN                (32)
#define BOOT_PIN                (0)
#define POWER_CTRL              (4)
#define USER_BUTTON             (35)
#define DS18B20_PIN             (21)                  //18b20 data pin

#include <Adafruit_BME280.h>
#include <Adafruit_SHT31.h>
#include <OneWire.h>
#include "DHT.h"
#include <BH1750.h>

#include "fdrs_node_config.h"
#include <fdrs_node.h>

#include "i2cscan.hpp"

BH1750              lightMeter(OB_BH1750_ADDRESS);  //0x23
DHT                 dht(DHT1x_PIN, DHTTYPE);
OneWire             ds;
Adafruit_SHT31      sht31 = Adafruit_SHT31(&Wire1);
Adafruit_BME280     bme;                            //0x77
float getsalt(){
  uint8_t samples = 120;
  uint32_t humi = 0;
  uint16_t arr[120];
  for (int i = 0; i < samples; i++) {
      int v = analogRead(SALT_PIN);
      humi += v;
      delay(2);
  }
  humi /= samples - 2;
  return humi / (float)(samples - 2);
}
float getbatvol(){
  int vref = 1100;
  uint16_t volt = analogRead(BAT_ADC);
  return ((float)volt / 4095.0) * 6.6 * (vref);  
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(POWER_CTRL, OUTPUT);
  digitalWrite(POWER_CTRL, HIGH);
  delay(100);
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire1.begin(I2C1_SDA, I2C1_SCL);
  deviceProbe(Wire);
  deviceProbe(Wire1);
  dht.begin();

  if (!lightMeter.begin()) {
    Serial.println("Warning: Failed to find BH1750 light sensor!");
  } else {
    Serial.println("BH1750 light sensor init succeeded, using BH1750");
  }
  dht.begin();

  beginFDRS();
  
}

void loop() {
  // put your main code here, to run repeatedly:
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    delay(10);
    return;
  }

  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);

  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.print(F("%  Temperature: "));
  Serial.print(t);
  Serial.print(F("°C "));
  Serial.print(f);
  Serial.print(F("°F  Heat index: "));
  Serial.print(hic);
  Serial.print(F("°C "));
  Serial.print(hif);
  Serial.println(F("°F"));
  Serial.print(F("LightLevel: "));
  float light = lightMeter.readLightLevel();
  Serial.print(light);
  Serial.println();

  int soil = analogRead(SOIL_PIN);
  int soil_perc = map(soil, 0, 4095, 100, 0);
  float salt = getsalt();
  float batvol = getbatvol();
  Serial.print(F("soil: "));
  Serial.print(soil);
  Serial.print(F("soil_perc: "));
  Serial.print(soil_perc);
  Serial.println();
  Serial.print(F("salt: "));
  Serial.print(salt,10);
  Serial.println();
  Serial.print(F("batvol: "));
  Serial.print(batvol);
  Serial.println();
  
  loadFDRS(h, HUMIDITY_T);
  loadFDRS(t, TEMP_T);
  loadFDRS(soil, SOIL_T);
  loadFDRS(soil_perc, SOIL2_T);
  loadFDRS(salt, SOILR_T);
  loadFDRS(analogRead(SALT_PIN), SOILR2_T);
  loadFDRS(light,LIGHT_T);
  loadFDRS(batvol, VOLTAGE_T);

  int tries =10;
  while(!sendFDRS()){
    DBG("Err sending message.");
    delay(500);
    tries--;
    if(tries<0){
      break;  //Sleep time in seconds
    }
  }
    DBG("Big Success!");
  digitalWrite(POWER_CTRL, LOW);
  sleepFDRS(60*10);  //Sleep time in seconds
}
