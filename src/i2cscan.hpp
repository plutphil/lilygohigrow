bool                    has_lora_shield = false;
bool                    has_bmeSensor   = false;
bool                    has_lightSensor = false;
bool                    has_dhtSensor   = false;
bool                    has_sht3xSensor = false;
bool                    has_ds18b20     = false;
bool                    has_dht11       = false;
void deviceProbe(TwoWire &t)
{

    uint8_t err, addr;
    int nDevices = 0;
    for (addr = 1; addr < 127; addr++) {
        t.beginTransmission(addr);
        err = t.endTransmission();
        if (err == 0) {

            switch (addr) {
            case OB_BH1750_ADDRESS:
                has_lightSensor = true;
                Serial.println("BH1750 light sensor found!");
                break;
            case OB_BME280_ADDRESS:
                has_bmeSensor = true;
                Serial.println("BME280 temperature and humidity sensor found!");
                break;
            case OB_SHT3X_ADDRESS:
                has_sht3xSensor = true;
                Serial.println("SHT3X temperature and humidity sensor found!");
                break;
            default:
                Serial.print("I2C device found at address 0x");
                if (addr < 16)
                    Serial.print("0");
                Serial.print(addr, HEX);
                Serial.println(" !");
                break;
            }
            nDevices++;
        } else if (err == 4) {
            Serial.print("Unknow error at address 0x");
            if (addr < 16)
                Serial.print("0");
            Serial.println(addr, HEX);
        }
    }
}
