/*
  Project History Taken from ESPPowerMonitorHouse
  v1.0 - 2025-02-02 - Initial version
  ****  ONLY ADC1 pins can be used for analogRead() function while WiFi
  is used  ****
  Pins: voltsPin = 32;
  Temperature sensors = 33 (Engine, coolant,ambient)
  Oil pressure = 34
  Fuel gauge = 35
*/
#include "secrets.h"
#include <WiFi.h>
#include <WiFiMulti.h>
WiFiMulti wifiMulti;
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <DallasTemperature.h>
#include <ESPmDNS.h>
#include <InfluxDbClient.h>
#include <InfluxDbCloud.h>
#include <OneWire.h>
#include <nvs.h>        // testing
#include <nvs_flash.h>  // testing

#define ONE_WIRE_BUS 33  // this is D33
OneWire ds(33);          // Temperature sensor setup
float AmbiantTemperature;
float EngineTemperature;
float CoolantTemperature;


// Perth, Western Australia
#define TZ_INFO "AWST-8"

// Declare InfluxDB client instance with preconfigured InfluxCloud certificate
InfluxDBClient client(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET,
                      INFLUXDB_TOKEN, InfluxDbCloud2CACert);

// Declare Data point Measurement
Point sensor("Generator Metrics");

const uint32_t connectTimeoutMs = 10000;
unsigned long watchdogtimer = 0;
unsigned long previousMillis;
bool otaInProgress = false;

int LED_BUILTIN = 2;
int RSSI = 0;
unsigned long CycleTime;
unsigned long VoidCycleTime;
uint32_t storedTotalMillis;
uint32_t storedTankMillis;
float Voltage;
float VoltageADC;
float OilADC;
float OilPressure;
float GenCurrent;
float GenCutOffLimit = 10;  // Arbitary ADC cuttoff value, below which the
                            // generator is not charging
const int voltsPin = 32;
const int OilPin = 34;
const int FuelPin = 35;
int fuelADC = 0;
float FuelLevel = 0;
float LastFuelLevel = 0;  // Store the last saved fuel level
float FuelEconomy = 0;
const int TankCapacity = 46;  //  46L tank: 58 * 40 * 20
const unsigned long HOURS = 5000;
const unsigned long MILLISECONDS_PER_HOUR = 3600000;
unsigned long engineRunTime =
    HOURS * MILLISECONDS_PER_HOUR;  // Total engine run time in milliseconds
                                    // excluding millis until used
unsigned long engineRunTimeSinceRefuel =
    0;  // Total engine run time in milliseconds since last refuel
unsigned long Flag5s = 0;
unsigned long Flag30s = 0;
// #define ACTectionRange 20    //set Non-invasive AC Current Sensor tection
// range (5A,10A,20A)

String formatRuntime(unsigned long millis) {
  unsigned long seconds = millis / 1000;
  unsigned long minutes = seconds / 60;
  unsigned long hours = minutes / 60;

  seconds = seconds % 60;
  minutes = minutes % 60;

  char buffer[9];  // HH:MM:SS format
  snprintf(buffer, sizeof(buffer), "%02lu:%02lu:%02lu", hours, minutes,
           seconds);
  return String(buffer);
}

float getCurrent() {
  WiFiClientSecure client;
  HTTPClient http;
  client.setInsecure();
  String url = String(INFLUXDB_URL) + "/api/v2/query?org=" + INFLUXDB_ORG;

  http.begin(client, url);
  http.addHeader("Authorization", String("Token ") + INFLUXDB_TOKEN);
  http.addHeader("Content-Type", "application/vnd.flux");

  String query = "from(bucket:\"" + String(INFLUXDB_BUCKET) +
                 "\") |> range(start:-1h) |> filter(fn:(r) => r._measurement "
                 "== \"Power Metrics\" and r._field == \"CTMeter\") |> last()";
  int httpCode = http.POST(query);

  if (httpCode == 200) {
    String response = http.getString();
    if (response.length() <= 2) {
      http.end();
      return -1;  // No data
    }

    // Split response into header and data
    int firstNewline = response.indexOf("\n");
    if (firstNewline == -1) {
      http.end();
      return -1;  // No header
    }

    String header = response.substring(0, firstNewline);
    String dataLine = response.substring(firstNewline + 1);

    // Find _value column index in header
    int valueIdx = -1;
    int commaCount = 0;
    int startIdx = 0;
    for (int i = 0; i <= header.length(); i++) {
      if (i == header.length() || header.charAt(i) == ',') {
        if (header.substring(startIdx, i) == "_value") {
          valueIdx = commaCount;
          break;
        }
        commaCount++;
        startIdx = i + 1;
      }
    }
    if (valueIdx == -1) {
      http.end();
      return -1;  // _value not found
    }

    // Extract _value from data line
    commaCount = 0;
    startIdx = 0;
    for (int i = 0; i <= dataLine.length(); i++) {
      if (i == dataLine.length() || dataLine.charAt(i) == ',') {
        if (commaCount == valueIdx) {
          GenCurrent = dataLine.substring(startIdx, i).toFloat();
          http.end();
          // Serial.print("GenCurrent: ");
          // Serial.println(GenCurrent);
          return GenCurrent;
        }
        commaCount++;
        startIdx = i + 1;
      }
    }
  }
  http.end();
  return -1;  // Error or no data
}

float getFuelLevel() {
  fuelADC = analogRead(FuelPin);
  for (int i = 0; i < 50; i++) {
    fuelADC += analogRead(FuelPin);
  }
  fuelADC = fuelADC / 50;

  // Ensure ADC value is within the range of ±5
  if (fuelADC < 100 || fuelADC > 517) {
    return -1;  // Invalid ADC value
  }

  // Map ADC values to fuel levels
  if (fuelADC >= 100 && fuelADC <= 110) return FuelLevel = 100;  // 105 ± 5
  if (fuelADC >= 141 && fuelADC <= 151) return FuelLevel = 80;   // 146 ± 5
  if (fuelADC >= 178 && fuelADC <= 188) return FuelLevel = 60;   // 183 ± 5
  if (fuelADC >= 222 && fuelADC <= 232) return FuelLevel = 40;   // 227 ± 5
  if (fuelADC >= 315 && fuelADC <= 325) return FuelLevel = 20;   // 320 ± 5
  if (fuelADC >= 507 && fuelADC <= 517) return FuelLevel = 0;    // 512 ± 5

  // Interpolate for values between the defined ranges
  if (fuelADC > 110 && fuelADC < 141) {
    return map(fuelADC, 110, 141, 100, 80);
  }
  if (fuelADC > 151 && fuelADC < 178) {
    return map(fuelADC, 151, 178, 80, 60);
  }
  if (fuelADC > 188 && fuelADC < 222) {
    return map(fuelADC, 188, 222, 60, 40);
  }
  if (fuelADC > 232 && fuelADC < 315) {
    return map(fuelADC, 232, 315, 40, 20);
  }
  if (fuelADC > 325 && fuelADC < 507) {
    return map(fuelADC, 325, 507, 20, 0);
  }

  return FuelLevel;  // Default case for invalid values
}

float getFuelEconomy() {
  if (engineRunTimeSinceRefuel == 0) {
    return 0;  // Avoid division by zero
  }

  // Convert fuel level percentage to actual fuel volume in liters
  float fuelUsed = TankCapacity * (100 - FuelLevel) / 100.0;

  // Calculate fuel economy in liters per hour
  FuelEconomy =
      fuelUsed / (engineRunTimeSinceRefuel / 1000.0 / 60.0 / 60.0);  // L/hr
  return FuelEconomy;
}

float MeasureTemperature(byte addr[8]) {
  byte i;
  byte present = 0;
  byte data[12];

  float celsius = 0;

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);
  delay(50);

  present = ds.reset();
  ds.select(addr);
  ds.write(0xBE);

  for (i = 0; i < 9; i++) {
    data[i] = ds.read();
  }

  int16_t raw1 = (data[1] << 8) | data[0];

  byte cfg = (data[4] & 0x60);
  if (cfg == 0x00)
    raw1 = raw1 & ~7;
  else if (cfg == 0x20)
    raw1 = raw1 & ~3;
  else if (cfg == 0x40)
    raw1 = raw1 & ~1;

  celsius = (float)raw1 / 16.0;
  return celsius;
}

float MeasureTemperature1() {  //  Ambient temperature (black lead)
  byte addr[8] = {0x28, 0x00, 0x15, 0x27, 0x46, 0x4C, 0x12, 0x51};
  return MeasureTemperature(addr);
}

float MeasureTemperature2() {  //  Engine Temperature (silver lead)
  byte addr[8] = {0x28, 0xCD, 0x50, 0x16, 0xA8, 0x01, 0x3C, 0x65};
  return MeasureTemperature(addr);
}

float MeasureTemperature3() {  //  Coolant Temperature (white lead)
  byte addr[8] = {0x28, 0x00, 0x95, 0x2A, 0x46, 0x4C, 0x17, 0x38};
  return MeasureTemperature(addr);
}

void MeasureVoltage() {
  float totalADC = 0;
  int Samples = 60;

  for (int i = 0; i < Samples; i++) {
    Voltage = analogRead(voltsPin);
    totalADC += Voltage;
  }
  VoltageADC = totalADC / Samples;
  Voltage = (VoltageADC * 0.0042895 + 1.5);
  if (VoltageADC < GenCutOffLimit) {  //  When the generator is switched off,
                                      //  ESP has no power.
    digitalWrite(LED_BUILTIN, LOW);
    storedTotalMillis = engineRunTime + millis();
    storedTankMillis = millis() - engineRunTimeSinceRefuel;
    nvs_handle_t handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
      Serial.printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
      err = nvs_set_u32(handle, "Total_millis", storedTotalMillis);
      if (err != ESP_OK) {
        Serial.printf("Error (%s) setting Total_millis!\n",
                      esp_err_to_name(err));
      }
      err = nvs_set_u32(handle, "Tank_millis", storedTankMillis);
      if (err != ESP_OK) {
        Serial.printf("Error (%s) setting Tank_millis!\n",
                      esp_err_to_name(err));
      }
      err = nvs_set_blob(handle, "Fuel_Level", &FuelLevel, sizeof(FuelLevel));
      if (err != ESP_OK) {
        Serial.printf("Error (%s) setting Fuel_Level!\n", esp_err_to_name(err));
      }
      err = nvs_commit(handle);
      if (err != ESP_OK) {
        Serial.printf("Error (%s) committing NVS!\n", esp_err_to_name(err));
      }
      nvs_close(handle);
    }
  }
}

void MeasureOilPressure() {
  float TotalADC = 0;
  int samples = 10;

  for (int i = 0; i < samples; i++) {
    OilADC = analogRead(OilPin);
    TotalADC += OilADC;
  }
  OilADC = TotalADC / samples;
  OilPressure = (OilADC / 4095) * 100;  // Not calibrated as yet
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  pinMode(LED_BUILTIN, OUTPUT);
  VoidCycleTime = millis();
  analogReadResolution(12);
  Flag30s = millis();

  // Initialize NVS
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
      err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    // NVS partition was truncated and needs to be erased
    // Retry nvs_flash_init
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);

  // Open NVS handle
  nvs_handle_t handle;
  err = nvs_open("storage", NVS_READWRITE, &handle);

  if (err != ESP_OK) {
    Serial.printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
  } else {
    // Read stored total millis value (engine hours)
    err = nvs_get_u32(handle, "Total_millis", &storedTotalMillis);
    switch (err) {
      case ESP_OK:
        Serial.printf("Total_millis: %u\n", storedTotalMillis);
        engineRunTime += storedTotalMillis;
        break;
      case ESP_ERR_NVS_NOT_FOUND:
        Serial.println("The value is not initialized yet!");
        break;
      default:
        Serial.printf("Error (%s) reading Total_millis!\n",
                      esp_err_to_name(err));
    }
    // Read stored  millis since refuel value
    err = nvs_get_u32(handle, "Tank_millis", &storedTankMillis);
    switch (err) {
      case ESP_OK:
        Serial.printf("Tank_millis: %u\n", storedTankMillis);
        engineRunTimeSinceRefuel = storedTankMillis;
        break;
      case ESP_ERR_NVS_NOT_FOUND:
        Serial.println("The value is not initialized yet!");
        break;
      default:
        Serial.printf("Error (%s) reading Tank_millis!\n",
                      esp_err_to_name(err));
    }

    // Read stored Fuel_Level value
    size_t required_size = sizeof(FuelLevel);
    err = nvs_get_blob(handle, "Fuel_Level", &LastFuelLevel, &required_size);
    switch (err) {
      case ESP_OK:
        Serial.printf("Stored Fuel_Level: %f\n", LastFuelLevel);
        break;
      case ESP_ERR_NVS_NOT_FOUND:
        Serial.println("The value is not initialized yet!");
        break;
      default:
        Serial.printf("Error (%s) reading Fuel_Level!\n", esp_err_to_name(err));
    }

    // Close NVS handle
    nvs_close(handle);
  }
  //   emon1.current(ACPin, 111.1);             // Current: input pin,
  //   calibration. delay(1000);
  // connect to WiFi
  Serial.println();
  Serial.print("Connecting to wifi... ");
  WiFi.setHostname("GenMonitor");
  ArduinoOTA.setHostname("GenMonitor");
  wifiMulti.addAP(ssid1, password1);
  wifiMulti.addAP(ssid2, password2);
  // esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_LR);   // testing
  // WiFi.setPhyMode(WIFI_PHY_MODE_11B);  // testing
  Serial.println("Starting WiFi");
  wifiMulti.run(10000);
  Serial.println(WiFi.RSSI());
  Serial.flush();
  // OTA event handlers (optional, for debug)
  ArduinoOTA.onStart([]() { Serial.println("OTA Update Start"); });
  ArduinoOTA.onEnd([]() { Serial.println("OTA Update End"); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("OTA Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR)
      Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR)
      Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR)
      Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR)
      Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR)
      Serial.println("End Failed");
  });

  if (wifiMulti.run() == WL_CONNECTED) {
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    // Start OTA
    Serial.println("Starting ArduinoOTA...");
    ArduinoOTA.begin();
    Serial.println("ArduinoOTA started, listening on port 3232");
    ArduinoOTA.onStart([]() {
      Serial.println("OTA Update Start");
      otaInProgress = true;  // Set flag to pause readings
    });
    ArduinoOTA.onEnd([]() {
      Serial.println("OTA Update End");
      otaInProgress = false;  // Resume readings
    });
  } else {
    Serial.println("\nWiFi connection failed");
  }

  //   Serial.println("");
  //   timeSync(TZ_INFO, "pool.ntp.org", "time.nis.gov");

  // Check server connection
  if (client.validateConnection()) {
    Serial.print("Connected to InfluxDB: ");
    Serial.println(client.getServerUrl());
  } else {
    Serial.print("InfluxDB connection failed: ");
    Serial.println(client.getLastErrorMessage());
  }

  getFuelLevel();
  if (FuelLevel < LastFuelLevel + 5) {  // not refuelled since startup
    FuelLevel = LastFuelLevel;
  }
}

void loop() {
  ArduinoOTA.handle();  // Handle OTA updates
  if (otaInProgress) {
    return;
  }
  // watchdogtimer = millis();

  if (wifiMulti.run() == WL_CONNECTED) {
    digitalWrite(LED_BUILTIN, HIGH);
    RSSI = int(WiFi.RSSI());
    previousMillis = millis();
    Serial.print("WiFi connected to ");
    Serial.print(WiFi.SSID());
    Serial.print(" ");
    Serial.println(WiFi.RSSI());
    if (millis() - Flag5s > 5000) {
      Flag5s = millis();
      getCurrent();
      if (GenCurrent <= GenCutOffLimit) {  // if regulator is charging
        GenCurrent = 0;
      }
    }
    MeasureVoltage();  // Check generator DC power supply voltage
    MeasureOilPressure();
    getFuelLevel();
    getFuelEconomy();
    AmbiantTemperature = MeasureTemperature1();
    EngineTemperature = MeasureTemperature2();
    CoolantTemperature = MeasureTemperature3();
    String runtimeStr = formatRuntime(millis());

    CycleTime = millis() - VoidCycleTime;  // time taken to complete a loop
    VoidCycleTime = millis();
    sensor.clearFields();
    sensor.addField("Gen Voltage", Voltage);
    sensor.addField("Gen Voltage ADC", VoltageADC);
    sensor.addField("Gen Oil Pressure", OilPressure);
    sensor.addField("Gen Oil Pressure ADC", OilADC);
    sensor.addField("Ambiant Temp", AmbiantTemperature);
    sensor.addField("Engine Temp", EngineTemperature);
    sensor.addField("Coolant Temp", CoolantTemperature);
    sensor.addField("Generator Current", GenCurrent);
    sensor.addField("Fuel Economy", FuelEconomy);
    sensor.addField("Fuel Level", FuelLevel);
    sensor.addField("Fuel Level ADC", fuelADC);
    sensor.addField("Session run time", runtimeStr);
    sensor.addField("Total run time",
                    (engineRunTime + millis()) / MILLISECONDS_PER_HOUR);
    sensor.addField("RSSI", RSSI);

    if (!client.writePoint(sensor)) {
      Serial.print("InfluxDB write failed: ");
      Serial.println(client.getLastErrorMessage());
    }

  } else {
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("WiFi not connected...");
  }
}