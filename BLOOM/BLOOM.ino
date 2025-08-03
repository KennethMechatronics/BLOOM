/*
 * Copyright (c) 2025 Kenneth Paulsen
 * Licensed under the MIT License.
 * See LICENSE file in the project root for full license information.
 */

#include <HardwareSerial.h>
#include <WiFi.h>
#include <time.h>
#include <esp_sntp.h>
#include <PubSubClient.h>
#include <credentials.h>

// credentials.h
const char* ssid = SSID;
const char* WiFiPw = WIFI_PW;
const char* HostName = "BLOOM";
const char* MQTTserver = MQTT_SERVER;
const char* MQTTuser = MQTT_USER;
const char* MQTTpw = MQTT_PW;


// Control conectivity
const bool connectivity = true;
const int maxNumberOfTries = 9;
int WiFiTry = 0;
int MQTTTry = 0;


WiFiClient espClient;
PubSubClient MQTTclient(espClient);

// NTP settings
const char* ntpServer1 = "pool.ntp.org";
const char* ntpServer2 = "time.nist.gov";
const long gmtOffset_sec = 3600;
const int daylightOffset_sec = 3600;
char timeStr[64];  // Could be less


// RS485/Modbus
HardwareSerial RS485Serial(1);
const int RS485_DIR = 4;
const int RS485_RX = 20;
const int RS485_TX = 21;
const int RS485_BAUD = 9600;

// Pins
const int ledPin = 1;
const int GPIO2 = 2;
const int loadSwitchEN = 3;
const int motorDriverIN1 = 5;
const int motorDriverIN2 = 6;
const int GPIO7 = 7;
const int GPIO10 = 10;

// RUN-indication (blinking LED)
bool runIndication = false;
unsigned long prevBlink = 0;
const int blinkInterval = 1800000;
bool ledState = false;

// Transmitting interval control

const int transmittInterval = 120000;  // 120000;
unsigned long prevTransmitt = transmittInterval;

// Sensor and motor driver power states
bool motorDriverPowered = true;
bool sensorPowered = false;



bool sensorPower(bool powerState) {
  // Datasheet: https://www.ti.com/lit/ds/symlink/tps22810.pdf
  // powers TPS22810DRVR load switch
  digitalWrite(loadSwitchEN, powerState);
  if (powerState == 1) {
    delayMicroseconds(400);
    Serial.println("Soil sensor ON");
    return sensorPowered = true;

  } else {
    delayMicroseconds(4);
    Serial.println("Soil sensor OFF");
    return sensorPowered = false;
  }
}

bool motorDriverState(bool state) {
  // Datasheet: https://www.ti.com/lit/ds/symlink/drv8871-q1.pdf
  // Power DRV8871DDAR or put to sleep
  if (state == 0) {
    // Put to sleep
    digitalWrite(motorDriverIN1, LOW);
    digitalWrite(motorDriverIN2, LOW);
    delay(2);  // Wait for sleep mode
    Serial.println("Motor driver sleeping");
    return motorDriverPowered = 0;

  } else {
    // Power up
    digitalWrite(motorDriverIN1, HIGH);
    digitalWrite(motorDriverIN2, HIGH);
    delayMicroseconds(5);  // Power on time
    delay(50);             // Wait for wake up
    Serial.println("Motor driver powered up");
    return motorDriverPowered = 1;
  }
}

void connectWiFi() {
  WiFiTry = 0;

  Serial.println("Starting WiFi");
  WiFi.begin(ssid, WiFiPw);

  Serial.println("Connecting to ssid: " + String(ssid));
  while (WiFi.status() != WL_CONNECTED && WiFiTry <= maxNumberOfTries) {
    WiFiTry++;
    delay(500);
    Serial.print(".");
  }

  if (WiFiTry <= maxNumberOfTries) {
    WiFi.setHostname(HostName);
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.println("Signal Strength: " + String(WiFi.RSSI()) + " dBm");
  } else {
    Serial.println("\nERROR: WiFi could not connect");
  }
}

void getTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("No time available (yet)");
    return;
  }

  strftime(timeStr, sizeof(timeStr), "%A, %B %d %Y %H:%M:%S", &timeinfo);
  //Serial.println(timeStr);  // Print to Serial
}

void connectMQTT() {
  MQTTTry = 0;

  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }

  Serial.println("Connecting to MQTT");
  MQTTclient.setServer(MQTTserver, 1883);  // 1883
  while (!MQTTclient.connect(HostName, MQTTuser, MQTTpw) && MQTTTry <= maxNumberOfTries) {
    MQTTTry++;
    delay(500);
    Serial.print(".");
  }

  if (MQTTclient.connect(HostName, MQTTuser, MQTTpw)) {
    Serial.println("MQTT connected");
  } else {
    Serial.println("\nERROR: MQTT could not connect");
  }
}


class soilSensor {
public:
  // Sensor datasheet: https://wiki.dfrobot.com/RS485_Soil_Sensor_Temperature_Humidity_SKU_SEN0600
  float humidity;
  float temperature;
  float humCal;
  float tempCal;
  float deviceAddr;
  float baud;

  soilSensor()
    : humidity(0.0f), temperature(0.0f), humCal(0.0f), tempCal(0.0f), deviceAddr(0), baud(0) {}

  void read() {
    uint16_t values[2];

    if (sensorPowered == false) {
      sensorPower(true);
    }

    sendModbusRequest(0x01, 0x0000, 2);
    delay(100);
    if (readModbusResponse(values, 2)) {
      humidity = values[0] / 10.0f;
      temperature = values[1] / 10.0f;
    } else {
      Serial.println("Failed to read humidity & temperature.");
      humidity = -1;
      temperature = -1;
    }
  }

  void readCalData() {
    uint16_t values[2];

    if (sensorPowered == false) {
      sensorPower(true);
    }

    // Read calibration values
    sendModbusRequest(0x01, 0x0050, 2);
    delay(100);
    if (readModbusResponse(values, 2)) {
      tempCal = values[0] / 10.0;
      humCal = values[1] / 10.0;
    } else {
      Serial.println("Failed to read calibration values.");
    }

    // Read device address & baud rate
    sendModbusRequest(0x01, 0x07D0, 2);
    delay(100);
    if (readModbusResponse(values, 2)) {
      deviceAddr = values[0];
      baud = values[1];
    } else {
      Serial.println("Failed to read device info.");
    }
  }

  void writeCalibration(float newTempCal, float newHumCal) {
    // Convert floats to register format (sensor expects 1 decimal place)
    uint16_t tempVal = (uint16_t)(newTempCal * 10);
    uint16_t humVal = (uint16_t)(newHumCal * 10);

    if (sensorPowered == false) {
      sensorPower(true);
    }

    // Write temperature calibration to register 0x0050
    writeModbusRegister(0x01, 0x0050, tempVal);
    delay(100);

    // Write humidity calibration to register 0x0051 (next register)
    writeModbusRegister(0x01, 0x0051, humVal);
  }


private:
  uint16_t calculateCRC(uint8_t* data, uint8_t length) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < length; i++) {
      crc ^= data[i];
      for (uint8_t j = 0; j < 8; j++) {
        if (crc & 0x0001) {
          crc >>= 1;
          crc ^= 0xA001;
        } else {
          crc >>= 1;
        }
      }
    }
    return crc;
  }

  void sendModbusRequest(uint8_t deviceAddr, uint16_t startReg, uint16_t numRegs) {
    uint8_t frame[8];
    frame[0] = deviceAddr;
    frame[1] = 0x03;  // Read holding registers
    frame[2] = startReg >> 8;
    frame[3] = startReg & 0xFF;
    frame[4] = numRegs >> 8;
    frame[5] = numRegs & 0xFF;
    uint16_t crc = calculateCRC(frame, 6);
    frame[6] = crc & 0xFF;
    frame[7] = crc >> 8;

    digitalWrite(RS485_DIR, HIGH);
    delayMicroseconds(100);
    RS485Serial.write(frame, 8);
    RS485Serial.flush();
    delayMicroseconds(100);
    digitalWrite(RS485_DIR, LOW);
  }

  bool readModbusResponse(uint16_t* values, uint8_t expectedRegs) {
    uint8_t expectedBytes = 5 + expectedRegs * 2;  // Addr + Func + ByteCount + Data + CRC
    unsigned long start = millis();
    while (RS485Serial.available() < expectedBytes && millis() - start < 1000)
      ;

    if (RS485Serial.available() >= expectedBytes) {
      uint8_t resp[256];
      for (uint8_t i = 0; i < expectedBytes; i++) resp[i] = RS485Serial.read();

      // CRC check
      uint16_t crcCalc = calculateCRC(resp, expectedBytes - 2);
      uint16_t crcRecv = resp[expectedBytes - 2] | (resp[expectedBytes - 1] << 8);
      if (crcCalc != crcRecv) return false;

      for (uint8_t i = 0; i < expectedRegs; i++) {
        values[i] = (resp[3 + i * 2] << 8) | resp[4 + i * 2];
      }
      return true;
    }
    return false;
  }

  void writeModbusRegister(uint8_t deviceAddr, uint16_t regAddr, uint16_t value) {
    uint8_t frame[8];
    frame[0] = deviceAddr;
    frame[1] = 0x06;  // Function code: Write Single Register
    frame[2] = regAddr >> 8;
    frame[3] = regAddr & 0xFF;
    frame[4] = value >> 8;
    frame[5] = value & 0xFF;

    uint16_t crc = calculateCRC(frame, 6);
    frame[6] = crc & 0xFF;
    frame[7] = crc >> 8;

    digitalWrite(RS485_DIR, HIGH);
    delayMicroseconds(100);
    RS485Serial.write(frame, 8);
    RS485Serial.flush();
    delayMicroseconds(100);
    digitalWrite(RS485_DIR, LOW);


    delay(100);
    while (RS485Serial.available()) RS485Serial.read();  // Clear buffer
  }
};

soilSensor soilSensor;



void setup() {
  delay(2000);  // Wait for PSU to settle etc.
  Serial.begin(115200);
  while (!Serial) {
    ;
  }
  Serial.println("Serial started \nBaud rate: " + String(Serial.baudRate()));

  Serial.print("Configuring GPIO: ");
  for (int i = 1; i <= 10; i++) {
    if (i == 8 || i == 9) continue;  // leave 8 and 9 floating
    pinMode(i, OUTPUT);
    digitalWrite(i, LOW);
    Serial.print(i);
    if (i == 10) {
      Serial.println();
      continue;  // Skip "," on the last one
    }
    Serial.print(", ");
  }
  Serial.println("GPIOs configured");

  motorDriverState(0);
  sensorPower(0);

  Serial.println("Starting RS485");
  RS485Serial.begin(RS485_BAUD, SERIAL_8N1, RS485_RX, RS485_TX);
  while (!RS485Serial) {
    delay(20);
    Serial.print(".");
  }
  Serial.println("RS485 Started");

  soilSensor.writeCalibration(0.0, 0.0);
  soilSensor.readCalData();
  Serial.println("Reading sensor");

  Serial.print("Device Address: 0x");
  int RS485DeviceAddrTempVal = soilSensor.deviceAddr;
  Serial.println(RS485DeviceAddrTempVal, HEX);

  Serial.print("Baud rate: ");
  int RS485BaudTempVal = soilSensor.baud;
  switch (RS485BaudTempVal) {
    case 0: Serial.println("2400"); break;
    case 1: Serial.println("4800"); break;
    case 2: Serial.println("9600"); break;
    default: Serial.println("Error/Unknown"); break;
  }

  Serial.println("Humidity Calibration Value: " + String(soilSensor.humCal));
  Serial.println("Temperature Calibration Value: " + String(soilSensor.tempCal));

  if (connectivity) {
    Serial.println("Connectivity enabled");
    connectWiFi();

    Serial.println("Configuring time");
    esp_sntp_servermode_dhcp(1);
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2);
    getTime();
    Serial.print("Time: ");
    Serial.println(timeStr);

    connectMQTT();
  } else {
    Serial.println("Connectivity disabled");
  }

  // Blink LED indicating setup config. complete
  for (int i = 0; i < 5; i++) {
    // Serial.println("LED ON");
    digitalWrite(ledPin, HIGH);
    delay(100);
    // Serial.println("LED OFF");
    digitalWrite(ledPin, LOW);
    delay(100);
  }

  Serial.println("BLOOM configured!\n");
}

void loop() {

  soilSensor.read();

  if (connectivity) {

    if (!MQTTclient.connected()) {
      connectMQTT();  // This also calls connectWiFi();
    }

    if (millis() - prevTransmitt >= transmittInterval) {
      int temperature = soilSensor.temperature;
      int humidity = soilSensor.humidity;

      MQTTclient.publish("BLOOM/Temperature", String(temperature).c_str());
      Serial.print("Send temperature to MQTT: ");
      Serial.println(temperature);

      MQTTclient.publish("BLOOM/Humidity", String(humidity).c_str());
      Serial.print("Send humidity to MQTT: ");
      Serial.println(humidity);

      getTime();
      MQTTclient.publish("BLOOM/Time", String(timeStr).c_str());
      Serial.print("Send time MQTT: ");
      Serial.println(timeStr);



      prevTransmitt = millis();
    }
  } else {
    Serial.println("Temperature: " + String(soilSensor.temperature));
    Serial.println("Humidity: " + String(soilSensor.humidity));
  }



  // RUN-indication (blinking LED)
  if (runIndication) {
    if (millis() - prevBlink >= blinkInterval) {
      ledState = !ledState;
      digitalWrite(ledPin, ledState ? HIGH : LOW);
      prevBlink = millis();
    }
  }
}
