/*
 * Copyright (c) 2025 Kenneth Paulsen
 * Licensed under the MIT License.
 * See LICENSE file in the project root for full license information.
 */
 
 #include <HardwareSerial.h>

// RS485/Modbus
HardwareSerial RS485Serial(1);
const int RS485_DIR = 4;
const int RS485_RX = 20;
const int RS485_TX = 21;

// Pins
const int ledPin = 1;
const int GPIO2 = 2;
const int loadSwitchEN = 3;
const int motorDriverIN1 = 5;
const int motorDriverIN2 = 6;
const int GPIO7 = 7;
const int GPIO10 = 10;

// RUN-indication (blinking LED)
unsigned long prevBlink = 0;
const int blinkInterval = 2000;
bool ledState = false;

// Sensor and motor driver power states
bool motorDriverPowered = true;
bool sensorPowered = false;


bool sensorPower(bool powerState) {
  // Datasheet: https://www.ti.com/lit/ds/symlink/tps22810.pdf
  // powers TPS22810DRVR load switch
  digitalWrite(loadSwitchEN, powerState);
  if (powerState == 1) {
    delayMicroseconds(400);
    Serial.println("Sensor ON");
    return sensorPowered = true;

  } else {
    delayMicroseconds(4);
    Serial.println("Sensor OFF");
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
    Serial.println("Motor driver put to sleep");
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
  uint16_t calculateCRC(uint8_t *data, uint8_t length) {
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

  bool readModbusResponse(uint16_t *values, uint8_t expectedRegs) {
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
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  RS485Serial.begin(9600, SERIAL_8N1, RS485_RX, RS485_TX);
  while (!RS485Serial) {
    ;
  }

  for (int i = 1; i <= 10; i++) {
    if (i == 8 || i == 9) continue;  // leave 8 and 9 floating
    pinMode(i, OUTPUT);
    digitalWrite(i, LOW);
  }

  sensorPower(0);
  motorDriverState(0);

  soilSensor.writeCalibration(0.0, 0.0);
  soilSensor.readCalData();
  Serial.print("Humidity Calibration Value: " + String(soilSensor.humCal) + "\t");
  Serial.print("Temperature Calibration Value: " + String(soilSensor.tempCal) + "\t");
  Serial.print("Device Address: " + String(soilSensor.deviceAddr) + "\t");
  Serial.println("Baud rate: " + String(soilSensor.baud));

  // Blink LED indicating setup config. complete
  for (int i = 0; i < 5; i++) {
    // Serial.println("LED ON");
    digitalWrite(ledPin, HIGH);
    delay(100);
    // Serial.println("LED OFF");
    digitalWrite(ledPin, LOW);
    delay(100);
  }
}

void loop() {

  soilSensor.read();
  Serial.print("Temperature: " + String(soilSensor.temperature) + "\t");
  Serial.println("Humidity: " + String(soilSensor.humidity));



  delay(1000);

  // RUN-indication (blinking LED)
  if (millis() - prevBlink >= blinkInterval) {
    ledState = !ledState;
    digitalWrite(ledPin, ledState ? HIGH : LOW);
    prevBlink = millis();
    Serial.println("Blink");
  }
}
