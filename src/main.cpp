#include <Arduino.h>
#include "Wifi_SoftAP.hpp"

#if defined(ARDUINO_ARCH_ESP32)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <Wire.h>

// forward declarations for tasks
static void udpTask(void* pvParameters);
static void ultrasonicTask(void* pvParameters);

// ultrasonic timing defines (milliseconds)
#define ULTRASONIC_MEASURE_WAIT_MS 70
#define ULTRASONIC_INTERVAL_MS 30

// mutex to protect Serial access between tasks
static SemaphoreHandle_t serialMutex = NULL;

void setup() {
  Serial.begin(921600);
  while (!Serial) {
    delay(10);
  }

  // create mutex for Serial protection between tasks
  serialMutex = xSemaphoreCreateMutex();
  if (serialMutex == NULL) {
    // mutex creation failed - print once (no mutex protection available)
    Serial.println("Warning: failed to create serial mutex");
  }

  WifiSoftAP::Setup();

  // I2C 初期化（SRF02用）
  Wire.begin();

  // コア数を判定してタスクを振り分ける。デュアルコア環境ではUDPをコア1、超音波をコア0に割り当てる。
  #if defined(portNUM_PROCESSORS)
    int coreCount = portNUM_PROCESSORS;
  #else
    int coreCount = 1;
  #endif

  if (coreCount > 1) {
    // デュアルコア: 明示的にコアにピン留め
  xTaskCreatePinnedToCore(udpTask, "udpTask", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(ultrasonicTask, "ultraTask", 2048, NULL, 1, NULL, 1);
  } else {
    // シングルコア環境でも動作するように通常タスクで生成
    xTaskCreate(udpTask, "udpTask", 4096, NULL, 2, NULL);
    xTaskCreate(ultrasonicTask, "ultraTask", 2048, NULL, 1, NULL);
  }
}

void loop() {
  // メインループは空にしてRTOSタスクに処理を任せる
  vTaskDelay(pdMS_TO_TICKS(1000));
}


// UDP受信処理を行うタスク（ESP32向けにコア分離する）
static void udpTask(void* pvParameters) {
  (void)pvParameters;
  const size_t bufSize = 1500;
  static uint8_t buf[bufSize];

  for (;;) {
    // 定期的なヘルスチェック
    WifiSoftAP::Tick();

    int packetSize = WifiSoftAP::udp.parsePacket();
    if (packetSize <= 0) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    IPAddress remote = WifiSoftAP::udp.remoteIP();
    uint16_t port = WifiSoftAP::udp.remotePort();

    int len = 0;
    size_t remaining = packetSize;
    while (remaining > 0) {
      size_t toRead;
      if (remaining > (bufSize - 1)) {
        toRead = bufSize - 1;
      } else {
        toRead = remaining;
      }
      int r = WifiSoftAP::udp.read(buf, toRead);
      if (r <= 0) break;
      len += r;
      remaining -= r;

      Serial.write(buf, r);
    }

    uint8_t status = 1;
    if (len == packetSize) {
      status = 0;
    } else {
      status = 1;
    }
    // read optional status override from Serial if available
    if (serialMutex && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      if (Serial.available() > 0) {
        int s = Serial.read();
        if (s >= 0) status = (uint8_t)s;
      }
      xSemaphoreGive(serialMutex);
    }

    WifiSoftAP::udp.beginPacket(remote, port);
    WifiSoftAP::udp.write(&status, 1);
    WifiSoftAP::udp.endPacket();
  }
}

// 超音波センサ処理用のプレースホルダタスク
static void ultrasonicTask(void* pvParameters) {
  (void)pvParameters;
  const uint8_t srf02_addr = 112; // 0x70

  if (serialMutex && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    Serial.println("ultrasonicTask started");
    xSemaphoreGive(serialMutex);
  }

  for (;;) {
    // step 1: instruct sensor to read echoes (command 0x51 -> cm)
    Wire.beginTransmission(srf02_addr);
    Wire.write(byte(0x00)); // command register
    Wire.write(byte(0x51)); // measure in centimeters
    Wire.endTransmission();

  // step 2: wait for measurement to complete (datasheet: >=65ms)
  vTaskDelay(pdMS_TO_TICKS(ULTRASONIC_MEASURE_WAIT_MS));

    // step 3: point to result register (0x02)
    Wire.beginTransmission(srf02_addr);
    Wire.write(byte(0x02));
    Wire.endTransmission();

    // step 4: request 2 bytes
    Wire.requestFrom((int)srf02_addr, 2);

    // step 5: combine bytes
    int reading = 0;
    if (Wire.available() >= 2) {
      int high = Wire.read();
      reading = high << 8;
      int low = Wire.read();
      reading |= low;

      if (serialMutex && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        Serial.print(reading);
        Serial.println("cm");
        xSemaphoreGive(serialMutex);
      }
    }

  // measurement interval
  vTaskDelay(pdMS_TO_TICKS(ULTRASONIC_INTERVAL_MS));
  }
}

#endif
