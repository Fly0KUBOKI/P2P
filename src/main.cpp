#include <Arduino.h>
#include "Wifi_SoftAP.hpp"

#if defined(ARDUINO_ARCH_ESP32)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// forward declarations for tasks
static void udpTask(void* pvParameters);
static void ultrasonicTask(void* pvParameters);

void setup() {
  Serial.begin(921600);
  while (!Serial) {
    delay(10);
  }

  WifiSoftAP::Setup();

  // コア数を判定してタスクを振り分ける。デュアルコア環境ではUDPをコア1、超音波をコア0に割り当てる。
  #if defined(portNUM_PROCESSORS)
    int coreCount = portNUM_PROCESSORS;
  #else
    int coreCount = 1;
  #endif

  if (coreCount > 1) {
    // デュアルコア: 明示的にコアにピン留め
    xTaskCreatePinnedToCore(udpTask, "udpTask", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(ultrasonicTask, "ultraTask", 2048, NULL, 1, NULL, 0);
  } else {
    // シングルコア環境でも動作するように通常タスクで生成
    xTaskCreate(udpTask, "udpTask", 4096, NULL, 1, NULL);
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

    uint8_t status;
    if (len == packetSize) {
      status = 0;
    } else {
      status = 1;
    }
    if (Serial.available() > 0) {
      int s = Serial.read();
      if (s >= 0) status = (uint8_t)s;
    }

    WifiSoftAP::udp.beginPacket(remote, port);
    WifiSoftAP::udp.write(&status, 1);
    WifiSoftAP::udp.endPacket();
  }
}

// 超音波センサ処理用のプレースホルダタスク（将来ここに処理を実装）
static void ultrasonicTask(void* pvParameters) {
  (void)pvParameters;
  for (;;) {
    // ここに超音波センサの処理を入れる予定
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

#endif
