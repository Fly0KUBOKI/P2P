
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
// periodic broadcast interval (ms)
#define BROADCAST_INTERVAL_MS 50
// set to 1 to enable ultrasonic task, 0 to disable

// mutex to protect Serial access between tasks
static SemaphoreHandle_t serialMutex = NULL;
// latest ultrasonic reading (cm) published as a single uint8_t
static volatile uint8_t ultrasonicValue = 0;
// mutex to protect UDP send operations
static SemaphoreHandle_t udpMutex = NULL;

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

  // create mutex for UDP protection between tasks
  udpMutex = xSemaphoreCreateMutex();
  if (udpMutex == NULL) {
    Serial.println("Warning: failed to create udp mutex");
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

    // periodic broadcast even when no incoming packet
    static unsigned long lastBroadcast = 0;
    unsigned long now = millis();
    if (now - lastBroadcast >= BROADCAST_INTERVAL_MS) {
      lastBroadcast = now;
      uint8_t status_b = 1; // default status for periodic broadcast
      uint8_t pkt[2] = { status_b, ultrasonicValue };
      //  Serial.print(ultrasonicValue);
      //  Serial.println("cm");
      if (udpMutex && xSemaphoreTake(udpMutex, pdMS_TO_TICKS(50)) == pdTRUE) {

        WifiSoftAP::udp.beginPacket(WifiSoftAP::broadcastIP, WifiSoftAP::udpPort);
        WifiSoftAP::udp.write(pkt, 2);
        WifiSoftAP::udp.endPacket();
        xSemaphoreGive(udpMutex);
      }
    }

    int packetSize = WifiSoftAP::udp.parsePacket();
    if (packetSize <= 0) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    IPAddress remote = WifiSoftAP::udp.remoteIP();
    uint16_t port = WifiSoftAP::udp.remotePort();

    // 受信があればログ出力 (remote/port を取得してから表示)
    if (serialMutex && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      Serial.print("Received packet size="); Serial.print(packetSize);
      Serial.print(" from "); Serial.print(remote);
      Serial.print(":"); Serial.println(port);
      xSemaphoreGive(serialMutex);
    }

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
      // データは buf に蓄積される（len が増加）
      // 出力は後でまとめて行う
    }

    // ここでは受信バイナリをそのままシリアルに出力する（生のバイト列）
    if (len > 0) {
      if (serialMutex && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        Serial.write(buf, len);
        xSemaphoreGive(serialMutex);
      }
    }

    uint8_t status = 1;
    // read optional status override from Serial if available
    if (serialMutex && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      if (Serial.available() > 0) {
        int s = Serial.read();
        if (s >= 0) status = (uint8_t)s;
      }
      xSemaphoreGive(serialMutex);
    }

    if (udpMutex && xSemaphoreTake(udpMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      WifiSoftAP::udp.beginPacket(remote, port);
      // send two bytes: [status, ultrasonicValue]
      uint8_t packetOut[2];
      packetOut[0] = status;
      packetOut[1] = ultrasonicValue;
      WifiSoftAP::udp.write(packetOut, 2);
      WifiSoftAP::udp.endPacket();
      xSemaphoreGive(udpMutex);
    }
  }
}

// 超音波センサ処理用のプレースホルダタスク
static void ultrasonicTask(void* pvParameters) {
  (void)pvParameters;
  const uint8_t srf02_addr = 112; // 0x70

  if (serialMutex && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    //Serial.println("ultrasonicTask started");
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

      // clamp reading to 0..255 and store as latest ultrasonicValue
      if (reading < 0) reading = 0;
      if (reading > 255) reading = 255;
      ultrasonicValue = (uint8_t)reading;
      // Serial.print("Ultrasonic: ");
      // Serial.print(reading);
      // Serial.println("cm");

      // immediate send: status=0, ultrasonicValue
      uint8_t immediatePkt[2] = { 0, ultrasonicValue };
      if (udpMutex && xSemaphoreTake(udpMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        WifiSoftAP::udp.beginPacket(WifiSoftAP::broadcastIP, WifiSoftAP::udpPort);
        WifiSoftAP::udp.write(immediatePkt, 2);
        WifiSoftAP::udp.endPacket();
        xSemaphoreGive(udpMutex);
      }

      if (serialMutex && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        xSemaphoreGive(serialMutex);
      }
    }

  }
}

#endif
