#include <Arduino.h>
#include "Wifi_SoftAP.hpp"
#include "LED.hpp"

#if defined(ARDUINO_ARCH_ESP32)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <Wire.h>

// --- タスクのプロトタイプ宣言 ---
static void transmitTask(void* pvParameters);
static void wifiReceiveTask(void* pvParameters);
static void uartReceiveTask(void* pvParameters);
static void ultrasonicTask(void* pvParameters);
static void ledTask(void* pvParameters);

// --- タイミング設定 (ミリ秒) ---
#define ULTRASONIC_MEASURE_WAIT_MS 65 // センサーの測定待機時間
#define ULTRASONIC_INTERVAL_MS 0   // センサーの測定周期
#define TRANSMIT_Hz 50     // 送信タスクの実行周期 (50Hz)

// --- リソース保護のためのミューテックス ---
static SemaphoreHandle_t serialMutex = NULL;
static SemaphoreHandle_t udpMutex = NULL;

// --- タスク間通信用の共有リソース ---
// 超音波センサーの値
static volatile uint8_t ultrasonicValue = 0;
// WiFi受信フラグ
static volatile bool wifiReceivedFlag = false;
// UART受信フラグ
static volatile bool uartReceivedFlag = false;
// 受信パケットの送信元情報
static IPAddress remoteClientIP;
static uint16_t remoteClientPort;


void setup() {
  Serial.begin(921600);
  while (!Serial) { delay(10); }

  serialMutex = xSemaphoreCreateMutex();
  udpMutex = xSemaphoreCreateMutex();

  WifiSoftAP::Setup();
  Wire.begin();
  setupLED(); // LED初期化

  // --- タスクの作成とコアへの割り当て ---
  // Core 1: センサー測定タスク (他から独立して動作)
  xTaskCreatePinnedToCore(ultrasonicTask, "UltraTask", 2048, NULL, 2, NULL, 1);
  // LEDタスクを作成（低優先度）
  xTaskCreatePinnedToCore(ledTask, "LEDTask", 2048, NULL, 1, NULL, 1);

  // Core 0: 通信関連タスク
  // 優先度: WiFi受信 (3) > UART受信 (2) > 送信 (1)
  // これにより、パケット到着時に受信タスクが送信タスクよりも優先して実行される
  xTaskCreatePinnedToCore(wifiReceiveTask, "WiFiRxTask", 4096, NULL, 3, NULL, 0); // 最優先
  xTaskCreatePinnedToCore(uartReceiveTask, "UartRxTask", 2048, NULL, 2, NULL, 0); // 中優先
  xTaskCreatePinnedToCore(transmitTask,    "TxTask",    4096, NULL, 1, NULL, 0); // 低優先
  
  
}

void loop() {
  vTaskDelay(portMAX_DELAY); // メインループは何もしない
}

/**
 * @brief 送信を専門に行うタスク (周期的実行)
 * - 定期的なブロードキャスト送信
 * - フラグに基づいた受信応答
 */
static void transmitTask(void* pvParameters) {
  (void)pvParameters;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(1000 / TRANSMIT_Hz);

  for (;;) {
    // 指定した周期で正確にタスクを実行するための待機
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    // --- 1. 定期的なブロードキャスト送信 ---
    if (udpMutex && xSemaphoreTake(udpMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      uint8_t broadcastPkt[2] = { 1, ultrasonicValue }; // Status 1: 定期送信
      WifiSoftAP::udp.beginPacket(WifiSoftAP::broadcastIP, WifiSoftAP::udpPort);
      WifiSoftAP::udp.write(broadcastPkt, 2);
      WifiSoftAP::udp.endPacket();
      xSemaphoreGive(udpMutex);
    }

    // --- 2. WiFi受信フラグを確認し、応答を送信 ---
    if (wifiReceivedFlag) {
      wifiReceivedFlag = false; // すぐにフラグをリセット
      if (udpMutex && xSemaphoreTake(udpMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        uint8_t replyPkt[2] = { 0, ultrasonicValue }; // Status 0: 応答
        WifiSoftAP::udp.beginPacket(remoteClientIP, remoteClientPort);
        WifiSoftAP::udp.write(replyPkt, 2);
        WifiSoftAP::udp.endPacket();
        xSemaphoreGive(udpMutex);
      }
    }

    // --- 3. UART受信フラグを確認し、処理 ---
    if (uartReceivedFlag) {
      uartReceivedFlag = false; // すぐにフラグをリセット
      if (serialMutex && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        Serial.println("UART-IN"); // UART受信時の処理をここに記述
        xSemaphoreGive(serialMutex);
      }
    }
  }
}

/**
 * @brief WiFi-UDPの受信を専門に行うタスク
 * - パケットを受信したら、送信元情報を保存し、フラグを立てる
 */
static void wifiReceiveTask(void* pvParameters) {
  (void)pvParameters;
  for (;;) {
    int packetSize = WifiSoftAP::udp.parsePacket();
    if (packetSize > 0) {
      // 応答のために送信元IPとポートをグローバル変数に保存
      remoteClientIP = WifiSoftAP::udp.remoteIP();
      remoteClientPort = WifiSoftAP::udp.remotePort();
      
      // 受信データをバッファに読み込む
      uint8_t tempBuf[packetSize];
      WifiSoftAP::udp.read(tempBuf, packetSize);

      // 受信データをそのままシリアルに出力
      if (serialMutex && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        Serial.write(tempBuf, packetSize);
        xSemaphoreGive(serialMutex);
      }
      // 送信タスクに処理を依頼するためにフラグを立てる
      wifiReceivedFlag = true;
    }
    vTaskDelay(pdMS_TO_TICKS(1)); // CPUを過剰に占有しないための短い遅延
  }
}

/**
 * @brief UART (シリアル) の受信を専門に行うタスク
 * - データを受信したら、フラグを立てる
 */
static void uartReceiveTask(void* pvParameters) {
  (void)pvParameters;
  for (;;) {
    if (Serial.available() > 0) {
      while(Serial.available() > 0) {
        Serial.read(); // バッファを空にする
      }
      uartReceivedFlag = true;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}


/**
 * @brief 超音波センサーの測定のみを行うタスク (Core 1で独立動作)
 */
static void ultrasonicTask(void* pvParameters) {
  (void)pvParameters;
  for (;;) {
    Wire.beginTransmission(112); // SRF02 Address 0x70
    Wire.write(byte(0x00));      // Command Register
    Wire.write(byte(0x51));      // Measure in cm
    Wire.endTransmission();

    vTaskDelay(pdMS_TO_TICKS(ULTRASONIC_MEASURE_WAIT_MS));

    Wire.beginTransmission(112);
    Wire.write(byte(0x02));      // Result Register
    Wire.endTransmission();

    Wire.requestFrom(112, 2);
    if (Wire.available() >= 2) {
      int high = Wire.read();
      int low = Wire.read();
      int reading = (high << 8) | low;

      if (reading > 255) reading = 255;
      ultrasonicValue = (uint8_t)reading;
    }
    vTaskDelay(pdMS_TO_TICKS(ULTRASONIC_INTERVAL_MS));
  }
}

/**
 * @brief LEDの点滅制御を行うタスク
 */
static void ledTask(void* pvParameters) {
  (void)pvParameters;
  for (;;) {
    loopLED(); // LED点滅処理を実行
  }
}

#endif

