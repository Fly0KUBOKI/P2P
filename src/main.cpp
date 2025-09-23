#include <Arduino.h>
#include "Wifi_Setting.hpp"
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
static void wifiHealthTask(void* pvParameters);

// --- タイミング設定 (ミリ秒) ---
#define ULTRASONIC_MEASURE_WAIT_MS 65 // センサーの測定待機時間
#define ULTRASONIC_INTERVAL_MS 0   // センサーの測定周期
#define TRANSMIT_Hz 50     // 送信タスクの実行周期 (50Hz)

// --- GPIO設定 ---
#define WIFI_STATUS_LED_PIN 2  // WiFi接続状態表示用LED

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

  // GPIO2をWiFi状態表示LED用に初期化
  pinMode(WIFI_STATUS_LED_PIN, OUTPUT);
  digitalWrite(WIFI_STATUS_LED_PIN, LOW); // 初期状態：消灯

  WifiSetting::Setup();
  
  // WiFi接続成功時にLEDを点灯
  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(WIFI_STATUS_LED_PIN, HIGH);
  }
  
  Wire.begin();
  setupLED(); // LED初期化

  // --- タスクの作成とコアへの割り当て ---
  // Core 1: センサー測定タスク (他から独立して動作)
  xTaskCreatePinnedToCore(ultrasonicTask, "UltraTask", 2048, NULL, 2, NULL, 1);
  // LEDタスクを作成（低優先度）
  xTaskCreatePinnedToCore(ledTask, "LEDTask", 2048, NULL, 1, NULL, 1);

  // Core 0: 通信関連タスク
  // 優先度: WiFi受信 (3) > UART受信 (2) > 送信 (1) > WiFiヘルスチェック (0)
  // これにより、パケット到着時に受信タスクが送信タスクよりも優先して実行される
  xTaskCreatePinnedToCore(wifiReceiveTask, "WiFiRxTask", 4096, NULL, 3, NULL, 0); // 最優先
  xTaskCreatePinnedToCore(uartReceiveTask, "UartRxTask", 2048, NULL, 2, NULL, 0); // 中優先
  xTaskCreatePinnedToCore(transmitTask,    "TxTask",    4096, NULL, 1, NULL, 0); // 低優先
  xTaskCreatePinnedToCore(wifiHealthTask,  "HealthTask", 2048, NULL, 0, NULL, 0); // 最低優先
  
  
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
      WifiSetting::udp.beginPacket(WifiSetting::broadcastIP, WifiSetting::udpPort);
      WifiSetting::udp.write(broadcastPkt, 2);
      WifiSetting::udp.endPacket();
      xSemaphoreGive(udpMutex);
    }

    // --- 2. WiFi受信フラグを確認し、応答を送信 ---
    if (wifiReceivedFlag) {
      wifiReceivedFlag = false; // すぐにフラグをリセット
      if (udpMutex && xSemaphoreTake(udpMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        uint8_t replyPkt[2] = { 0, ultrasonicValue }; // Status 0: 応答
        WifiSetting::udp.beginPacket(remoteClientIP, remoteClientPort);
        WifiSetting::udp.write(replyPkt, 2);
        WifiSetting::udp.endPacket();
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
    int packetSize = WifiSetting::udp.parsePacket();
    if (packetSize > 0) {
      // 応答のために送信元IPとポートをグローバル変数に保存
      remoteClientIP = WifiSetting::udp.remoteIP();
      remoteClientPort = WifiSetting::udp.remotePort();

      // 受信データをバッファに読み込む
      uint8_t tempBuf[packetSize];
      WifiSetting::udp.read(tempBuf, packetSize);

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

/**
 * @brief WiFi接続とメモリの簡易ヘルスチェックを行うタスク
 */
static void wifiHealthTask(void* pvParameters) {
  (void)pvParameters;
  for (;;) {
    // 5秒間隔でヘルスチェックを実行
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    // WiFi接続状態をチェック
    if (WiFi.status() != WL_CONNECTED) {
      // WiFi切断時：LEDを消灯
      digitalWrite(WIFI_STATUS_LED_PIN, LOW);
      
      // UDP停止
      if (udpMutex && xSemaphoreTake(udpMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        WifiSetting::udp.stop();
        xSemaphoreGive(udpMutex);
      }
      
      // WiFi再接続を試行
      WiFi.disconnect();
      WiFi.begin("2025_ulysses", "2025_ulysses"); // 簡易版：直接指定
      
      // 5秒間接続を待機
      unsigned long start = millis();
      const unsigned long timeout = 5000;
      while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeout) {
        vTaskDelay(pdMS_TO_TICKS(100));
      }
      
      // 接続成功時にUDP再開とLED点灯
      if (WiFi.status() == WL_CONNECTED) {
        digitalWrite(WIFI_STATUS_LED_PIN, HIGH); // WiFi接続成功：LEDを点灯
        if (udpMutex && xSemaphoreTake(udpMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
          WifiSetting::udp.begin(1234);
          xSemaphoreGive(udpMutex);
        }
      }
    } else {
      // WiFi接続中：LEDを点灯状態に保つ
      digitalWrite(WIFI_STATUS_LED_PIN, HIGH);
    }
    
    // メモリチェック（簡易版）
    #if defined(ESP32)
      const size_t minHeap = 8 * 1024; // 8KB
      if (ESP.getFreeHeap() < minHeap) {
        // UDP再起動でメモリ解放を試行
        if (udpMutex && xSemaphoreTake(udpMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
          WifiSetting::udp.stop();
          vTaskDelay(pdMS_TO_TICKS(10));
          WifiSetting::udp.begin(1234);
          xSemaphoreGive(udpMutex);
        }
      }
    #endif
  }
}

#endif

