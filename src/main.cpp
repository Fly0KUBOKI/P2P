#include <Arduino.h>
#include "Wifi_SoftAP.hpp"

#if defined(ARDUINO_ARCH_ESP32)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <Wire.h>

// タスクのプロトタイプ宣言
static void udpTask(void* pvParameters);
static void ultrasonicTask(void* pvParameters);

// タイミング設定 (ミリ秒)
#define ULTRASONIC_MEASURE_WAIT_MS 70 // センサーの測定待機時間

// リソース保護のためのミューテックス（シンプル化のためシリアルミューテックスは削除）
static SemaphoreHandle_t udpMutex = NULL;

// 超音波タスクのハンドル（通知で起動するために保持）
static TaskHandle_t ultrasonicHandle = NULL;

// タスク間で共有する変数 (超音波センサーの値)
// volatileキーワードは、複数のタスクからアクセスされる変数に必須です
static volatile uint8_t ultrasonicValue = 0;

void setup() {
  Serial.begin(921600);
  while (!Serial) {
    delay(10);
  }

  // WiFiとUDPの初期化
  WifiSoftAP::Setup();

  // SRF02センサー用のI2Cを初期化
  Wire.begin();

  // CPUコア数を判定してタスクを割り当て
  #if defined(portNUM_PROCESSORS)
    int coreCount = portNUM_PROCESSORS;
  #else
    int coreCount = 1;
  #endif

  if (coreCount > 1) {
    // デュアルコアの場合：パフォーマンス向上のためタスクをコアに固定
    xTaskCreatePinnedToCore(udpTask, "udpTask", 4096, NULL, 2, NULL, 0); // ネットワークタスクをコア0に
    xTaskCreatePinnedToCore(ultrasonicTask, "ultraTask", 2048, NULL, 1, &ultrasonicHandle, 1); // センサータスクをコア1に
  } else {
    // シングルコアの場合：コアを固定せずにタスクを作成
    xTaskCreate(udpTask, "udpTask", 4096, NULL, 2, NULL);
    xTaskCreate(ultrasonicTask, "ultraTask", 2048, NULL, 1, &ultrasonicHandle);
  }
}

void loop() {
  // メインループは空。処理はFreeRTOSのタスクスケジューラに任せる
  vTaskDelay(pdMS_TO_TICKS(1000));
}

/**
 * @brief 全てのUDP通信（送受信）を一元管理するタスク
 * このタスクのみが WifiSoftAP::udp オブジェクトにアクセスする
 */
static void udpTask(void* pvParameters) {
  (void)pvParameters;
  const size_t bufSize = 1500;
  static uint8_t buf[bufSize];

  for (;;) {
    // --- 簡潔な受信処理のみ ---
    int packetSize = WifiSoftAP::udp.parsePacket();
    if (packetSize > 0) {
      // 受信データを読み込む（バッファをクリア）
      int len = WifiSoftAP::udp.read(buf, bufSize);

      // 受信をトリガーに超音波測定タスクを起動（通知）
      if (ultrasonicHandle != NULL) {
        xTaskNotifyGive(ultrasonicHandle);
      }
    }

    // 短い遅延でCPUを他タスクに譲る
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

/**
 * @brief 超音波センサーの測定を行うタスク (簡易版)
 * 元の詳細実装を簡略化してリンクエラーを解消します。
 */
static void ultrasonicTask(void* pvParameters) {
  (void)pvParameters;
  const uint8_t srf02_addr = 112; // 0x70

  for (;;) {
    // 通知で起床する（受信がトリガーされたらすぐ実行）。タイムアウトで定期測定も行う。
    uint32_t notified = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));

    // 測定開始 (0x51: センチメートル)
    Wire.beginTransmission(srf02_addr);
    Wire.write(byte(0x00));
    Wire.write(byte(0x51));
    Wire.endTransmission();

    // 測定待ち
    vTaskDelay(pdMS_TO_TICKS(ULTRASONIC_MEASURE_WAIT_MS));

    // 測定値を要求して読み取り
    Wire.beginTransmission(srf02_addr);
    Wire.write(byte(0x02));
    Wire.endTransmission();

    Wire.requestFrom((int)srf02_addr, 2);
    if (Wire.available() >= 2) {
      int high = Wire.read();
      int low = Wire.read();
      int reading = (high << 8) | low;
      if (reading < 0) reading = 0;
      if (reading > 255) reading = 255;
      ultrasonicValue = (uint8_t)reading;
    }

    // 通知トリガーなら簡潔なログを出す
    if (notified) {
      Serial.print("UDP->ultrasonic: ");
      Serial.println(ultrasonicValue);
    }
  }
}

#endif

