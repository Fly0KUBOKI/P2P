#ifndef WIFI_SOFTAP_HPP
#define WIFI_SOFTAP_HPP

#include <WiFi.h>
#include <cstring>
#if defined(ARDUINO_ARCH_ESP32)
#include <esp32-hal-cpu.h> // for setCpuFrequencyMhz()
#endif

namespace WifiSoftAP{

  // UDPオブジェクト
  WiFiUDP udp;
  
  // 送信先の設定
  // デフォルトは全体ブロードキャスト。接続先APのネットワークに合わせて自動計算する
  IPAddress broadcastIP(255, 255, 255, 255);  // 初期値
  int udpPort = 1234;

  // ssid（WiFiのネットワーク名）
  // PC のアクセスポイント情報（ユーザー指定: SSID とパスワードが同じ）
  const char *ssid = "2025_ulysses";
  const char *pass = "2025_ulysses";

  // 設定をもとに Station 接続を初期化
  void Setup(){

  // 明示的に STA モードに設定
  WiFi.mode(WIFI_STA);

  // 開始: 接続を試みる
  WiFi.begin(ssid, pass);

  // 簡易待機ループ（非ブロッキングではないがセットアップ時のみ使用）
  unsigned long start = millis();
  const unsigned long timeout = 10000; // 10s
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeout) {
    delay(200);
  }

  // UDP通信を開始（ポートをローカルでリッスン）
  udp.begin(udpPort);
  // 接続情報が得られればブロードキャスト先を自動計算して表示
  if (WiFi.status() == WL_CONNECTED) {
    IPAddress local = WiFi.localIP();
    IPAddress gw = WiFi.gatewayIP();
    IPAddress mask = WiFi.subnetMask();
    }
  }

  // Minimal Tick(): periodic health check
  void Tick() {
    static unsigned long lastCheck = 0;
    const unsigned long interval = 5000; // ms

    unsigned long now = millis();
    if (now - lastCheck < interval) return;
    lastCheck = now;

    // If WiFi is disconnected, attempt a quick reconnect (no prints)
    if (WiFi.status() != WL_CONNECTED) {
      udp.stop();
      WiFi.disconnect();
      WiFi.begin(ssid, pass);
      // give a short window for reconnection
      unsigned long start = millis();
      const unsigned long timeout = 5000; // ms
      while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeout) {
        delay(100);
      }
      if (WiFi.status() == WL_CONNECTED) {
        udp.begin(udpPort);
      }
      return;
    }

    // Minimal memory check: if heap is very low, restart UDP to free resources
#if defined(ESP32)
    const size_t minHeap = 8 * 1024; // 8KB - conservative minimal threshold
    if (ESP.getFreeHeap() < minHeap) {
      udp.stop();
      delay(10);
      udp.begin(udpPort);
    }
#endif
  }
  
  // 送信ユーティリティ（簡易版）は削除しました。必要なら再実装してください。
}

#endif // WIFI_SOFTAP_HPP
