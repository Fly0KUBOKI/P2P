#ifndef WIFI_SOFTAP_HPP
#define WIFI_SOFTAP_HPP

#include <WiFi.h>
#include <cstring>
#if defined(ARDUINO_ARCH_ESP32)
#include <esp32-hal-cpu.h> // for setCpuFrequencyMhz()
#endif

// --- 静的IPアドレス設定 ---
IPAddress local_IP(192, 168, 137, 123); // 固定したいIPアドレス
IPAddress gateway(192, 168, 137, 1);    // ゲートウェイのIPアドレス
IPAddress subnet(255, 255, 255, 0);   // サブネットマスク
IPAddress primaryDNS(8, 8, 8, 8);     // (オプション) プライマリDNS
IPAddress secondaryDNS(8, 8, 4, 4);   // (オプション) セカンダリDNS

namespace WifiSetting {

  // UDPオブジェクト
  WiFiUDP udp;
  
  // 送信先の設定
  // デフォルトは全体ブロードキャスト。接続先APのネットワークに合わせて自動計算する
  IPAddress broadcastIP(255, 255, 255, 255);  // 初期値
  int udpPort = 1234;

  // ssid（WiFiのネットワーク名）
  // PC のアクセスポイント情報
  const char *ssid = "2025_ulysses";
  const char *pass = "2025_ulysses";

  // 設定をもとに Station 接続を初期化
  void Setup(){

      // WiFi.begin() を呼び出す前にIPアドレスを設定します
    if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
      Serial.println("STA Failed to configure");
    }

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
}

#endif // WIFI_SOFTAP_HPP
