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
  IPAddress broadcastIP(192, 168, 30, 255);  // ブロードキャスト
  int udpPort = 1234;

  // ssid（WiFiのネットワーク名）
  const char *ssid = "TEST_WIFI";

  // pass（WiFiのパスワード。NULLにするとパスワードなしになる）
  const char *pass = nullptr;  // パスワードなしの場合

  // ip（このデバイスが持つIPアドレス）
  const IPAddress ip(192, 168, 30, 3);

  // subnet（サブネットマスク。ネットワークの範囲を指定）
  const IPAddress subnet(255, 255, 255, 0);

  //設定をもとにSoftAPを初期化
  void Setup(){

  // 明示的にAPモードに設定
  WiFi.mode(WIFI_AP);

#if defined(ARDUINO_ARCH_ESP32)
  // 安定性優先: スリープを無効化（消費電力は上がる）
  // Arduino core for ESP32 は WiFi.setSleep(false) をサポート
  WiFi.setSleep(false);
  // 可能であれば CPU 周波数を最大に上げて WiFi スタックの処理を安定化
  #if defined(CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ)
    setCpuFrequencyMhz(CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ);
  #else
    setCpuFrequencyMhz(240);
  #endif
  
#endif

  WiFi.softAPConfig(ip, ip, subnet);
  WiFi.softAP(ssid, pass, 1, 0, 4);  // 最大4台まで接続可能

  // UDP通信を開始
  udp.begin(udpPort);
  }

  // SoftAPの状態を表示
  void PrintStatus(){
    // Serial 出力は無効化しているため、必要なら UDP で状態通知する実装に置き換えてください。
  }

  // 定期的なヘルスチェック。loop から定期的に呼ぶことを想定しています。
  // 目的: SoftAP が落ちている/無効になっている場合に再初期化して安定化を図る
  void Tick() {
    static unsigned long lastHealthMillis = 0;
    const unsigned long healthInterval = 5000; // ms

    unsigned long now = millis();
    if (now - lastHealthMillis < healthInterval) return;
    lastHealthMillis = now;

    // SoftAP の IP を確認
    IPAddress apip = WiFi.softAPIP();
    if (apip == IPAddress(0,0,0,0)) {
      // SoftAP が無効化されている可能性があるため再初期化
      udp.stop();
      WiFi.softAPdisconnect(true);
      delay(100);
      WiFi.mode(WIFI_AP);
      WiFi.softAPConfig(ip, ip, subnet);
      WiFi.softAP(ssid, pass, 1, 0, 4);
      delay(50);
      udp.begin(udpPort);
      return;
    }

    // 過度なメモリ不足であれば警告的に再初期化を試みる（閾値は環境に合わせて調整）
#if defined(ESP32)
    const size_t minHeap = 16 * 1024; // 16KB
    if (ESP.getFreeHeap() < minHeap) {
      // 軽微な回復処理: UDP を再起動することでリソースが解放される場合がある
      udp.stop();
      delay(20);
      udp.begin(udpPort);
    }
#endif
  }
  
  // 任意の数のデータを送信(ラベルなし/CSV保存可能)
  template<typename... Args>
  void SendData(Args... args) {
    udp.beginPacket(broadcastIP, udpPort);
    
    // 引数をString配列に変換
    uint8_t length = sizeof...(args);
    String ary[] = { String(args)... };
    
    // for文でカンマ区切りで送信
    for(uint8_t i = 0; i < length; i++) {
  // if(i > 0) udp.print(",");  // 最初以外はカンマを付ける
  // udp.print(ary[i]);
    }
    
  // udp.println();  // 最後に改行
    udp.endPacket();
  }
  
  // 任意の数のデータを送信(ラベル付き)
  template<typename... Args>
  void SendData(const char* label, Args... args) {
    udp.beginPacket(broadcastIP, udpPort);
  // udp.print(label);
  // udp.print(":");
    
    // 引数をString配列に変換
    uint8_t length = sizeof...(args);
    String ary[] = { String(args)... };
    
    // for文でカンマ区切りで送信(7桁固定長)
    for(uint8_t i = 0; i < length; i++) {
  // if(i > 0) udp.print(",");  // 最初以外はカンマを付ける
      
      // 7桁固定長にフォーマット(右寄せ、左側スペース詰め)
      String formatted = ary[i];
      while(formatted.length() < 7) {
        formatted = " " + formatted;  // 左側にスペースを追加
      }
      if(formatted.length() > 7) {
        formatted = formatted.substring(0, 7);  // 7桁を超える場合は切り詰め
      }
      
  // udp.print(formatted);
    }
    
  // udp.println();  // 最後に改行
    udp.endPacket();
  }

  // 生の文字列をそのまま送信(一行をそのまま送る用途向け)
  // 長いデータは安全なチャンクサイズに分割して複数パケットで送信
  void SendRaw(const char* data) {
    const size_t maxChunk = 1200; // より安全なUDPチャンクサイズに下げる
    size_t len = strlen(data);
    const char* ptr = data;
    size_t sent = 0;

    if (len == 0) {
      udp.beginPacket(broadcastIP, udpPort);
  // udp.println();
      udp.endPacket();
      return;
    }

    while (sent < len) {
      size_t chunk = (len - sent) > maxChunk ? maxChunk : (len - sent);
      udp.beginPacket(broadcastIP, udpPort);
      udp.write((const uint8_t*)(ptr + sent), chunk);
      // 最後のチャンクのあとに改行を付ける
      if (sent + chunk >= len) {
  // udp.println();
      }
      udp.endPacket();
      sent += chunk;
      // 連続送信でWiFiスタックに負荷をかけないよう短い待ちを入れる
      // delay(1) と yield() でタスク切り替えを許可
      delay(1);
      yield();
    }
  }

  void SendRaw(const String& data) {
    SendRaw(data.c_str());
  }
}

#endif // WIFI_SOFTAP_HPP
