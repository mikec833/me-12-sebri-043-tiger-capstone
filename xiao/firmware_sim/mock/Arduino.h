#pragma once
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <string>
#include <deque>
#include <vector>
using std::isfinite;
#define PI 3.14159265358979f
enum { D0, D1, D2, D3, D4, D5, D6, D7, D8, D9, D10, LED_RED = 26 };
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define CHANGE 2
#define AR_INTERNAL_3_0 0
template <class T> T min(T a, T b) { return a < b ? a : b; }
template <class T> T max(T a, T b) { return a > b ? a : b; }
template <class T, class L, class H> T constrain(T v, L lo, H hi) { return v < lo ? lo : (v > hi ? hi : v); }
extern uint32_t g_millis;
inline uint32_t millis() { return g_millis; }
inline uint32_t micros() { return g_millis * 1000u; }
inline void delay(uint32_t) {}
inline int digitalRead(int) { return 0; }
inline void digitalWrite(int, int) {}
inline void pinMode(int, int) {}
inline int analogRead(int) { return 2650; }
inline void analogReference(int) {}
inline void analogReadResolution(int) {}
inline int digitalPinToInterrupt(int p) { return p; }
inline void attachInterrupt(int, void (*)(), int) {}
inline void taskENTER_CRITICAL() {}
inline void taskEXIT_CRITICAL() {}
class String {
 public:
  std::string s;
  String() {}
  String(const char *c) : s(c) {}
  String(const std::string &x) : s(x) {}
  size_t length() const { return s.size(); }
  const char *c_str() const { return s.c_str(); }
  char operator[](size_t i) const { return i < s.size() ? s[i] : 0; }
  bool operator==(const char *o) const { return s == o; }
  void trim() { size_t a = s.find_first_not_of(" \t\r\n"); size_t b = s.find_last_not_of(" \t\r\n");
                s = a == std::string::npos ? "" : s.substr(a, b - a + 1); }
};
// FreeRTOS queue
typedef unsigned UBaseType_t;
struct Queue { std::deque<std::vector<char>> items; size_t depth, size; };
typedef Queue *QueueHandle_t;
#define pdPASS 1
#define pdFAIL 0
inline QueueHandle_t xQueueCreate(UBaseType_t d, UBaseType_t sz) { auto q = new Queue; q->depth = d; q->size = sz; return q; }
inline int xQueueSend(QueueHandle_t q, const void *item, int) {
  if (q->items.size() >= q->depth) return pdFAIL;
  q->items.emplace_back((const char *)item, (const char *)item + q->size); return pdPASS; }
inline int xQueueReceive(QueueHandle_t q, void *item, int) {
  if (q->items.empty()) return pdFAIL;
  memcpy(item, q->items.front().data(), q->size); q->items.pop_front(); return pdPASS; }
// USB serial
struct MockSerial {
  bool open = true; std::deque<char> in; std::string out;
  void begin(int) {}
  explicit operator bool() const { return open; }
  int available() { return (int)in.size(); }
  int read() { char c = in.front(); in.pop_front(); return (unsigned char)c; }
  int availableForWrite() { return open ? 4096 : 0; }
  size_t write(const char *b, size_t n) { out.append(b, n); return n; }
  void feed(const std::string &x) { for (char c : x) in.push_back(c); }
};
extern MockSerial Serial;
