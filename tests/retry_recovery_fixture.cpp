// Host platform substitutes for test_retry_recovery.py.
#include <cassert>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <string>
#include <utility>

template<typename... Args> void log(const char *, Args...) {}
#define ESP_LOGW(...) log(__VA_ARGS__)
#define ESP_LOGD(...) log(__VA_ARGS__)
#define ESP_LOGE(...) log(__VA_ARGS__)
static const char *TAG = "test";
static uint32_t now;
uint32_t millis() { return now; }
namespace espbt {
enum class ClientState { IDLE, CONNECTING, DISCONNECTING };
enum class ConnectionType { V3_WITH_CACHE, V3_WITHOUT_CACHE };
}
struct Client {
  espbt::ClientState state_{espbt::ClientState::IDLE};
  unsigned connects{0}, disconnects{0};
  auto state() const { return state_; }
  void set_connection_type(espbt::ConnectionType) {}
  void connect() { connects++; state_ = espbt::ClientState::CONNECTING; }
  void disconnect() {
    disconnects++;
    // ESPHome cannot actively disconnect until CONNECT_EVT supplies a conn_id.
    // A disconnect requested while CONNECTING is only remembered by the real
    // client, whose state stays CONNECTING until the stack reports completion.
    if (state_ != espbt::ClientState::CONNECTING)
      state_ = espbt::ClientState::DISCONNECTING;
  }
};
class YaleXSBLE {
 public:
  // PRODUCTION_DECLARATIONS
  struct Timer { uint32_t due; std::function<void()> callback; };
  std::map<std::string, Timer> timers;
  Client client;
  void *lock_entity_{nullptr};
  unsigned failures{0};
  bool loop_enabled{true};
  Client *parent() { return &client; }
  void enable_loop() { loop_enabled = true; }
  void disable_loop() { loop_enabled = false; }
  void status_set_warning(const char *) { failures++; }
  void publish_lock_state_() {}
  bool cached_gatt_handles_valid_() { return false; }
  void reset_session_state_() {
    active_step_ = StepType::NONE;
    pending_response_channel_ = ResponseChannel::NONE;
  }
  StepType active_step_{StepType::NONE};
  ResponseChannel pending_response_channel_{ResponseChannel::NONE};
  void cancel_timeout(const char *name) { timers.erase(name); }
  void set_timeout(const char *name, uint32_t delay, std::function<void()> callback) {
    timers.insert_or_assign(name, Timer{now + delay, std::move(callback)});
  }
  void tick() {
    // Like ESPHome, run scheduled callbacks and then the component loop.
    for (;;) {
      auto due = timers.end();
      for (auto it = timers.begin(); it != timers.end(); ++it)
        if (uint32_t(now - it->second.due) < 0x80000000U) { due = it; break; }
      if (due == timers.end()) break;
      auto callback = std::move(due->second.callback);
      timers.erase(due);
      callback();
    }
    if (loop_enabled) loop();
  }
  void advance(uint32_t duration) {
    for (uint32_t i = 0; i < duration; ++i) { now++; tick(); }
  }
  void begin() {
    operation_timeout_ms_ = 100;
    connect_timeout_ms_ = 10000;
    operation_retries_ = 1;
    operation_queue_.push_back(OperationType::UPDATE);
    tick();
    assert(client.connects == 1);
  }
};
// PRODUCTION_METHODS
int main(int argc, char **argv) {
  assert(argc == 2);
  const std::string test = argv[1];
  YaleXSBLE yale;
  now = test == "wrap" ? UINT32_MAX - 50 : 1000;
  yale.begin();
  if (test == "zero") yale.operation_retries_ = 0;
  yale.advance(101);
  assert(yale.client.disconnects >= 1);
  if (test == "zero") {
    assert(yale.failures == 1);
    assert(yale.current_operation_ == YaleXSBLE::OperationType::NONE);
    yale.advance(1000);
    assert(yale.failures == 1);
    assert(yale.client.connects == 1);
    return 0;
  }
  assert(yale.timers.count("retry") == 1);
  const auto due = yale.timers.at("retry").due;
  const auto disconnects = yale.client.disconnects;
  yale.advance(100);
  if (test == "duplicate") {
    yale.retry_current_operation_("late connect failure");
    yale.connect_current_attempt_();
    assert(yale.client.connects == 1);
  }
  assert(yale.timers.at("retry").due == due);
  assert(yale.client.disconnects == disconnects);
  if (test == "connecting") {
    yale.operation_timeout_ms_ = 1000;
    yale.advance(150);
    assert(yale.current_attempt_ == 1);
    assert(yale.client.connects == 1);
    assert(yale.timers.count("retry_after_disconnect") == 1);
    yale.advance(250);
    assert(yale.current_attempt_ == 1);
    assert(yale.client.connects == 1);
    yale.client.state_ = espbt::ClientState::IDLE;
    yale.advance(250);
    assert(yale.current_attempt_ == 2);
    assert(yale.client.connects == 2);
    return 0;
  }
  if (test != "disconnect") yale.client.state_ = espbt::ClientState::IDLE;
  yale.advance(150);
  if (test == "disconnect") {
    assert(yale.failures == 1);
    assert(yale.current_attempt_ == 1);
    assert(yale.client.connects == 1);
    assert(yale.current_operation_ == YaleXSBLE::OperationType::NONE);
  } else {
    assert(yale.current_attempt_ == 2);
    assert(yale.client.connects == 2);
  }
  yale.advance(101);
  assert(yale.failures == 1);
  assert(yale.current_operation_ == YaleXSBLE::OperationType::NONE);
  assert(yale.timers.count("retry") == 0);
  assert(yale.timers.count("retry_after_disconnect") == 0);
  // Once the transport becomes available, a new operation can run normally.
  yale.client.state_ = espbt::ClientState::IDLE;
  yale.operation_queue_.push_back(YaleXSBLE::OperationType::UPDATE);
  yale.enable_loop();
  yale.tick();
  assert(yale.current_attempt_ == 1);
  assert(yale.current_operation_ == YaleXSBLE::OperationType::UPDATE);
}
