// Host platform substitutes for test_retry_recovery.py.
#include <cassert>
#include <algorithm>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <string>
#include <utility>
#include <vector>

template<typename... Args> void log(const char *, Args...) {}
#define ESP_LOGW(...) log(__VA_ARGS__)
#define ESP_LOGD(...) log(__VA_ARGS__)
#define ESP_LOGE(...) log(__VA_ARGS__)
static const char *TAG = "test";
static uint32_t now;
uint32_t millis() { return now; }
namespace espbt {
enum class ClientState { IDLE, CONNECTING, DISCONNECTING, ESTABLISHED };
enum class ConnectionType { V3_WITH_CACHE, V3_WITHOUT_CACHE };
}
struct Client {
  espbt::ClientState state_{espbt::ClientState::IDLE};
  espbt::ConnectionType connection_type_{espbt::ConnectionType::V3_WITHOUT_CACHE};
  espbt::ConnectionType connection_type_at_connect_{espbt::ConnectionType::V3_WITHOUT_CACHE};
  unsigned connects{0}, disconnects{0};
  auto state() const { return state_; }
  void set_connection_type(espbt::ConnectionType type) { connection_type_ = type; }
  void connect() {
    connects++;
    connection_type_at_connect_ = connection_type_;
    state_ = espbt::ClientState::CONNECTING;
  }
  void disconnect() {
    disconnects++;
    // ESPHome cannot actively disconnect until CONNECT_EVT supplies a conn_id.
    // A disconnect requested while CONNECTING is only remembered by the real
    // client, whose state stays CONNECTING until the stack reports completion.
    if (state_ != espbt::ClientState::CONNECTING)
      state_ = espbt::ClientState::DISCONNECTING;
  }
};
// PRODUCTION_PROTOCOL

struct YaleBatteryState {
  bool valid{false};
  float voltage{0};
};

class YaleXSBLE {
 public:
  // PRODUCTION_DECLARATIONS
  struct Timer { uint32_t due; std::function<void()> callback; };
  std::map<std::string, Timer> timers;
  Client client;
  struct {
    std::string model{"YUR/SSDL/1/SIL"};
    bool door_sense() const { return true; }
  } lock_info_;
  YaleLockStatus lock_status_{YaleLockStatus::LOCKED};
  YaleDoorStatus door_status_{YaleDoorStatus::UNKNOWN};
  uint32_t last_state_change_ms_{0};
  uint32_t next_battery_attempt_ms_{0};
  uint16_t pending_notify_handle_{0}, pending_cccd_handle_{0};
  NotifySetup pending_notify_setup_{NotifySetup::NONE};
  unsigned failures{0};
  unsigned lock_publications{0}, battery_updates{0};
  bool loop_enabled{true};
  bool session_authenticated_{false};
  uint32_t last_secure_notify_ms_{0};
  uint32_t last_normal_notify_ms_{0};
  unsigned run_next_calls{0};
  unsigned writes{0};
  bool cached_handles{false};
  static constexpr uint32_t REQUEST_COOLDOWN_MS = 250;
  static constexpr uint32_t BATTERY_TIMEOUT_COOLDOWN_MS = 300000;
  Client *parent() { return &client; }
  void enable_loop() { loop_enabled = true; }
  void disable_loop() { loop_enabled = false; }
  void status_set_warning(const char *) { failures++; }
  void publish_lock_state_() { lock_publications++; }
  void update_door_status_(YaleDoorStatus status) { door_status_ = status; }
  YaleBatteryState parse_battery_state_(const std::vector<uint8_t> &) const { return {true, 6.0}; }
  void update_battery_(const YaleBatteryState &) { battery_updates++; }
  const char *operation_to_string_(OperationType) { return "operation"; }
  const char *step_to_string_(StepType) { return "step"; }
  const char *response_channel_to_string_(ResponseChannel) { return "channel"; }
  const char *notify_setup_to_string_(NotifySetup) { return "notify"; }
  bool operation_queued_(OperationType operation) const {
    return std::find(operation_queue_.begin(), operation_queue_.end(), operation) != operation_queue_.end();
  }
  void run_next_step_() { run_next_calls++; }
  void write_command_now_(ResponseChannel, uint16_t, std::vector<uint8_t>, StepType, const char *) { writes++; }
  bool cached_gatt_handles_valid_() { return cached_handles; }
  void invalidate_gatt_cache_(const char *) { cached_handles = false; }
  void finish_operation_and_disconnect_() {}
  void queue_status_update() { queue_operation_(OperationType::UPDATE); }
  void reset_session_state_() {
    session_authenticated_ = false;
    session_shutting_down_ = false;
    active_step_ = StepType::NONE;
    pending_response_channel_ = ResponseChannel::NONE;
    lock_status_confirmed_ = false;
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
  void wait_for(StepType step) {
    active_step_ = step;
    pending_response_channel_ = ResponseChannel::NORMAL_NOTIFY;
    set_timeout("command_timeout", 1000, [this]() { handle_command_timeout_(); });
  }
};
// PRODUCTION_METHODS

static std::vector<uint8_t> response(uint8_t header, uint8_t opcode, uint8_t subtype = 0,
                                     uint8_t state = 0, uint8_t result = 0) {
  std::vector<uint8_t> frame(18, 0);
  frame[0] = header;
  frame[1] = opcode;
  frame[4] = subtype;
  frame[8] = state;
  frame[MOTOR_RESULT_OFFSET] = result;
  frame[3] = static_cast<uint8_t>(0 - header - opcode - subtype - state - result);
  return frame;
}

int main(int argc, char **argv) {
  assert(argc == 2);
  const std::string test = argv[1];
  YaleXSBLE yale;
  now = test == "wrap" ? UINT32_MAX - 50 : 1000;
  using Operation = YaleXSBLE::OperationType;
  using Step = YaleXSBLE::StepType;
  using Progress = YaleXSBLE::MotorProgress;
  if (test == "motor_sequence") {
    yale.lock_status_ = YaleLockStatus::SECUREMODE;
    yale.current_operation_ = Operation::UNLOCK;
    yale.wait_for(Step::FORCE_UNLOCK);
    yale.handle_normal_response_(response(0xAA, COMMAND_UNLOCK));
    assert(yale.motor_progress_ == Progress::ACKNOWLEDGED);
    assert(!yale.is_expected_complete_());
    yale.handle_normal_response_(response(0xBB, COMMAND_GETSTATUS, STATUS_LOCK_ONLY, 0x03));
    yale.handle_normal_response_(response(0xBB, COMMAND_LOCK));
    yale.handle_normal_response_(response(0xBB, COMMAND_GETSTATUS, STATUS_DOOR_ONLY, 0x02));
    assert(yale.lock_status_ == YaleLockStatus::SECUREMODE);
    assert(yale.door_status_ == YaleDoorStatus::AJAR);
    assert(yale.active_step_ == Step::FORCE_UNLOCK && yale.run_next_calls == 0);
    assert(yale.timers.count("command_timeout") == 1 && yale.lock_publications == 0);
    yale.handle_normal_response_(response(0xBB, COMMAND_UNLOCK));
    assert(yale.motor_progress_ == Progress::SUCCEEDED && yale.run_next_calls == 1);
    assert(yale.lock_status_ == YaleLockStatus::SECUREMODE && !yale.is_expected_complete_());
    yale.lock_status_ = YaleLockStatus::UNLOCKED;
    assert(!yale.is_expected_complete_());

    yale.build_operation_steps_();
    assert((yale.steps_ == std::deque<Step>{Step::LOCK_STATUS, Step::DOOR_STATUS}));
    yale.wait_for(Step::LOCK_STATUS);
    auto invalid = response(0xBB, COMMAND_GETSTATUS, STATUS_LOCK_ONLY, 0x03);
    invalid[3]++;
    yale.handle_normal_response_(invalid);
    yale.handle_normal_response_(response(0xAA, COMMAND_GETSTATUS, STATUS_LOCK_ONLY));
    yale.handle_normal_response_(response(0xBB, COMMAND_UNLOCK));
    assert(yale.active_step_ == Step::LOCK_STATUS && yale.run_next_calls == 1);
    yale.handle_normal_response_(response(0xBB, COMMAND_GETSTATUS, STATUS_LOCK_ONLY, 0x03));
    assert(yale.lock_status_ == YaleLockStatus::UNLOCKED && yale.is_expected_complete_());
    assert(yale.run_next_calls == 2 && yale.lock_publications == 1);

    // Confirm the same cache value on a later request and clear the entity's transition.
    yale.wait_for(Step::LOCK_STATUS);
    yale.handle_normal_response_(response(0xBB, COMMAND_GETSTATUS, STATUS_LOCK_ONLY, 0x03));
    assert(yale.lock_publications == 2);
    return 0;
  }
  if (test == "jam") {
    yale.current_operation_ = Operation::UNLOCK;
    yale.wait_for(Step::FORCE_UNLOCK);
    yale.handle_normal_response_(response(0xAA, COMMAND_UNLOCK));
    yale.handle_normal_response_(response(0xBB, COMMAND_UNLOCK, 0, 0, 0x1F));
    assert(yale.lock_status_ == YaleLockStatus::JAMMED && yale.failures == 1);
    assert(yale.current_operation_ == Operation::NONE && yale.run_next_calls == 0);
    assert(yale.timers.count("retry") == 0);
    yale.handle_normal_response_(response(0xBB, COMMAND_GETSTATUS, STATUS_LOCK_ONLY, 0x07));
    assert(yale.lock_status_ == YaleLockStatus::JAMMED);
    return 0;
  }
  if (test == "ack_timeout") {
    yale.current_operation_ = Operation::LOCK;
    yale.wait_for(Step::FORCE_LOCK);
    yale.handle_normal_response_(response(0xAA, COMMAND_LOCK, 0x04));
    assert(yale.motor_progress_ == Progress::NONE);
    yale.handle_normal_response_(response(0xAA, COMMAND_LOCK));
    assert(yale.motor_progress_ == Progress::ACKNOWLEDGED);
    yale.handle_command_timeout_();
    assert(yale.lock_status_ == YaleLockStatus::UNKNOWN && yale.failures == 1);
    assert(yale.current_operation_ == Operation::NONE && yale.timers.count("retry") == 0);
    assert((yale.operation_queue_ == std::deque<Operation>{Operation::UPDATE}));
    return 0;
  }
  if (test == "verification_timeout") {
    yale.current_operation_ = Operation::UNLOCK;
    yale.current_attempt_ = 1;
    yale.operation_retries_ = 1;
    yale.wait_for(Step::FORCE_UNLOCK);
    yale.handle_normal_response_(response(0xBB, COMMAND_UNLOCK));
    assert(yale.motor_progress_ == Progress::SUCCEEDED);
    yale.wait_for(Step::LOCK_STATUS);
    yale.handle_command_timeout_();
    assert(yale.timers.count("retry") == 1 && yale.failures == 0);
    yale.advance(250);
    assert(yale.current_attempt_ == 2);
    yale.build_operation_steps_();
    assert((yale.steps_ == std::deque<Step>{Step::LOCK_STATUS, Step::DOOR_STATUS}));
    yale.wait_for(Step::LOCK_STATUS);
    yale.handle_command_timeout_();
    assert(yale.lock_status_ == YaleLockStatus::UNKNOWN && yale.failures == 1);
    assert(yale.timers.count("retry") == 0);
    return 0;
  }
  if (test == "poll_matching") {
    yale.current_operation_ = Operation::UPDATE;
    yale.wait_for(Step::BATTERY_STATUS);
    yale.handle_normal_response_(response(0xBB, COMMAND_GETSTATUS, STATUS_LOCK_ONLY, 0x0C));
    assert(yale.lock_status_ == YaleLockStatus::SECUREMODE && yale.run_next_calls == 0);
    yale.handle_normal_response_(response(0xAA, COMMAND_GETSTATUS, STATUS_BATTERY));
    assert(yale.battery_updates == 0 && yale.timers.count("command_timeout") == 1);
    yale.handle_normal_response_(response(0xBB, COMMAND_GETSTATUS, STATUS_BATTERY));
    assert(yale.battery_updates == 1 && yale.run_next_calls == 1);
    return 0;
  }
  if (test == "priority") {
    yale.current_operation_ = YaleXSBLE::OperationType::UPDATE;
    yale.session_authenticated_ = true;
    yale.client.state_ = espbt::ClientState::ESTABLISHED;
    yale.operation_queue_.push_back(YaleXSBLE::OperationType::REFRESH_DOOR);
    yale.operation_queue_.push_back(YaleXSBLE::OperationType::UPDATE);
    yale.set_timeout("cooldown", 100, []() {});
    yale.queue_operation_(YaleXSBLE::OperationType::UNLOCK);
    assert(yale.operation_queue_.size() == 2);
    assert(yale.operation_queue_[0] == YaleXSBLE::OperationType::UNLOCK);
    assert(yale.operation_queue_[1] == YaleXSBLE::OperationType::REFRESH_DOOR);
    assert(yale.preempt_current_poll_);
    assert(yale.run_next_calls == 0);
    assert(yale.timers.count("cooldown") == 1);
    return 0;
  }
  if (test == "reuse") {
    yale.session_authenticated_ = true;
    yale.client.state_ = espbt::ClientState::ESTABLISHED;
    yale.operation_queue_.push_back(YaleXSBLE::OperationType::LOCK);
    yale.tick();
    assert(yale.current_operation_ == YaleXSBLE::OperationType::LOCK);
    assert(yale.current_attempt_ == 1);
    assert(yale.client.connects == 0);
    assert(yale.run_next_calls == 1);
    return 0;
  }
  if (test == "cooldown") {
    const std::vector<uint8_t> command(18, 0);
    yale.last_secure_notify_ms_ = now;
    yale.write_command_(YaleXSBLE::ResponseChannel::SECURE_NOTIFY, 1, command,
                        YaleXSBLE::StepType::SECURE_AUTH_2, "auth");
    assert(yale.writes == 1);
    assert(yale.timers.count("cooldown") == 0);

    yale.session_authenticated_ = true;
    yale.last_normal_notify_ms_ = 0;
    yale.write_command_(YaleXSBLE::ResponseChannel::NORMAL_NOTIFY, 2, command,
                        YaleXSBLE::StepType::FORCE_UNLOCK, "unlock");
    assert(yale.writes == 2);

    yale.last_normal_notify_ms_ = now - 100;
    yale.write_command_(YaleXSBLE::ResponseChannel::NORMAL_NOTIFY, 2, command,
                        YaleXSBLE::StepType::LOCK_STATUS, "status");
    assert(yale.writes == 2);
    assert(yale.timers.at("cooldown").due == now + 150);
    yale.advance(149);
    assert(yale.writes == 2);
    yale.advance(1);
    assert(yale.writes == 3);
    return 0;
  }
  if (test == "aggressive") {
    yale.cached_handles = true;
    yale.operation_queue_.push_back(YaleXSBLE::OperationType::UPDATE);
    yale.tick();
    assert(yale.client.connects == 1);
    assert(yale.client.connection_type_at_connect_ == espbt::ConnectionType::V3_WITHOUT_CACHE);
    assert(yale.client.connection_type_ == espbt::ConnectionType::V3_WITH_CACHE);
    return 0;
  }
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
