#pragma once

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/ble_client/ble_client.h"
#include "esphome/components/button/button.h"
#include "esphome/components/esp32_ble_tracker/esp32_ble_tracker.h"
#include "esphome/components/lock/lock.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"

#ifdef USE_ESP32

#include <array>
#include <cmath>
#include <deque>
#include <string>
#include <vector>

#include <esp_gap_ble_api.h>
#include <esp_gattc_api.h>

#define MBEDTLS_AES_ALT
#include <aes_alt.h>

namespace esphome::yalexs_ble {

namespace espbt = esphome::esp32_ble_tracker;

class YaleXSBLE;

class YaleXSBLELock : public lock::Lock {
 public:
  void set_parent(YaleXSBLE *parent) { this->parent_ = parent; }

 protected:
  void control(const lock::LockCall &call) override;

  YaleXSBLE *parent_{nullptr};
};

class YaleXSBLERefreshButton : public button::Button {
 public:
  void set_parent(YaleXSBLE *parent) { this->parent_ = parent; }

 protected:
  void press_action() override;

  YaleXSBLE *parent_{nullptr};
};

enum class YaleLockStatus : uint8_t {
  UNKNOWN = 0x00,
  UNKNOWN_01 = 0x01,
  UNLOCKING = 0x02,
  UNLOCKED = 0x03,
  LOCKING = 0x04,
  LOCKED = 0x05,
  UNKNOWN_06 = 0x06,
  SECUREMODE = 0x0C,
  JAMMED = 0x1B,
};

enum class YaleDoorStatus : uint8_t {
  UNKNOWN = 0x00,
  CLOSED = 0x01,
  AJAR = 0x02,
  OPENED = 0x03,
  UNKNOWN_04 = 0x04,
};

struct YaleBatteryState {
  bool valid{false};
  float voltage{NAN};
  uint8_t percentage{0};
};

struct YaleLockInfo {
  bool valid{false};
  std::string manufacturer;
  std::string model;
  std::string serial;
  std::string firmware;

  bool door_sense() const;
};

class YaleXSBLE : public Component,
                  public ble_client::BLEClientNode,
                  public esp32_ble_tracker::ESPBTDeviceListener {
 public:
  void setup() override;
  void dump_config() override;
  void loop() override;

  bool parse_device(const espbt::ESPBTDevice &device) override;
  void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                           esp_ble_gattc_cb_param_t *param) override;
  void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) override;

  void set_key(const std::string &key);
  void set_slot(uint8_t slot) { this->slot_ = slot; }
  void set_local_name(const std::string &local_name) { this->local_name_ = local_name; }
  void set_operation_retries(uint8_t retries) { this->operation_retries_ = retries; }
  void set_connect_timeout(uint32_t timeout) { this->connect_timeout_ms_ = timeout; }
  void set_command_timeout(uint32_t timeout) { this->command_timeout_ms_ = timeout; }
  void set_operation_timeout(uint32_t timeout) { this->operation_timeout_ms_ = timeout; }

  void set_lock(YaleXSBLELock *lock) { this->lock_entity_ = lock; }
  void set_refresh_button(YaleXSBLERefreshButton *button) { this->refresh_button_ = button; }
  void set_door_binary_sensor(binary_sensor::BinarySensor *sensor) { this->door_binary_sensor_ = sensor; }
  void set_door_status_text_sensor(text_sensor::TextSensor *sensor) { this->door_status_sensor_ = sensor; }
  void set_last_seen_broadcast_text_sensor(text_sensor::TextSensor *sensor) { this->last_seen_broadcast_sensor_ = sensor; }
  void set_battery_level_sensor(sensor::Sensor *sensor) { this->battery_level_sensor_ = sensor; }
  void set_battery_voltage_sensor(sensor::Sensor *sensor) { this->battery_voltage_sensor_ = sensor; }
  void set_rssi_sensor(sensor::Sensor *sensor) { this->rssi_sensor_ = sensor; }

  void request_lock();
  void request_unlock();
  void refresh_door_status();
  void queue_status_update();

 protected:
  enum class OperationType : uint8_t {
    NONE,
    UPDATE,
    REFRESH_DOOR,
    LOCK,
    UNLOCK,
  };

  enum class StepType : uint8_t {
    NONE,
    READ_INFO_MANUFACTURER,
    READ_INFO_MODEL,
    READ_INFO_SERIAL,
    READ_INFO_FIRMWARE,
    SECURE_AUTH_1,
    SECURE_AUTH_2,
    BATTERY_STATUS,
    DOOR_STATUS,
    LOCK_STATUS,
    FORCE_LOCK,
    FORCE_UNLOCK,
    SHUTDOWN,
  };

  enum class ResponseChannel : uint8_t {
    NONE,
    SECURE_NOTIFY,
    NORMAL_NOTIFY,
    READ_CHAR,
  };

  enum class NotifySetup : uint8_t {
    NONE,
    SECURE,
    NORMAL,
  };

  static constexpr uint16_t YALE_MFR_ID = 465;
  static constexpr uint16_t APPLE_MFR_ID = 76;
  static constexpr uint8_t HAP_FIRST_BYTE = 0x06;
  static constexpr uint8_t HAP_ENCRYPTED_FIRST_BYTE = 0x11;

  static constexpr uint32_t FALLBACK_NO_ANNOUNCEMENT_MS = 5UL * 60UL * 1000UL;
  static constexpr uint32_t FALLBACK_CHECK_INTERVAL_MS = 60UL * 1000UL;
  static constexpr uint32_t FALLBACK_MAX_BACKOFF_MS = 30UL * 60UL * 1000UL;
  static constexpr uint32_t IDLE_STATUS_POLL_MS = 20UL * 60UL * 1000UL;
  static constexpr uint32_t FIRST_UPDATE_COALESCE_MS = 10;
  static constexpr uint32_t ADV_UPDATE_COALESCE_MS = 50;
  static constexpr uint32_t HK_UPDATE_COALESCE_MS = 25;
  static constexpr uint32_t MANUAL_UPDATE_COALESCE_MS = 50;
  static constexpr uint32_t UPDATE_IN_PROGRESS_DEFER_MS = 4100;
  static constexpr uint32_t LOCK_STALE_STATE_DEBOUNCE_MS = 6100;
  static constexpr uint32_t BATTERY_TIMEOUT_COOLDOWN_MS = 5UL * 60UL * 1000UL;
  static constexpr uint32_t REQUEST_COOLDOWN_MS = 250;
  static constexpr uint32_t IDLE_DISCONNECT_MS = 5100;
  static constexpr uint32_t UNKNOWN_STATE_ADV_RETRY_MS = 30UL * 1000UL;

  bool matches_device_(const espbt::ESPBTDevice &device);
  void publish_last_seen_(const espbt::ESPBTDevice &device);
  void handle_advertisement_change_(const espbt::ESPBTDevice &device);
  int32_t get_manufacturer_u16_(const espbt::ESPBTDevice &device, uint16_t manufacturer_id,
                                const std::vector<uint8_t> **data) const;
  uint16_t get_homekit_state_num_(const std::vector<uint8_t> &data) const;
  bool needs_advertisement_retry_update_() const;

  void schedule_deferred_update_(uint32_t delay_ms);
  void queue_operation_(OperationType operation);
  bool operation_queued_(OperationType operation) const;
  void maybe_start_next_operation_();
  void start_current_attempt_();
  void connect_current_attempt_();
  void retry_current_operation_(const char *reason);
  void fail_current_operation_(const char *reason);
  void complete_current_operation_();
  void finish_operation_and_disconnect_();
  void force_disconnect_(const char *reason);
  void reset_session_state_();

  void on_connected_(bool services_discovered);
  bool resolve_handles_();
  bool find_characteristic_handles_(const espbt::ESPBTUUID &service_uuid, const espbt::ESPBTUUID &char_uuid,
                                    uint16_t *handle, uint8_t *properties);
  bool find_characteristic_handle_any_service_(const espbt::ESPBTUUID &char_uuid, uint16_t *handle);
  bool resolve_notify_descriptor_(uint16_t char_handle, NotifySetup setup, uint16_t *descriptor_handle);
  bool cached_gatt_handles_valid_() const;
  void clear_gatt_handles_();
  void invalidate_gatt_cache_(const char *reason);
  void start_notify_(NotifySetup setup);
  bool write_notify_descriptor_(uint16_t char_handle, NotifySetup setup);
  void begin_secure_handshake_();
  void set_secure_key_(const std::array<uint8_t, 16> &key);
  void set_normal_key_(const std::array<uint8_t, 16> &key);

  void run_next_step_();
  void build_operation_steps_();
  void execute_step_(StepType step);
  void read_info_char_(StepType step, uint16_t handle);
  void handle_info_response_(StepType step, const uint8_t *data, uint16_t len);

  void execute_secure_command_(uint8_t opcode, const uint8_t *payload, uint8_t payload_len, StepType step,
                               const char *name);
  void execute_normal_command_(uint8_t opcode, uint8_t command_byte, bool has_command_byte, StepType step,
                               const char *name);
  void execute_shutdown_();
  void write_command_(ResponseChannel channel, uint16_t handle, std::vector<uint8_t> command, StepType step,
                      const char *name);
  void write_command_now_(ResponseChannel channel, uint16_t handle, std::vector<uint8_t> command, StepType step,
                          const char *name);

  void handle_secure_response_(std::vector<uint8_t> encrypted, std::vector<uint8_t> decrypted);
  void handle_normal_response_(std::vector<uint8_t> decrypted);
  void handle_unsolicited_normal_(const std::vector<uint8_t> &decrypted);
  void handle_command_timeout_();

  std::vector<uint8_t> build_secure_command_(uint8_t opcode, const uint8_t *payload, uint8_t payload_len);
  std::vector<uint8_t> build_normal_command_(uint8_t opcode, uint8_t command_byte, bool has_command_byte);
  bool encrypt_secure_(std::vector<uint8_t> &command);
  bool decrypt_secure_(std::vector<uint8_t> &data);
  bool encrypt_normal_(std::vector<uint8_t> &command);
  bool decrypt_normal_(std::vector<uint8_t> &data);
  static uint8_t simple_checksum_(const std::vector<uint8_t> &buffer);
  static uint32_t security_checksum_(const std::vector<uint8_t> &buffer);
  static uint16_t bytes_to_u16_(const std::vector<uint8_t> &buffer, uint8_t offset);
  static void put_u32_le_(std::vector<uint8_t> &buffer, uint8_t offset, uint32_t value);
  static bool validate_simple_response_(const std::vector<uint8_t> &response);
  static bool validate_secure_response_(const std::vector<uint8_t> &response);

  void parse_state_response_(const std::vector<uint8_t> &response);
  void parse_lock_and_door_state_(const std::vector<uint8_t> &response);
  YaleLockStatus parse_lock_status_(uint8_t value) const;
  YaleDoorStatus parse_door_status_(uint8_t value) const;
  YaleBatteryState parse_battery_state_(const std::vector<uint8_t> &response) const;
  static uint8_t convert_voltage_to_percentage_(float single_cell_voltage);
  void update_lock_status_(YaleLockStatus status);
  void update_door_status_(YaleDoorStatus status);
  void update_battery_(const YaleBatteryState &battery);
  void publish_lock_state_();
  void publish_door_state_();
  void publish_battery_state_();

  bool is_expected_complete_() const;
  bool is_bad_lock_state_(YaleLockStatus status) const;
  bool is_jammed_state_(YaleLockStatus status) const;
  const char *lock_status_to_string_(YaleLockStatus status) const;
  const char *door_status_to_string_(YaleDoorStatus status) const;
  const char *operation_to_string_(OperationType operation) const;
  const char *step_to_string_(StepType step) const;
  const char *notify_setup_to_string_(NotifySetup setup) const;
  const char *response_channel_to_string_(ResponseChannel channel) const;

  YaleXSBLELock *lock_entity_{nullptr};
  YaleXSBLERefreshButton *refresh_button_{nullptr};
  binary_sensor::BinarySensor *door_binary_sensor_{nullptr};
  text_sensor::TextSensor *door_status_sensor_{nullptr};
  text_sensor::TextSensor *last_seen_broadcast_sensor_{nullptr};
  sensor::Sensor *battery_level_sensor_{nullptr};
  sensor::Sensor *battery_voltage_sensor_{nullptr};
  sensor::Sensor *rssi_sensor_{nullptr};

  std::string local_name_;
  std::array<uint8_t, 16> key_{};
  std::array<uint8_t, 16> session_key_{};
  std::array<uint8_t, 16> handshake_keys_{};
  std::array<uint8_t, 16> normal_encrypt_iv_{};
  std::array<uint8_t, 16> normal_decrypt_iv_{};
  uint8_t slot_{0};
  uint8_t operation_retries_{5};
  uint32_t connect_timeout_ms_{8000};
  uint32_t command_timeout_ms_{10000};
  uint32_t operation_timeout_ms_{20000};

  mbedtls_aes_context secure_encrypt_ctx_{};
  mbedtls_aes_context secure_decrypt_ctx_{};
  mbedtls_aes_context normal_encrypt_ctx_{};
  mbedtls_aes_context normal_decrypt_ctx_{};
  bool aes_initialized_{false};
  bool session_authenticated_{false};

  uint16_t secure_read_handle_{0};
  uint16_t secure_write_handle_{0};
  uint16_t normal_read_handle_{0};
  uint16_t normal_write_handle_{0};
  uint8_t secure_read_properties_{0};
  uint8_t normal_read_properties_{0};
  uint16_t secure_cccd_handle_{0};
  uint16_t normal_cccd_handle_{0};
  uint16_t manufacturer_handle_{0};
  uint16_t model_handle_{0};
  uint16_t serial_handle_{0};
  uint16_t firmware_handle_{0};
  uint16_t pending_cccd_handle_{0};
  uint16_t pending_notify_handle_{0};
  uint16_t pending_read_handle_{0};
  NotifySetup pending_notify_setup_{NotifySetup::NONE};
  ResponseChannel pending_response_channel_{ResponseChannel::NONE};
  StepType active_step_{StepType::NONE};

  std::deque<OperationType> operation_queue_;
  std::deque<StepType> steps_;
  OperationType current_operation_{OperationType::NONE};
  uint8_t current_attempt_{0};
  uint32_t operation_started_ms_{0};
  bool retry_pending_{false};
  bool operation_steps_built_{false};
  bool ignore_next_disconnect_{false};
  bool gatt_handles_valid_{false};
  bool current_connection_uses_gatt_cache_{false};

  YaleLockInfo lock_info_;
  YaleBatteryState battery_;
  YaleLockStatus lock_status_{YaleLockStatus::UNKNOWN};
  YaleDoorStatus door_status_{YaleDoorStatus::UNKNOWN};
  bool lock_seen_this_session_{false};
  bool door_seen_this_session_{false};
  bool battery_seen_this_session_{false};

  int last_adv_value_{-1};
  int last_hk_state_{-1};
  bool deferred_update_scheduled_{false};
  uint32_t deferred_update_due_ms_{0};
  uint32_t last_adv_seen_ms_{0};
  uint32_t last_state_change_ms_{0};
  uint32_t next_fallback_update_ms_{0};
  uint32_t fallback_backoff_ms_{FALLBACK_NO_ANNOUNCEMENT_MS};
  uint32_t last_lock_operation_complete_ms_{0};
  uint32_t last_operation_complete_ms_{0};
  uint32_t next_battery_attempt_ms_{0};
  uint32_t last_notify_ms_{0};
  uint32_t last_unknown_state_update_request_ms_{0};
};

}  // namespace esphome::yalexs_ble

#endif  // USE_ESP32
