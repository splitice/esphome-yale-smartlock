#include "yalexs_ble.h"

#include "esphome/core/application.h"
#include "esphome/core/log.h"

#ifdef USE_ESP32

#include <algorithm>
#include <cinttypes>
#include <cstring>

namespace esphome::yalexs_ble {

static const char *const TAG = "yalexs_ble";

static const espbt::ESPBTUUID COMMAND_SERVICE_UUID =
    espbt::ESPBTUUID::from_raw("0000fe24-0000-1000-8000-00805f9b34fb");
static const espbt::ESPBTUUID WRITE_CHARACTERISTIC =
    espbt::ESPBTUUID::from_raw("bd4ac611-0b45-11e3-8ffd-0800200c9a66");
static const espbt::ESPBTUUID READ_CHARACTERISTIC =
    espbt::ESPBTUUID::from_raw("bd4ac612-0b45-11e3-8ffd-0800200c9a66");
static const espbt::ESPBTUUID SECURE_WRITE_CHARACTERISTIC =
    espbt::ESPBTUUID::from_raw("bd4ac613-0b45-11e3-8ffd-0800200c9a66");
static const espbt::ESPBTUUID SECURE_READ_CHARACTERISTIC =
    espbt::ESPBTUUID::from_raw("bd4ac614-0b45-11e3-8ffd-0800200c9a66");

static const espbt::ESPBTUUID MANUFACTURER_NAME_CHARACTERISTIC = espbt::ESPBTUUID::from_uint16(0x2A29);
static const espbt::ESPBTUUID MODEL_NUMBER_CHARACTERISTIC = espbt::ESPBTUUID::from_uint16(0x2A24);
static const espbt::ESPBTUUID SERIAL_NUMBER_CHARACTERISTIC = espbt::ESPBTUUID::from_uint16(0x2A25);
static const espbt::ESPBTUUID FIRMWARE_REVISION_CHARACTERISTIC = espbt::ESPBTUUID::from_uint16(0x2A26);

static const esp_bt_uuid_t NOTIFY_DESC_UUID = {
    .len = ESP_UUID_LEN_16,
    .uuid =
        {
            .uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,
        },
};

enum YaleCommand : uint8_t {
  COMMAND_GETSTATUS = 0x02,
  COMMAND_UNLOCK = 0x0A,
  COMMAND_LOCK = 0x0B,
  COMMAND_LOCK_ACTIVITY = 0x2D,
};

enum YaleStatusType : uint8_t {
  STATUS_LOCK_ONLY = 0x02,
  STATUS_BATTERY = 0x0F,
  STATUS_DOOR_ONLY = 0x2E,
  STATUS_DOOR_AND_LOCK = 0x2F,
};

bool YaleLockInfo::door_sense() const {
  if (!this->valid || this->model.empty())
    return false;
  return this->model.rfind("ASL-02", 0) != 0 && this->model.rfind("ASL-01", 0) != 0;
}

void YaleXSBLELock::control(const lock::LockCall &call) {
  if (this->parent_ == nullptr)
    return;
  if (!call.get_state().has_value())
    return;
  switch (*call.get_state()) {
    case lock::LOCK_STATE_LOCKED:
      this->parent_->request_lock();
      break;
    case lock::LOCK_STATE_UNLOCKED:
      this->parent_->request_unlock();
      break;
    default:
      ESP_LOGW(TAG, "Unsupported requested lock state");
      break;
  }
}

void YaleXSBLERefreshButton::press_action() {
  if (this->parent_ != nullptr)
    this->parent_->refresh_door_status();
}

void YaleXSBLE::setup() {
  mbedtls_aes_init(&this->secure_encrypt_ctx_);
  mbedtls_aes_init(&this->secure_decrypt_ctx_);
  mbedtls_aes_init(&this->normal_encrypt_ctx_);
  mbedtls_aes_init(&this->normal_decrypt_ctx_);
  this->aes_initialized_ = true;

  const uint32_t now = millis();
  this->last_adv_seen_ms_ = now;
  this->last_state_change_ms_ = now;

  this->set_interval("fallback", FALLBACK_CHECK_INTERVAL_MS, [this]() {
    const uint32_t now = millis();
    if (now - this->last_adv_seen_ms_ >= FALLBACK_NO_ANNOUNCEMENT_MS &&
        (this->next_fallback_update_ms_ == 0 || now - this->next_fallback_update_ms_ < 0x80000000UL)) {
      ESP_LOGD(TAG, "No advertisement seen recently; queueing fallback status update");
      this->queue_status_update();
      this->next_fallback_update_ms_ = now + this->fallback_backoff_ms_;
      this->fallback_backoff_ms_ = std::min<uint32_t>(this->fallback_backoff_ms_ * 2, FALLBACK_MAX_BACKOFF_MS);
    }
    if (now - this->last_state_change_ms_ >= IDLE_STATUS_POLL_MS) {
      ESP_LOGD(TAG, "Idle status poll interval elapsed; queueing status update");
      this->queue_status_update();
    }
  });

  this->disable_loop();
}

void YaleXSBLE::dump_config() {
  ESP_LOGCONFIG(TAG, "Yale XS BLE:");
  ESP_LOGCONFIG(TAG, "  Local Name: %s", this->local_name_.empty() ? "(not set)" : this->local_name_.c_str());
  ESP_LOGCONFIG(TAG, "  Slot: %u", this->slot_);
  ESP_LOGCONFIG(TAG, "  Operation Retries: %u", this->operation_retries_);
  ESP_LOGCONFIG(TAG, "  Connect Timeout: %" PRIu32 " ms", this->connect_timeout_ms_);
  ESP_LOGCONFIG(TAG, "  Command Timeout: %" PRIu32 " ms", this->command_timeout_ms_);
  ESP_LOGCONFIG(TAG, "  Operation Timeout: %" PRIu32 " ms", this->operation_timeout_ms_);
  LOG_LOCK("  ", "Lock", this->lock_entity_);
  LOG_BUTTON("  ", "Refresh Door Status", this->refresh_button_);
  LOG_BINARY_SENSOR("  ", "Door", this->door_binary_sensor_);
  LOG_TEXT_SENSOR("  ", "Door Status", this->door_status_sensor_);
  LOG_TEXT_SENSOR("  ", "Last Seen Broadcast", this->last_seen_broadcast_sensor_);
  LOG_SENSOR("  ", "Battery Level", this->battery_level_sensor_);
  LOG_SENSOR("  ", "Battery Voltage", this->battery_voltage_sensor_);
  LOG_SENSOR("  ", "RSSI", this->rssi_sensor_);
}

void YaleXSBLE::loop() {
  if (this->current_operation_ == OperationType::NONE && this->operation_queue_.empty()) {
    this->disable_loop();
    return;
  }

  if (this->current_operation_ != OperationType::NONE && !this->retry_pending_ &&
      millis() - this->operation_started_ms_ > this->operation_timeout_ms_) {
    this->retry_current_operation_("operation timeout");
    return;
  }

  this->maybe_start_next_operation_();
}

void YaleXSBLE::set_key(const std::string &key) {
  for (size_t i = 0; i < this->key_.size(); i++) {
    const std::string byte = key.substr(i * 2, 2);
    this->key_[i] = static_cast<uint8_t>(strtoul(byte.c_str(), nullptr, 16));
  }
}

bool YaleXSBLE::matches_device_(const espbt::ESPBTDevice &device) {
  if (!this->local_name_.empty() && this->local_name_.size() == 7 && device.get_name() == this->local_name_)
    return true;
  return this->parent() != nullptr && device.address_uint64() == this->parent()->get_address();
}

bool YaleXSBLE::parse_device(const espbt::ESPBTDevice &device) {
  if (!this->matches_device_(device))
    return false;

  this->parent()->set_address(device.address_uint64());
  this->parent()->set_remote_addr_type(device.get_address_type());

  this->last_adv_seen_ms_ = millis();
  this->fallback_backoff_ms_ = FALLBACK_NO_ANNOUNCEMENT_MS;
  this->next_fallback_update_ms_ = 0;

  if (this->rssi_sensor_ != nullptr)
    this->rssi_sensor_->publish_state(device.get_rssi());

  this->publish_last_seen_(device);
  this->handle_advertisement_change_(device);
  return true;
}

void YaleXSBLE::publish_last_seen_(const espbt::ESPBTDevice &device) {
  if (this->last_seen_broadcast_sensor_ == nullptr)
    return;

  std::string message = "address=" + device.address_str();
  message += ", name=";
  const auto &name = device.get_name();
  if (name.empty())
    message += "(none)";
  else
    message.append(name.begin(), name.end());
  message += ", rssi=" + str_sprintf("%d", device.get_rssi());
  message += ", mfr=";

  bool first = true;
  for (const auto &mfr : device.get_manufacturer_datas()) {
    const auto uuid = mfr.uuid.get_uuid();
    uint32_t id = 0;
    if (uuid.len == ESP_UUID_LEN_16) {
      id = uuid.uuid.uuid16;
    } else if (uuid.len == ESP_UUID_LEN_32) {
      id = uuid.uuid.uuid32;
    }
    if (!first)
      message += ";";
    first = false;
    message += str_sprintf("%04" PRIX32 ":%s", id, format_hex(mfr.data).c_str());
  }
  if (first)
    message += "(none)";

  this->last_seen_broadcast_sensor_->publish_state(message);
}

int32_t YaleXSBLE::get_manufacturer_u16_(const espbt::ESPBTDevice &device, uint16_t manufacturer_id,
                                         const std::vector<uint8_t> **data) const {
  for (const auto &mfr : device.get_manufacturer_datas()) {
    const auto uuid = mfr.uuid.get_uuid();
    if (uuid.len == ESP_UUID_LEN_16 && uuid.uuid.uuid16 == manufacturer_id) {
      *data = &mfr.data;
      return mfr.data.empty() ? -1 : mfr.data[0];
    }
  }
  *data = nullptr;
  return -1;
}

uint16_t YaleXSBLE::get_homekit_state_num_(const std::vector<uint8_t> &data) const {
  if (data.size() < 13)
    return 0;
  return static_cast<uint16_t>(data[11]) | (static_cast<uint16_t>(data[12]) << 8);
}

void YaleXSBLE::handle_advertisement_change_(const espbt::ESPBTDevice &device) {
  uint32_t next_update = 0;
  bool saw_state_advertisement = false;
  const std::vector<uint8_t> *apple_data = nullptr;
  const std::vector<uint8_t> *yale_data = nullptr;

  if (this->get_manufacturer_u16_(device, APPLE_MFR_ID, &apple_data) >= 0 && apple_data != nullptr &&
      !apple_data->empty()) {
    const uint8_t first_byte = (*apple_data)[0];
    if (first_byte == HAP_FIRST_BYTE && apple_data->size() >= 13) {
      saw_state_advertisement = true;
      const uint16_t hk_state = this->get_homekit_state_num_(*apple_data);
      if (this->last_hk_state_ == -1)
        next_update = FIRST_UPDATE_COALESCE_MS;
      else if (hk_state != this->last_hk_state_)
        next_update = HK_UPDATE_COALESCE_MS;
      this->last_hk_state_ = hk_state;
    } else if (first_byte == HAP_ENCRYPTED_FIRST_BYTE) {
      saw_state_advertisement = true;
      next_update = HK_UPDATE_COALESCE_MS;
    }
  }

  const bool first_yale_adv = this->last_adv_value_ == -1;
  if (this->get_manufacturer_u16_(device, YALE_MFR_ID, &yale_data) >= 0 && yale_data != nullptr &&
      (yale_data->size() == 1 || first_yale_adv)) {
    const uint8_t current_value = (*yale_data)[0];
    const bool valid_yale_value = current_value == 0 || current_value == 1;
    if (valid_yale_value)
      saw_state_advertisement = true;
    if (next_update == 0) {
      if (first_yale_adv) {
        next_update = FIRST_UPDATE_COALESCE_MS;
      } else if (valid_yale_value && current_value != this->last_adv_value_) {
        next_update = ADV_UPDATE_COALESCE_MS;
      }
    }
    this->last_adv_value_ = current_value;
  }

  if (next_update == 0 && saw_state_advertisement && this->needs_advertisement_retry_update_()) {
    const uint32_t now = millis();
    if (this->last_unknown_state_update_request_ms_ == 0 ||
        now - this->last_unknown_state_update_request_ms_ >= UNKNOWN_STATE_ADV_RETRY_MS) {
      ESP_LOGD(TAG, "State still unknown while advertisements are present; queueing authenticated update");
      this->last_unknown_state_update_request_ms_ = now;
      next_update = MANUAL_UPDATE_COALESCE_MS;
    }
  }

  if (next_update != 0)
    this->schedule_deferred_update_(next_update);
}

bool YaleXSBLE::needs_advertisement_retry_update_() const {
  if (!this->lock_info_.valid)
    return true;
  if (this->lock_entity_ != nullptr && this->lock_status_ == YaleLockStatus::UNKNOWN)
    return true;
  if (this->lock_info_.door_sense() && (this->door_binary_sensor_ != nullptr || this->door_status_sensor_ != nullptr) &&
      this->door_status_ == YaleDoorStatus::UNKNOWN)
    return true;
  if ((this->battery_level_sensor_ != nullptr || this->battery_voltage_sensor_ != nullptr) && !this->battery_.valid)
    return true;
  return false;
}

void YaleXSBLE::schedule_deferred_update_(uint32_t delay_ms) {
  const uint32_t now = millis();
  const uint32_t due = now + delay_ms;
  if (this->deferred_update_scheduled_) {
    const uint32_t remaining = this->deferred_update_due_ms_ - now;
    if (remaining <= delay_ms && remaining >= HK_UPDATE_COALESCE_MS)
      return;
    this->cancel_timeout("deferred_update");
  }
  this->deferred_update_scheduled_ = true;
  this->deferred_update_due_ms_ = due;
  this->set_timeout("deferred_update", delay_ms, [this]() {
    this->deferred_update_scheduled_ = false;
    if (this->current_operation_ != OperationType::NONE) {
      this->schedule_deferred_update_(UPDATE_IN_PROGRESS_DEFER_MS);
      return;
    }
    if (millis() - this->last_lock_operation_complete_ms_ < LOCK_STALE_STATE_DEBOUNCE_MS) {
      const uint32_t wait = LOCK_STALE_STATE_DEBOUNCE_MS - (millis() - this->last_lock_operation_complete_ms_);
      this->schedule_deferred_update_(wait);
      return;
    }
    this->queue_status_update();
  });
}

void YaleXSBLE::request_lock() {
  ESP_LOGI(TAG, "Lock requested");
  if (this->lock_entity_ != nullptr)
    this->lock_entity_->publish_state(lock::LOCK_STATE_LOCKING);
  this->cancel_timeout("deferred_update");
  this->deferred_update_scheduled_ = false;
  this->queue_operation_(OperationType::LOCK);
}

void YaleXSBLE::request_unlock() {
  ESP_LOGI(TAG, "Unlock requested");
  if (this->lock_entity_ != nullptr)
    this->lock_entity_->publish_state(lock::LOCK_STATE_UNLOCKING);
  this->cancel_timeout("deferred_update");
  this->deferred_update_scheduled_ = false;
  this->queue_operation_(OperationType::UNLOCK);
}

void YaleXSBLE::refresh_door_status() {
  ESP_LOGI(TAG, "Refresh door status requested");
  this->queue_operation_(OperationType::REFRESH_DOOR);
}

void YaleXSBLE::queue_status_update() { this->queue_operation_(OperationType::UPDATE); }

bool YaleXSBLE::operation_queued_(OperationType operation) const {
  return std::find(this->operation_queue_.begin(), this->operation_queue_.end(), operation) != this->operation_queue_.end();
}

void YaleXSBLE::queue_operation_(OperationType operation) {
  if ((operation == OperationType::UPDATE || operation == OperationType::REFRESH_DOOR) &&
      (this->current_operation_ == operation || this->operation_queued_(operation))) {
    return;
  }

  const bool priority_operation = operation == OperationType::LOCK || operation == OperationType::UNLOCK;
  if (!priority_operation) {
    this->operation_queue_.push_back(operation);
    this->enable_loop();
    return;
  }
  if (this->current_operation_ == operation)
    return;

  // The most recent physical command wins. A queued automatic update is stale
  // once a lock command has been requested, while a manual door refresh can
  // safely remain behind the command.
  this->operation_queue_.erase(
      std::remove_if(this->operation_queue_.begin(), this->operation_queue_.end(), [](OperationType queued) {
        return queued == OperationType::LOCK || queued == OperationType::UNLOCK || queued == OperationType::UPDATE;
      }),
      this->operation_queue_.end());
  this->operation_queue_.push_front(operation);

  if (this->current_operation_ == OperationType::UPDATE || this->current_operation_ == OperationType::REFRESH_DOOR) {
    this->preempt_current_poll_ = true;
    this->steps_.clear();
    // Let an in-flight request, including one whose encrypted bytes are waiting
    // for the cooldown timer, finish first. Dropping an already-encrypted CBC
    // command would desynchronize the lock's IV from ours.
  }
  this->enable_loop();
}

void YaleXSBLE::maybe_start_next_operation_() {
  if (this->current_operation_ != OperationType::NONE || this->operation_queue_.empty())
    return;
  this->current_operation_ = this->operation_queue_.front();
  this->operation_queue_.pop_front();
  this->current_attempt_ = 0;

  if (this->session_authenticated_ && !this->session_shutting_down_ && this->parent() != nullptr &&
      this->parent()->state() == espbt::ClientState::ESTABLISHED) {
    ESP_LOGD(TAG, "Reusing authenticated BLE session for %s", this->operation_to_string_(this->current_operation_));
    this->cancel_timeout("idle_disconnect");
    this->current_attempt_ = 1;
    this->operation_started_ms_ = millis();
    this->operation_steps_built_ = false;
    this->steps_.clear();
    this->active_step_ = StepType::NONE;
    this->pending_response_channel_ = ResponseChannel::NONE;
    this->preempt_current_poll_ = false;
    this->run_next_step_();
    return;
  }

  this->start_current_attempt_();
}

void YaleXSBLE::start_current_attempt_() {
  if (this->current_operation_ == OperationType::NONE)
    return;

  // If a foreground lock command arrived while a background poll was being
  // retried, abandon the poll before opening another connection for it.
  if (this->preempt_current_poll_ && !this->session_authenticated_ &&
      (this->current_operation_ == OperationType::UPDATE || this->current_operation_ == OperationType::REFRESH_DOOR) &&
      !this->operation_queue_.empty() &&
      (this->operation_queue_.front() == OperationType::LOCK ||
       this->operation_queue_.front() == OperationType::UNLOCK)) {
    this->current_operation_ = this->operation_queue_.front();
    this->operation_queue_.pop_front();
    this->current_attempt_ = 0;
    this->preempt_current_poll_ = false;
    ESP_LOGD(TAG, "Abandoned background poll for priority %s",
             this->operation_to_string_(this->current_operation_));
  }

  if (this->current_attempt_ > this->operation_retries_) {
    this->fail_current_operation_("retries exhausted");
    return;
  }

  // ESPHome cannot cancel a connection until ESP-IDF assigns it a connection
  // ID. In that case disconnect() only records the request and the client stays
  // CONNECTING until the eventual OPEN/DISCONNECT events arrive. Do not consume
  // another attempt or call connect() again while that previous attempt is
  // still being settled.
  if (this->parent() != nullptr && this->parent()->state() != espbt::ClientState::IDLE) {
    this->retry_pending_ = true;
    if (!this->waiting_for_ble_idle_) {
      this->waiting_for_ble_idle_ = true;
      this->ble_idle_wait_started_ms_ = millis();
      this->force_disconnect_("waiting for idle before attempt");
    }
    if (millis() - this->ble_idle_wait_started_ms_ > this->operation_timeout_ms_) {
      this->fail_current_operation_("BLE client did not become idle");
      return;
    }
    this->set_timeout("retry_after_disconnect", 250, [this]() { this->start_current_attempt_(); });
    return;
  }

  this->waiting_for_ble_idle_ = false;
  this->retry_pending_ = false;
  // Count an attempt only when a new connection can actually be started.
  this->current_attempt_++;
  this->operation_started_ms_ = millis();
  this->operation_steps_built_ = false;
  this->steps_.clear();
  this->active_step_ = StepType::NONE;
  this->pending_response_channel_ = ResponseChannel::NONE;
  this->preempt_current_poll_ = false;
  this->reset_session_state_();

  this->connect_current_attempt_();
}

void YaleXSBLE::connect_current_attempt_() {
  if (this->current_operation_ == OperationType::NONE || this->retry_pending_)
    return;
  if (this->parent() == nullptr) {
    this->fail_current_operation_("missing ble_client parent");
    return;
  }
  if (this->parent()->state() != espbt::ClientState::IDLE) {
    this->set_timeout("retry_after_disconnect", 250, [this]() { this->start_current_attempt_(); });
    return;
  }

  this->current_connection_uses_gatt_cache_ = this->cached_gatt_handles_valid_();

  ESP_LOGD(TAG, "Connecting to lock for attempt %u/%u using %s GATT handles", this->current_attempt_,
           this->operation_retries_ + 1, this->current_connection_uses_gatt_cache_ ? "cached" : "discovered");
  this->ignore_next_disconnect_ = false;
  if (this->current_connection_uses_gatt_cache_) {
    // ESPHome normally selects its medium (8.75-11.25 ms) parameters for a
    // cached connection. Select its fast 7.5 ms setup parameters for the open
    // call, then restore the cached type before the asynchronous OPEN event so
    // service discovery is still skipped.
    this->parent()->set_connection_type(espbt::ConnectionType::V3_WITHOUT_CACHE);
    this->parent()->connect();
    this->parent()->set_connection_type(espbt::ConnectionType::V3_WITH_CACHE);
  } else {
    this->parent()->set_connection_type(espbt::ConnectionType::V3_WITHOUT_CACHE);
    this->parent()->connect();
  }
  this->set_timeout("connect_timeout", this->connect_timeout_ms_, [this]() {
    this->retry_current_operation_("connect timeout");
  });
}

void YaleXSBLE::retry_current_operation_(const char *reason) {
  if (this->current_operation_ == OperationType::NONE || this->retry_pending_)
    return;
  // Disarm the expired attempt before disconnecting. Otherwise loop() keeps
  // replacing the retry timer, so its callback never gets a chance to run.
  this->retry_pending_ = true;
  ESP_LOGW(TAG, "Attempt failed: %s", reason);
  this->cancel_timeout("connect_timeout");
  this->cancel_timeout("command_timeout");
  this->cancel_timeout("cooldown");
  this->cancel_timeout("notify_register_fallback");
  this->cancel_timeout("retry_after_disconnect");
  this->cancel_timeout("retry");
  this->waiting_for_ble_idle_ =
      this->parent() != nullptr && this->parent()->state() != espbt::ClientState::IDLE;
  this->ble_idle_wait_started_ms_ = millis();
  this->force_disconnect_(reason);

  if (this->current_attempt_ > this->operation_retries_) {
    this->fail_current_operation_(reason);
    return;
  }

  this->set_timeout("retry", 250, [this]() { this->start_current_attempt_(); });
}

void YaleXSBLE::fail_current_operation_(const char *reason) {
  ESP_LOGE(TAG, "Operation failed: %s", reason);
  this->status_set_warning(reason);
  this->cancel_timeout("connect_timeout");
  this->cancel_timeout("command_timeout");
  this->cancel_timeout("cooldown");
  this->cancel_timeout("notify_register_fallback");
  this->cancel_timeout("retry");
  this->cancel_timeout("retry_after_disconnect");
  this->force_disconnect_(reason);
  if (this->lock_entity_ != nullptr &&
      (this->current_operation_ == OperationType::LOCK || this->current_operation_ == OperationType::UNLOCK)) {
    this->publish_lock_state_();
  }
  this->current_operation_ = OperationType::NONE;
  this->retry_pending_ = false;
  this->waiting_for_ble_idle_ = false;
  this->operation_steps_built_ = false;
  this->steps_.clear();
  this->active_step_ = StepType::NONE;
  this->preempt_current_poll_ = false;
  this->enable_loop();
}

void YaleXSBLE::complete_current_operation_() {
  if (!this->is_expected_complete_()) {
    this->retry_current_operation_("expected lock state not confirmed");
    return;
  }
  this->status_clear_warning();
  if (!this->operation_queue_.empty() && this->session_authenticated_ && !this->session_shutting_down_ &&
      this->parent() != nullptr && this->parent()->state() == espbt::ClientState::ESTABLISHED) {
    ESP_LOGD(TAG, "Keeping authenticated BLE session for queued %s",
             this->operation_to_string_(this->operation_queue_.front()));
    this->finish_operation_and_disconnect_();
    return;
  }
  if (this->session_authenticated_ && this->secure_write_handle_ != 0) {
    this->execute_shutdown_();
    return;
  }
  this->finish_operation_and_disconnect_();
}

void YaleXSBLE::finish_operation_and_disconnect_() {
  const uint32_t now = millis();
  if (this->current_operation_ == OperationType::LOCK || this->current_operation_ == OperationType::UNLOCK)
    this->last_lock_operation_complete_ms_ = now;
  this->last_operation_complete_ms_ = now;

  this->current_operation_ = OperationType::NONE;
  this->retry_pending_ = false;
  this->waiting_for_ble_idle_ = false;
  this->operation_steps_built_ = false;
  this->steps_.clear();
  this->active_step_ = StepType::NONE;
  this->preempt_current_poll_ = false;
  const uint32_t idle_delay = this->operation_queue_.empty() ? 100 : IDLE_DISCONNECT_MS;
  this->set_timeout("idle_disconnect", idle_delay, [this]() {
    if (this->current_operation_ == OperationType::NONE)
      this->force_disconnect_("idle");
  });
  this->enable_loop();
}

void YaleXSBLE::force_disconnect_(const char *reason) {
  ESP_LOGD(TAG, "Disconnecting BLE client: %s", reason);
  this->cancel_timeout("notify_register_fallback");
  this->reset_session_state_();
  if (this->parent() != nullptr && this->parent()->state() != espbt::ClientState::IDLE) {
    this->ignore_next_disconnect_ = true;
    this->parent()->disconnect();
  }
}

void YaleXSBLE::reset_session_state_() {
  this->session_authenticated_ = false;
  this->session_shutting_down_ = false;
  this->aggressive_conn_params_requested_ = false;
  this->pending_cccd_handle_ = 0;
  this->pending_notify_handle_ = 0;
  this->pending_read_handle_ = 0;
  this->pending_notify_setup_ = NotifySetup::NONE;
  this->pending_response_channel_ = ResponseChannel::NONE;
  this->active_step_ = StepType::NONE;
  this->lock_seen_this_session_ = false;
  this->door_seen_this_session_ = false;
  this->battery_seen_this_session_ = false;
  this->last_secure_notify_ms_ = 0;
  this->last_normal_notify_ms_ = 0;
  this->normal_encrypt_iv_.fill(0);
  this->normal_decrypt_iv_.fill(0);
}

void YaleXSBLE::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                                    esp_ble_gattc_cb_param_t *param) {
  switch (event) {
    case ESP_GATTC_CONNECT_EVT:
      if (!this->parent()->check_addr(param->connect.remote_bda))
        return;
      this->request_aggressive_connection_params_();
      break;

    case ESP_GATTC_OPEN_EVT:
      if (!this->parent()->check_addr(param->open.remote_bda))
        return;
      if (param->open.status != ESP_GATT_OK && param->open.status != ESP_GATT_ALREADY_OPEN) {
        this->retry_current_operation_("connection open failed");
        return;
      }
      if (this->current_connection_uses_gatt_cache_) {
        this->cancel_timeout("connect_timeout");
        this->on_connected_(false);
      }
      break;

    case ESP_GATTC_SEARCH_CMPL_EVT:
      if (this->parent()->get_conn_id() != param->search_cmpl.conn_id)
        return;
      if (param->search_cmpl.status != ESP_GATT_OK) {
        this->retry_current_operation_("service discovery failed");
        return;
      }
      this->cancel_timeout("connect_timeout");
      this->on_connected_(true);
      break;

    case ESP_GATTC_REG_FOR_NOTIFY_EVT:
      ESP_LOGD(TAG, "Notify registration event: status=%d handle=0x%04x pending=0x%04x setup=%s",
               param->reg_for_notify.status, param->reg_for_notify.handle, this->pending_notify_handle_,
               this->notify_setup_to_string_(this->pending_notify_setup_));
      if (this->pending_notify_setup_ == NotifySetup::NONE)
        return;
      if (param->reg_for_notify.status != ESP_GATT_OK || param->reg_for_notify.handle != this->pending_notify_handle_) {
        this->invalidate_gatt_cache_("register notify failed");
        this->retry_current_operation_("register notify failed");
        return;
      }
      this->cancel_timeout("notify_register_fallback");
      if (this->pending_cccd_handle_ != 0) {
        ESP_LOGD(TAG, "Notify CCCD write already in progress; ignoring late registration event");
        return;
      }
      this->write_notify_descriptor_(this->pending_notify_handle_, this->pending_notify_setup_);
      break;

    case ESP_GATTC_WRITE_DESCR_EVT: {
      ESP_LOGD(TAG, "Notify descriptor write event: status=%d conn_id=%d handle=0x%04x pending=0x%04x setup=%s",
               param->write.status, param->write.conn_id, param->write.handle, this->pending_cccd_handle_,
               this->notify_setup_to_string_(this->pending_notify_setup_));
      if (this->parent()->get_conn_id() != param->write.conn_id || param->write.handle != this->pending_cccd_handle_)
        return;
      this->cancel_timeout("command_timeout");
      if (param->write.status != ESP_GATT_OK) {
        this->invalidate_gatt_cache_("write notify descriptor failed");
        this->retry_current_operation_("write notify descriptor failed");
        return;
      }
      const NotifySetup completed_setup = this->pending_notify_setup_;
      this->pending_cccd_handle_ = 0;
      if (completed_setup == NotifySetup::SECURE) {
        this->pending_notify_setup_ = NotifySetup::NONE;
        this->begin_secure_handshake_();
      } else if (completed_setup == NotifySetup::NORMAL) {
        this->pending_notify_setup_ = NotifySetup::NONE;
        this->session_authenticated_ = true;
        this->run_next_step_();
      }
      break;
    }

    case ESP_GATTC_NOTIFY_EVT: {
      if (this->parent()->get_conn_id() != param->notify.conn_id)
        return;
      std::vector<uint8_t> data(param->notify.value, param->notify.value + param->notify.value_len);
      if (param->notify.handle == this->secure_read_handle_) {
        this->last_secure_notify_ms_ = millis();
        auto decrypted = data;
        if (!this->decrypt_secure_(decrypted)) {
          this->retry_current_operation_("secure decrypt failed");
          return;
        }
        if (this->pending_response_channel_ == ResponseChannel::SECURE_NOTIFY)
          this->handle_secure_response_(data, decrypted);
      } else if (param->notify.handle == this->normal_read_handle_) {
        this->last_normal_notify_ms_ = millis();
        auto decrypted = data;
        if (!this->decrypt_normal_(decrypted)) {
          this->retry_current_operation_("normal decrypt failed");
          return;
        }
        if (this->pending_response_channel_ == ResponseChannel::NORMAL_NOTIFY)
          this->handle_normal_response_(decrypted);
        else
          this->handle_unsolicited_normal_(decrypted);
      }
      break;
    }

    case ESP_GATTC_READ_CHAR_EVT:
      if (this->parent()->get_conn_id() != param->read.conn_id || this->pending_response_channel_ != ResponseChannel::READ_CHAR ||
          param->read.handle != this->pending_read_handle_)
        return;
      this->cancel_timeout("command_timeout");
      if (param->read.status != ESP_GATT_OK) {
        this->invalidate_gatt_cache_("read characteristic failed");
        this->retry_current_operation_("read characteristic failed");
        return;
      }
      this->handle_info_response_(this->active_step_, param->read.value, param->read.value_len);
      this->pending_response_channel_ = ResponseChannel::NONE;
      this->active_step_ = StepType::NONE;
      this->run_next_step_();
      break;

    case ESP_GATTC_DISCONNECT_EVT: {
      if (!this->parent()->check_addr(param->disconnect.remote_bda))
        return;
      const bool shutdown_pending = this->active_step_ == StepType::SHUTDOWN;
      this->cancel_timeout("connect_timeout");
      this->cancel_timeout("command_timeout");
      this->cancel_timeout("cooldown");
      this->cancel_timeout("notify_register_fallback");
      this->reset_session_state_();
      if (this->ignore_next_disconnect_) {
        this->ignore_next_disconnect_ = false;
        return;
      }
      if (shutdown_pending) {
        this->finish_operation_and_disconnect_();
        return;
      }
      if (this->current_operation_ != OperationType::NONE)
        this->set_timeout("retry_after_disconnect", 250, [this]() { this->retry_current_operation_("disconnected"); });
      break;
    }

    default:
      break;
  }
}

void YaleXSBLE::gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  if (event == ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT &&
      memcmp(param->update_conn_params.bda, this->parent()->get_remote_bda(), sizeof(esp_bd_addr_t)) == 0) {
    ESP_LOGD(TAG, "Connection params updated: interval=%d latency=%d timeout=%d", param->update_conn_params.conn_int,
             param->update_conn_params.latency, param->update_conn_params.timeout);
  }
}

void YaleXSBLE::request_aggressive_connection_params_() {
  if (this->aggressive_conn_params_requested_ || this->parent() == nullptr)
    return;

  // 0x06 is the BLE minimum connection interval: 6 * 1.25 ms = 7.5 ms.
  esp_ble_conn_update_params_t conn_params = {};
  memcpy(conn_params.bda, this->parent()->get_remote_bda(), sizeof(esp_bd_addr_t));
  conn_params.min_int = 0x06;
  conn_params.max_int = 0x06;
  conn_params.latency = 0;
  conn_params.timeout = 400;
  const esp_err_t err = esp_ble_gap_update_conn_params(&conn_params);
  if (err == ESP_OK) {
    this->aggressive_conn_params_requested_ = true;
    ESP_LOGD(TAG, "Requested aggressive BLE connection params: interval=7.5ms latency=0 timeout=4000ms");
  } else {
    ESP_LOGW(TAG, "Failed to request aggressive BLE connection params: err=%d", err);
  }
}

void YaleXSBLE::on_connected_(bool services_discovered) {
  if (this->current_operation_ == OperationType::NONE || this->retry_pending_ || this->ignore_next_disconnect_)
    return;
  ESP_LOGD(TAG, "Connected; %s Yale GATT handles", services_discovered ? "resolving" : "using cached");
  this->ignore_next_disconnect_ = false;
  this->node_state = espbt::ClientState::ESTABLISHED;
  this->request_aggressive_connection_params_();

  if (services_discovered && !this->resolve_handles_()) {
    this->retry_current_operation_("missing Yale characteristics");
    return;
  }
  if (!services_discovered && !this->cached_gatt_handles_valid_()) {
    this->invalidate_gatt_cache_("cached handles unavailable");
    this->retry_current_operation_("cached Yale characteristics unavailable");
    return;
  }
  this->start_notify_(NotifySetup::SECURE);
}

bool YaleXSBLE::resolve_handles_() {
  this->clear_gatt_handles_();
  uint8_t props = 0;
  if (!this->find_characteristic_handles_(COMMAND_SERVICE_UUID, SECURE_READ_CHARACTERISTIC, &this->secure_read_handle_,
                                          &this->secure_read_properties_)) {
    ESP_LOGW(TAG, "Secure read characteristic not found");
    return false;
  }
  if (!this->find_characteristic_handles_(COMMAND_SERVICE_UUID, SECURE_WRITE_CHARACTERISTIC, &this->secure_write_handle_,
                                          &props)) {
    ESP_LOGW(TAG, "Secure write characteristic not found");
    return false;
  }
  if (!this->find_characteristic_handles_(COMMAND_SERVICE_UUID, READ_CHARACTERISTIC, &this->normal_read_handle_,
                                          &this->normal_read_properties_)) {
    ESP_LOGW(TAG, "Normal read characteristic not found");
    return false;
  }
  if (!this->find_characteristic_handles_(COMMAND_SERVICE_UUID, WRITE_CHARACTERISTIC, &this->normal_write_handle_,
                                          &props)) {
    ESP_LOGW(TAG, "Normal write characteristic not found");
    return false;
  }
  if (!this->find_characteristic_handle_any_service_(MANUFACTURER_NAME_CHARACTERISTIC, &this->manufacturer_handle_)) {
    ESP_LOGW(TAG, "Manufacturer characteristic not found");
    return false;
  }
  if (!this->find_characteristic_handle_any_service_(MODEL_NUMBER_CHARACTERISTIC, &this->model_handle_)) {
    ESP_LOGW(TAG, "Model characteristic not found");
    return false;
  }
  if (!this->find_characteristic_handle_any_service_(SERIAL_NUMBER_CHARACTERISTIC, &this->serial_handle_)) {
    ESP_LOGW(TAG, "Serial characteristic not found");
    return false;
  }
  if (!this->find_characteristic_handle_any_service_(FIRMWARE_REVISION_CHARACTERISTIC, &this->firmware_handle_)) {
    ESP_LOGW(TAG, "Firmware characteristic not found");
    return false;
  }
  if (!this->resolve_notify_descriptor_(this->secure_read_handle_, NotifySetup::SECURE, &this->secure_cccd_handle_))
    return false;
  if (!this->resolve_notify_descriptor_(this->normal_read_handle_, NotifySetup::NORMAL, &this->normal_cccd_handle_))
    return false;

  ESP_LOGD(TAG,
           "Yale handles: secure_read=0x%04x props=0x%02x secure_write=0x%04x normal_read=0x%04x props=0x%02x "
           "normal_write=0x%04x secure_cccd=0x%04x normal_cccd=0x%04x manufacturer=0x%04x model=0x%04x serial=0x%04x "
           "firmware=0x%04x",
           this->secure_read_handle_, this->secure_read_properties_, this->secure_write_handle_,
           this->normal_read_handle_, this->normal_read_properties_, this->normal_write_handle_,
           this->secure_cccd_handle_, this->normal_cccd_handle_, this->manufacturer_handle_, this->model_handle_,
           this->serial_handle_, this->firmware_handle_);
  this->gatt_handles_valid_ = true;
  return true;
}

bool YaleXSBLE::find_characteristic_handles_(const espbt::ESPBTUUID &service_uuid, const espbt::ESPBTUUID &char_uuid,
                                             uint16_t *handle, uint8_t *properties) {
  esp_gattc_service_elem_t service_result;
  uint16_t service_count = 1;
  esp_bt_uuid_t service_id = service_uuid.get_uuid();
  esp_gatt_status_t service_status = esp_ble_gattc_get_service(this->parent()->get_gattc_if(), this->parent()->get_conn_id(),
                                                               &service_id, &service_result, &service_count, 0);
  if (service_status != ESP_GATT_OK || service_count == 0) {
    ESP_LOGD(TAG, "Service lookup failed: status=%d count=%u", service_status, service_count);
    return false;
  }

  esp_gattc_char_elem_t char_result;
  uint16_t char_count = 1;
  esp_bt_uuid_t characteristic_id = char_uuid.get_uuid();
  esp_gatt_status_t char_status =
      esp_ble_gattc_get_char_by_uuid(this->parent()->get_gattc_if(), this->parent()->get_conn_id(),
                                     service_result.start_handle, service_result.end_handle, characteristic_id,
                                     &char_result, &char_count);
  if (char_status != ESP_GATT_OK || char_count == 0) {
    ESP_LOGD(TAG, "Characteristic lookup failed: status=%d count=%u service_start=0x%04x service_end=0x%04x",
             char_status, char_count, service_result.start_handle, service_result.end_handle);
    return false;
  }
  *handle = char_result.char_handle;
  if (properties != nullptr)
    *properties = char_result.properties;
  ESP_LOGD(TAG, "Resolved characteristic: handle=0x%04x props=0x%02x", char_result.char_handle,
           char_result.properties);
  return true;
}

bool YaleXSBLE::find_characteristic_handle_any_service_(const espbt::ESPBTUUID &char_uuid, uint16_t *handle) {
  uint16_t service_offset = 0;
  while (true) {
    esp_gattc_service_elem_t service_result;
    uint16_t service_count = 1;
    esp_gatt_status_t service_status = esp_ble_gattc_get_service(this->parent()->get_gattc_if(), this->parent()->get_conn_id(),
                                                                 nullptr, &service_result, &service_count, service_offset);
    if (service_status == ESP_GATT_INVALID_OFFSET || service_status == ESP_GATT_NOT_FOUND || service_count == 0) {
      ESP_LOGD(TAG, "Characteristic not found in any service: last_status=%d offset=%u", service_status, service_offset);
      return false;
    }
    if (service_status != ESP_GATT_OK) {
      ESP_LOGD(TAG, "Service iteration failed: status=%d offset=%u", service_status, service_offset);
      return false;
    }

    esp_gattc_char_elem_t char_result;
    uint16_t char_count = 1;
    esp_bt_uuid_t characteristic_id = char_uuid.get_uuid();
    esp_gatt_status_t char_status =
        esp_ble_gattc_get_char_by_uuid(this->parent()->get_gattc_if(), this->parent()->get_conn_id(),
                                       service_result.start_handle, service_result.end_handle, characteristic_id,
                                       &char_result, &char_count);
    if (char_status == ESP_GATT_OK && char_count > 0) {
      *handle = char_result.char_handle;
      ESP_LOGD(TAG, "Resolved info characteristic: handle=0x%04x props=0x%02x service_start=0x%04x service_end=0x%04x",
               char_result.char_handle, char_result.properties, service_result.start_handle, service_result.end_handle);
      return true;
    }
    service_offset++;
  }
}

bool YaleXSBLE::resolve_notify_descriptor_(uint16_t char_handle, NotifySetup setup, uint16_t *descriptor_handle) {
  esp_gattc_descr_elem_t desc_result;
  uint16_t count = 1;
  esp_gatt_status_t descr_status =
      esp_ble_gattc_get_descr_by_char_handle(this->parent()->get_gattc_if(), this->parent()->get_conn_id(), char_handle,
                                             NOTIFY_DESC_UUID, &desc_result, &count);
  if (descr_status != ESP_GATT_OK || count == 0) {
    ESP_LOGW(TAG, "Notify descriptor lookup failed: status=%d count=%u char_handle=0x%04x setup=%s", descr_status, count,
             char_handle, this->notify_setup_to_string_(setup));
    return false;
  }
  *descriptor_handle = desc_result.handle;
  ESP_LOGD(TAG, "Resolved %s CCCD: char=0x%04x cccd=0x%04x", this->notify_setup_to_string_(setup), char_handle,
           desc_result.handle);
  return true;
}

bool YaleXSBLE::cached_gatt_handles_valid_() const {
  return this->gatt_handles_valid_ && this->secure_read_handle_ != 0 && this->secure_write_handle_ != 0 &&
         this->normal_read_handle_ != 0 && this->normal_write_handle_ != 0 && this->secure_cccd_handle_ != 0 &&
         this->normal_cccd_handle_ != 0 && this->manufacturer_handle_ != 0 && this->model_handle_ != 0 &&
         this->serial_handle_ != 0 && this->firmware_handle_ != 0;
}

void YaleXSBLE::clear_gatt_handles_() {
  this->gatt_handles_valid_ = false;
  this->secure_read_handle_ = 0;
  this->secure_write_handle_ = 0;
  this->normal_read_handle_ = 0;
  this->normal_write_handle_ = 0;
  this->secure_read_properties_ = 0;
  this->normal_read_properties_ = 0;
  this->secure_cccd_handle_ = 0;
  this->normal_cccd_handle_ = 0;
  this->manufacturer_handle_ = 0;
  this->model_handle_ = 0;
  this->serial_handle_ = 0;
  this->firmware_handle_ = 0;
}

void YaleXSBLE::invalidate_gatt_cache_(const char *reason) {
  if (this->gatt_handles_valid_)
    ESP_LOGW(TAG, "Invalidating cached Yale GATT handles: %s", reason);
  this->clear_gatt_handles_();
  this->current_connection_uses_gatt_cache_ = false;
  if (this->parent() != nullptr)
    this->parent()->set_connection_type(espbt::ConnectionType::V3_WITHOUT_CACHE);
}

void YaleXSBLE::start_notify_(NotifySetup setup) {
  const uint16_t handle = setup == NotifySetup::SECURE ? this->secure_read_handle_ : this->normal_read_handle_;
  ESP_LOGD(TAG, "Starting %s notify registration on handle 0x%04x", this->notify_setup_to_string_(setup), handle);
  this->pending_notify_setup_ = setup;
  this->pending_notify_handle_ = handle;
  this->pending_cccd_handle_ = 0;
  this->set_timeout("command_timeout", this->command_timeout_ms_, [this]() { this->handle_command_timeout_(); });
  esp_err_t err = esp_ble_gattc_register_for_notify(this->parent()->get_gattc_if(), this->parent()->get_remote_bda(), handle);
  if (err != ESP_OK) {
    this->invalidate_gatt_cache_("notify registration call failed");
    ESP_LOGW(TAG, "Notify registration call failed: err=%d", err);
    this->retry_current_operation_("register notify call failed");
    return;
  }
  this->set_timeout("notify_register_fallback", 300, [this, setup, handle]() {
    if (this->current_operation_ == OperationType::NONE || this->pending_notify_setup_ != setup ||
        this->pending_notify_handle_ != handle || this->pending_cccd_handle_ != 0) {
      return;
    }
    ESP_LOGW(TAG, "Notify registration event did not arrive; writing CCCD directly for %s handle 0x%04x",
             this->notify_setup_to_string_(setup), handle);
    this->write_notify_descriptor_(handle, setup);
  });
}

bool YaleXSBLE::write_notify_descriptor_(uint16_t char_handle, NotifySetup setup) {
  uint16_t descriptor_handle = setup == NotifySetup::SECURE ? this->secure_cccd_handle_ : this->normal_cccd_handle_;
  if (descriptor_handle == 0 &&
      !this->resolve_notify_descriptor_(char_handle, setup, &descriptor_handle)) {
    this->invalidate_gatt_cache_("notify descriptor not found");
    this->retry_current_operation_("notify descriptor not found");
    return false;
  }
  if (setup == NotifySetup::SECURE)
    this->secure_cccd_handle_ = descriptor_handle;
  else if (setup == NotifySetup::NORMAL)
    this->normal_cccd_handle_ = descriptor_handle;

  const uint8_t properties = setup == NotifySetup::SECURE ? this->secure_read_properties_ : this->normal_read_properties_;
  uint16_t notify_en = 1;
  if ((properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY) != 0) {
    notify_en = 1;
  } else if ((properties & ESP_GATT_CHAR_PROP_BIT_INDICATE) != 0) {
    notify_en = 2;
  } else {
    ESP_LOGW(TAG, "Characteristic 0x%04x has no notify/indicate property bits set: props=0x%02x", char_handle,
             properties);
  }
  this->pending_cccd_handle_ = descriptor_handle;
  this->pending_notify_setup_ = setup;
  ESP_LOGD(TAG, "Writing %s CCCD: char=0x%04x cccd=0x%04x props=0x%02x value=%u",
           this->notify_setup_to_string_(setup), char_handle, descriptor_handle, properties, notify_en);
  this->cancel_timeout("command_timeout");
  this->set_timeout("command_timeout", this->command_timeout_ms_, [this]() { this->handle_command_timeout_(); });
  esp_err_t err = esp_ble_gattc_write_char_descr(this->parent()->get_gattc_if(), this->parent()->get_conn_id(),
                                                 descriptor_handle, sizeof(notify_en),
                                                 reinterpret_cast<uint8_t *>(&notify_en), ESP_GATT_WRITE_TYPE_RSP,
                                                 ESP_GATT_AUTH_REQ_NONE);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Notify descriptor write call failed: err=%d", err);
    this->invalidate_gatt_cache_("notify descriptor write call failed");
    this->retry_current_operation_("write notify descriptor call failed");
    return false;
  }
  return true;
}

void YaleXSBLE::begin_secure_handshake_() {
  this->set_secure_key_(this->key_);
  if (!random_bytes(this->handshake_keys_.data(), this->handshake_keys_.size())) {
    this->retry_current_operation_("failed to generate handshake keys");
    return;
  }
  this->execute_secure_command_(0x01, this->handshake_keys_.data(), 8, StepType::SECURE_AUTH_1,
                                "SEC_LOCK_TO_MOBILE_KEY_EXCHANGE");
}

void YaleXSBLE::set_secure_key_(const std::array<uint8_t, 16> &key) {
  mbedtls_aes_setkey_enc(&this->secure_encrypt_ctx_, key.data(), 128);
  mbedtls_aes_setkey_dec(&this->secure_decrypt_ctx_, key.data(), 128);
}

void YaleXSBLE::set_normal_key_(const std::array<uint8_t, 16> &key) {
  mbedtls_aes_setkey_enc(&this->normal_encrypt_ctx_, key.data(), 128);
  mbedtls_aes_setkey_dec(&this->normal_decrypt_ctx_, key.data(), 128);
  this->normal_encrypt_iv_.fill(0);
  this->normal_decrypt_iv_.fill(0);
}

void YaleXSBLE::run_next_step_() {
  if (this->current_operation_ == OperationType::NONE)
    return;
  if (!this->session_authenticated_) {
    return;
  }

  if (this->preempt_current_poll_ &&
      (this->current_operation_ == OperationType::UPDATE || this->current_operation_ == OperationType::REFRESH_DOOR)) {
    ESP_LOGD(TAG, "Preempting background %s for queued lock command",
             this->operation_to_string_(this->current_operation_));
    this->steps_.clear();
    this->operation_steps_built_ = true;
    this->preempt_current_poll_ = false;
    this->complete_current_operation_();
    return;
  }
  if (millis() - this->operation_started_ms_ > this->operation_timeout_ms_) {
    this->retry_current_operation_("operation timeout");
    return;
  }

  if (!this->lock_info_.valid) {
    if (this->lock_info_.manufacturer.empty()) {
      this->execute_step_(StepType::READ_INFO_MANUFACTURER);
      return;
    }
    if (this->lock_info_.model.empty()) {
      this->execute_step_(StepType::READ_INFO_MODEL);
      return;
    }
    if (this->lock_info_.serial.empty()) {
      this->execute_step_(StepType::READ_INFO_SERIAL);
      return;
    }
    if (this->lock_info_.firmware.empty()) {
      this->execute_step_(StepType::READ_INFO_FIRMWARE);
      return;
    }
    this->lock_info_.valid = true;
    ESP_LOGD(TAG, "Lock info: manufacturer=%s model=%s serial=%s firmware=%s", this->lock_info_.manufacturer.c_str(),
             this->lock_info_.model.c_str(), this->lock_info_.serial.c_str(), this->lock_info_.firmware.c_str());
  }

  if (!this->operation_steps_built_)
    this->build_operation_steps_();

  if (this->steps_.empty()) {
    this->complete_current_operation_();
    return;
  }

  const StepType step = this->steps_.front();
  this->steps_.pop_front();
  this->execute_step_(step);
}

void YaleXSBLE::build_operation_steps_() {
  this->operation_steps_built_ = true;
  const bool no_battery_model = this->lock_info_.model == "SL-103" || this->lock_info_.model == "CERES" ||
                                this->lock_info_.model == "Yale Linus L2";
  switch (this->current_operation_) {
    case OperationType::UPDATE:
      if (!no_battery_model && (this->next_battery_attempt_ms_ == 0 || millis() - this->next_battery_attempt_ms_ < 0x80000000UL))
        this->steps_.push_back(StepType::BATTERY_STATUS);
      if (this->lock_info_.door_sense())
        this->steps_.push_back(StepType::DOOR_STATUS);
      this->steps_.push_back(StepType::LOCK_STATUS);
      break;
    case OperationType::REFRESH_DOOR:
      this->steps_.push_back(StepType::DOOR_STATUS);
      break;
    case OperationType::LOCK:
      this->steps_.push_back(StepType::FORCE_LOCK);
      this->steps_.push_back(StepType::LOCK_STATUS);
      if (this->lock_info_.door_sense())
        this->steps_.push_back(StepType::DOOR_STATUS);
      break;
    case OperationType::UNLOCK:
      this->steps_.push_back(StepType::FORCE_UNLOCK);
      this->steps_.push_back(StepType::LOCK_STATUS);
      if (this->lock_info_.door_sense())
        this->steps_.push_back(StepType::DOOR_STATUS);
      break;
    case OperationType::NONE:
      break;
  }
}

void YaleXSBLE::execute_step_(StepType step) {
  switch (step) {
    case StepType::READ_INFO_MANUFACTURER:
      this->read_info_char_(step, this->manufacturer_handle_);
      break;
    case StepType::READ_INFO_MODEL:
      this->read_info_char_(step, this->model_handle_);
      break;
    case StepType::READ_INFO_SERIAL:
      this->read_info_char_(step, this->serial_handle_);
      break;
    case StepType::READ_INFO_FIRMWARE:
      this->read_info_char_(step, this->firmware_handle_);
      break;
    case StepType::BATTERY_STATUS:
      this->execute_normal_command_(COMMAND_GETSTATUS, STATUS_BATTERY, true, step, "battery");
      break;
    case StepType::DOOR_STATUS:
      this->execute_normal_command_(COMMAND_GETSTATUS, STATUS_DOOR_ONLY, true, step, "door_status");
      break;
    case StepType::LOCK_STATUS:
      this->execute_normal_command_(COMMAND_GETSTATUS, STATUS_LOCK_ONLY, true, step, "lock_status");
      break;
    case StepType::FORCE_LOCK:
      this->execute_normal_command_(COMMAND_LOCK, 0, false, step, "force_lock");
      break;
    case StepType::FORCE_UNLOCK:
      this->execute_normal_command_(COMMAND_UNLOCK, 0, false, step, "force_unlock");
      break;
    default:
      break;
  }
}

void YaleXSBLE::read_info_char_(StepType step, uint16_t handle) {
  this->active_step_ = step;
  this->pending_response_channel_ = ResponseChannel::READ_CHAR;
  this->pending_read_handle_ = handle;
  this->set_timeout("command_timeout", this->command_timeout_ms_, [this]() { this->handle_command_timeout_(); });
  esp_err_t err = esp_ble_gattc_read_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(), handle,
                                          ESP_GATT_AUTH_REQ_NONE);
  if (err != ESP_OK) {
    this->invalidate_gatt_cache_("read characteristic call failed");
    this->retry_current_operation_("read characteristic call failed");
  }
}

void YaleXSBLE::handle_info_response_(StepType step, const uint8_t *data, uint16_t len) {
  std::string value(reinterpret_cast<const char *>(data), reinterpret_cast<const char *>(data) + len);
  const auto null_pos = value.find('\0');
  if (null_pos != std::string::npos)
    value.resize(null_pos);
  switch (step) {
    case StepType::READ_INFO_MANUFACTURER:
      this->lock_info_.manufacturer = value;
      break;
    case StepType::READ_INFO_MODEL:
      this->lock_info_.model = value;
      break;
    case StepType::READ_INFO_SERIAL:
      this->lock_info_.serial = value;
      break;
    case StepType::READ_INFO_FIRMWARE:
      this->lock_info_.firmware = value;
      break;
    default:
      break;
  }
}

void YaleXSBLE::execute_secure_command_(uint8_t opcode, const uint8_t *payload, uint8_t payload_len, StepType step,
                                        const char *name) {
  auto command = this->build_secure_command_(opcode, payload, payload_len);
  if (!this->encrypt_secure_(command)) {
    this->retry_current_operation_("secure encrypt failed");
    return;
  }
  this->write_command_(ResponseChannel::SECURE_NOTIFY, this->secure_write_handle_, std::move(command), step, name);
}

void YaleXSBLE::execute_normal_command_(uint8_t opcode, uint8_t command_byte, bool has_command_byte, StepType step,
                                        const char *name) {
  auto command = this->build_normal_command_(opcode, command_byte, has_command_byte);
  if (!this->encrypt_normal_(command)) {
    this->retry_current_operation_("normal encrypt failed");
    return;
  }
  this->write_command_(ResponseChannel::NORMAL_NOTIFY, this->normal_write_handle_, std::move(command), step, name);
}

void YaleXSBLE::execute_shutdown_() {
  std::vector<uint8_t> command(18, 0);
  command[0] = 0x05;
  command[0x10] = 0x0F;
  command[0x11] = 0x00;
  put_u32_le_(command, 0x0C, security_checksum_(command));
  if (!this->encrypt_secure_(command)) {
    ESP_LOGD(TAG, "Secure shutdown encryption failed; disconnecting anyway");
    this->finish_operation_and_disconnect_();
    return;
  }
  this->session_shutting_down_ = true;
  this->write_command_(ResponseChannel::SECURE_NOTIFY, this->secure_write_handle_, std::move(command), StepType::SHUTDOWN,
                       "shutdown");
}

void YaleXSBLE::write_command_(ResponseChannel channel, uint16_t handle, std::vector<uint8_t> command, StepType step,
                               const char *name) {
  const uint32_t now = millis();
  uint32_t last_notify_ms = 0;
  if (this->session_authenticated_) {
    if (channel == ResponseChannel::SECURE_NOTIFY)
      last_notify_ms = this->last_secure_notify_ms_;
    else if (channel == ResponseChannel::NORMAL_NOTIFY)
      last_notify_ms = this->last_normal_notify_ms_;
  }
  if (last_notify_ms != 0 && now - last_notify_ms < REQUEST_COOLDOWN_MS) {
    const uint32_t wait = REQUEST_COOLDOWN_MS - (now - last_notify_ms);
    this->set_timeout("cooldown", wait, [this, channel, handle, command = std::move(command), step, name]() mutable {
      this->write_command_now_(channel, handle, std::move(command), step, name);
    });
    return;
  }
  this->write_command_now_(channel, handle, std::move(command), step, name);
}

void YaleXSBLE::write_command_now_(ResponseChannel channel, uint16_t handle, std::vector<uint8_t> command, StepType step,
                                   const char *name) {
  char hex_buf[format_hex_size(18)];
  ESP_LOGV(TAG, "Writing %s: %s", name, format_hex_to(hex_buf, command.data(), command.size()));
  this->active_step_ = step;
  this->pending_response_channel_ = channel;
  this->set_timeout("command_timeout", this->command_timeout_ms_, [this]() { this->handle_command_timeout_(); });
  esp_err_t err = esp_ble_gattc_write_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(), handle,
                                           command.size(), command.data(), ESP_GATT_WRITE_TYPE_RSP,
                                           ESP_GATT_AUTH_REQ_NONE);
  if (err != ESP_OK) {
    if (step == StepType::SHUTDOWN) {
      ESP_LOGD(TAG, "Secure shutdown write failed; disconnecting anyway");
      this->finish_operation_and_disconnect_();
      return;
    }
    this->invalidate_gatt_cache_("write characteristic call failed");
    this->retry_current_operation_("write characteristic call failed");
  }
}

void YaleXSBLE::handle_secure_response_(std::vector<uint8_t> encrypted, std::vector<uint8_t> decrypted) {
  (void) encrypted;
  this->cancel_timeout("command_timeout");
  this->pending_response_channel_ = ResponseChannel::NONE;
  const StepType step = this->active_step_;
  if (!validate_secure_response_(decrypted)) {
    if (step == StepType::SHUTDOWN) {
      ESP_LOGD(TAG, "Secure shutdown checksum failed; disconnecting anyway");
      this->finish_operation_and_disconnect_();
      return;
    }
    this->retry_current_operation_("secure response checksum failed");
    return;
  }

  if (step == StepType::SHUTDOWN) {
    if (decrypted.empty() || decrypted[0] != 0x8B)
      ESP_LOGD(TAG, "Unexpected secure shutdown response");
    this->finish_operation_and_disconnect_();
    return;
  }

  if (step == StepType::SECURE_AUTH_1) {
    if (decrypted.size() < 12 || decrypted[0] != 0x02) {
      this->retry_current_operation_("secure auth step 1 failed");
      return;
    }
    std::copy(this->handshake_keys_.begin(), this->handshake_keys_.begin() + 8, this->session_key_.begin());
    std::copy(decrypted.begin() + 4, decrypted.begin() + 12, this->session_key_.begin() + 8);
    this->set_secure_key_(this->session_key_);
    this->set_normal_key_(this->session_key_);
    this->execute_secure_command_(0x03, this->handshake_keys_.data() + 8, 8, StepType::SECURE_AUTH_2,
                                  "SEC_INITIALIZATION_COMMAND");
    return;
  }

  if (step == StepType::SECURE_AUTH_2) {
    if (decrypted.empty() || decrypted[0] != 0x04) {
      this->retry_current_operation_("secure auth step 2 failed");
      return;
    }
    this->start_notify_(NotifySetup::NORMAL);
    return;
  }
}

void YaleXSBLE::handle_normal_response_(std::vector<uint8_t> decrypted) {
  this->cancel_timeout("command_timeout");
  this->pending_response_channel_ = ResponseChannel::NONE;
  if (!validate_simple_response_(decrypted)) {
    this->retry_current_operation_("normal response checksum failed");
    return;
  }

  const bool discard_preempted_poll = this->preempt_current_poll_ &&
                                      (this->current_operation_ == OperationType::UPDATE ||
                                       this->current_operation_ == OperationType::REFRESH_DOOR);
  if (!discard_preempted_poll) {
    this->parse_state_response_(decrypted);
    if (this->active_step_ == StepType::BATTERY_STATUS) {
      const auto battery = this->parse_battery_state_(decrypted);
      if (battery.valid && battery.voltage <= 3.0f) {
        this->update_battery_(battery);
        this->retry_current_operation_("impossible battery voltage");
        return;
      }
      this->update_battery_(battery);
      this->next_battery_attempt_ms_ = 0;
    }

    if (this->active_step_ == StepType::LOCK_STATUS && this->is_bad_lock_state_(this->lock_status_)) {
      this->retry_current_operation_("bad lock state");
      return;
    }
  } else {
    ESP_LOGD(TAG, "Discarding response from preempted background poll");
  }

  this->active_step_ = StepType::NONE;
  this->run_next_step_();
}

void YaleXSBLE::handle_unsolicited_normal_(const std::vector<uint8_t> &decrypted) { this->parse_state_response_(decrypted); }

void YaleXSBLE::handle_command_timeout_() {
  ESP_LOGW(TAG, "Command timeout: operation=%s attempt=%u/%u step=%s response=%s notify_setup=%s notify=0x%04x cccd=0x%04x",
           this->operation_to_string_(this->current_operation_), this->current_attempt_, this->operation_retries_ + 1,
           this->step_to_string_(this->active_step_),
           this->response_channel_to_string_(this->pending_response_channel_),
           this->notify_setup_to_string_(this->pending_notify_setup_), this->pending_notify_handle_,
           this->pending_cccd_handle_);
  if (this->active_step_ == StepType::SHUTDOWN) {
    ESP_LOGD(TAG, "Secure shutdown timed out; disconnecting anyway");
    this->finish_operation_and_disconnect_();
    return;
  }
  if (this->active_step_ == StepType::BATTERY_STATUS)
    this->next_battery_attempt_ms_ = millis() + BATTERY_TIMEOUT_COOLDOWN_MS;
  if (this->current_connection_uses_gatt_cache_)
    this->invalidate_gatt_cache_("cached GATT command timeout");
  this->retry_current_operation_("command timeout");
}

std::vector<uint8_t> YaleXSBLE::build_secure_command_(uint8_t opcode, const uint8_t *payload, uint8_t payload_len) {
  std::vector<uint8_t> command(18, 0);
  command[0] = opcode;
  command[0x10] = 0x0F;
  command[0x11] = this->slot_;
  if (payload != nullptr && payload_len > 0) {
    const uint8_t len = std::min<uint8_t>(payload_len, 8);
    memcpy(command.data() + 4, payload, len);
  }
  put_u32_le_(command, 0x0C, security_checksum_(command));
  return command;
}

std::vector<uint8_t> YaleXSBLE::build_normal_command_(uint8_t opcode, uint8_t command_byte, bool has_command_byte) {
  std::vector<uint8_t> command(18, 0);
  command[0] = 0xEE;
  command[1] = opcode;
  command[0x10] = 0x02;
  if (has_command_byte)
    command[4] = command_byte;
  command[3] = simple_checksum_(command);
  return command;
}

bool YaleXSBLE::encrypt_secure_(std::vector<uint8_t> &command) {
  uint8_t out[16];
  if (mbedtls_aes_crypt_ecb(&this->secure_encrypt_ctx_, ESP_AES_ENCRYPT, command.data(), out) != 0)
    return false;
  memcpy(command.data(), out, 16);
  return true;
}

bool YaleXSBLE::decrypt_secure_(std::vector<uint8_t> &data) {
  if (data.size() < 16)
    return false;
  uint8_t out[16];
  if (mbedtls_aes_crypt_ecb(&this->secure_decrypt_ctx_, ESP_AES_DECRYPT, data.data(), out) != 0)
    return false;
  memcpy(data.data(), out, 16);
  return true;
}

bool YaleXSBLE::encrypt_normal_(std::vector<uint8_t> &command) {
  if (command.size() < 16)
    return false;
  uint8_t block[16];
  uint8_t out[16];
  for (uint8_t i = 0; i < 16; i++)
    block[i] = command[i] ^ this->normal_encrypt_iv_[i];
  if (mbedtls_aes_crypt_ecb(&this->normal_encrypt_ctx_, ESP_AES_ENCRYPT, block, out) != 0)
    return false;
  memcpy(command.data(), out, 16);
  memcpy(this->normal_encrypt_iv_.data(), out, 16);
  return true;
}

bool YaleXSBLE::decrypt_normal_(std::vector<uint8_t> &data) {
  if (data.size() < 16)
    return false;
  uint8_t cipher[16];
  uint8_t out[16];
  memcpy(cipher, data.data(), 16);
  if (mbedtls_aes_crypt_ecb(&this->normal_decrypt_ctx_, ESP_AES_DECRYPT, cipher, out) != 0)
    return false;
  for (uint8_t i = 0; i < 16; i++)
    out[i] ^= this->normal_decrypt_iv_[i];
  memcpy(data.data(), out, 16);
  memcpy(this->normal_decrypt_iv_.data(), cipher, 16);
  return true;
}

uint8_t YaleXSBLE::simple_checksum_(const std::vector<uint8_t> &buffer) {
  uint8_t checksum = 0;
  for (size_t i = 0; i < 18 && i < buffer.size(); i++)
    checksum = static_cast<uint8_t>(checksum + buffer[i]);
  return static_cast<uint8_t>(-checksum);
}

uint32_t YaleXSBLE::security_checksum_(const std::vector<uint8_t> &buffer) {
  if (buffer.size() < 12)
    return 0;
  uint32_t val1 = static_cast<uint32_t>(buffer[0]) | (static_cast<uint32_t>(buffer[1]) << 8) |
                  (static_cast<uint32_t>(buffer[2]) << 16) | (static_cast<uint32_t>(buffer[3]) << 24);
  uint32_t val2 = static_cast<uint32_t>(buffer[4]) | (static_cast<uint32_t>(buffer[5]) << 8) |
                  (static_cast<uint32_t>(buffer[6]) << 16) | (static_cast<uint32_t>(buffer[7]) << 24);
  uint32_t val3 = static_cast<uint32_t>(buffer[8]) | (static_cast<uint32_t>(buffer[9]) << 8) |
                  (static_cast<uint32_t>(buffer[10]) << 16) | (static_cast<uint32_t>(buffer[11]) << 24);
  return static_cast<uint32_t>(0 - (static_cast<uint64_t>(val1) + val2 + val3));
}

uint16_t YaleXSBLE::bytes_to_u16_(const std::vector<uint8_t> &buffer, uint8_t offset) {
  if (buffer.size() <= offset + 1)
    return 0;
  return static_cast<uint16_t>(buffer[offset]) | (static_cast<uint16_t>(buffer[offset + 1]) << 8);
}

void YaleXSBLE::put_u32_le_(std::vector<uint8_t> &buffer, uint8_t offset, uint32_t value) {
  buffer[offset] = value & 0xFF;
  buffer[offset + 1] = (value >> 8) & 0xFF;
  buffer[offset + 2] = (value >> 16) & 0xFF;
  buffer[offset + 3] = (value >> 24) & 0xFF;
}

bool YaleXSBLE::validate_simple_response_(const std::vector<uint8_t> &response) {
  if (response.size() < 18)
    return false;
  if (response[0] != 0xBB && response[0] != 0xAA)
    return false;
  uint8_t sum = 0;
  for (size_t i = 0; i < 18; i++)
    sum = static_cast<uint8_t>(sum + response[i]);
  return sum == 0;
}

bool YaleXSBLE::validate_secure_response_(const std::vector<uint8_t> &response) {
  if (response.size() < 18)
    return false;
  const uint32_t response_checksum = static_cast<uint32_t>(response[0x0C]) |
                                     (static_cast<uint32_t>(response[0x0D]) << 8) |
                                     (static_cast<uint32_t>(response[0x0E]) << 16) |
                                     (static_cast<uint32_t>(response[0x0F]) << 24);
  return response_checksum == security_checksum_(response);
}

void YaleXSBLE::parse_state_response_(const std::vector<uint8_t> &response) {
  if (response.size() < 9)
    return;
  if (response[0] == 0xBB) {
    if ((response[1] == COMMAND_LOCK || response[1] == COMMAND_UNLOCK) && response.size() > 3) {
      this->update_lock_status_(this->parse_lock_status_(response[3]));
      return;
    }
    if (response[1] == COMMAND_LOCK_ACTIVITY)
      return;
    if (response[1] == COMMAND_GETSTATUS) {
      if (response[4] == STATUS_LOCK_ONLY) {
        this->update_lock_status_(this->parse_lock_status_(response[8]));
      } else if (response[4] == STATUS_DOOR_ONLY) {
        this->update_door_status_(this->parse_door_status_(response[8]));
      } else if (response[4] == STATUS_DOOR_AND_LOCK) {
        this->parse_lock_and_door_state_(response);
      }
    }
  } else if (response[0] == 0xAA) {
    if (response[1] == COMMAND_UNLOCK)
      this->update_lock_status_(YaleLockStatus::UNLOCKED);
    else if (response[1] == COMMAND_LOCK)
      this->update_lock_status_(YaleLockStatus::LOCKED);
  }
}

void YaleXSBLE::parse_lock_and_door_state_(const std::vector<uint8_t> &response) {
  if (response.size() < 10)
    return;
  this->update_lock_status_(this->parse_lock_status_(response[8]));
  this->update_door_status_(this->parse_door_status_(response[9]));
}

YaleLockStatus YaleXSBLE::parse_lock_status_(uint8_t value) const {
  switch (value) {
    case 0x01:
      return YaleLockStatus::UNKNOWN_01;
    case 0x02:
      return YaleLockStatus::UNLOCKING;
    case 0x03:
      return YaleLockStatus::UNLOCKED;
    case 0x04:
      return YaleLockStatus::LOCKING;
    case 0x05:
      return YaleLockStatus::LOCKED;
    case 0x06:
      return YaleLockStatus::UNKNOWN_06;
    case 0x0C:
      return YaleLockStatus::SECUREMODE;
    case 0x1B:
      return YaleLockStatus::JAMMED;
    default:
      return YaleLockStatus::UNKNOWN;
  }
}

YaleDoorStatus YaleXSBLE::parse_door_status_(uint8_t value) const {
  switch (value) {
    case 0x01:
      return YaleDoorStatus::CLOSED;
    case 0x02:
      return YaleDoorStatus::AJAR;
    case 0x03:
      return YaleDoorStatus::OPENED;
    case 0x04:
      return YaleDoorStatus::UNKNOWN_04;
    default:
      return YaleDoorStatus::UNKNOWN;
  }
}

YaleBatteryState YaleXSBLE::parse_battery_state_(const std::vector<uint8_t> &response) const {
  YaleBatteryState state;
  if (response.size() < 10 || response[0] != 0xBB || response[1] != COMMAND_GETSTATUS || response[4] != STATUS_BATTERY)
    return state;
  state.valid = true;
  state.voltage = bytes_to_u16_(response, 8) / 1000.0f;
  state.percentage = convert_voltage_to_percentage_(state.voltage / 4.0f);
  return state;
}

uint8_t YaleXSBLE::convert_voltage_to_percentage_(float single_cell_voltage) {
  struct Point {
    float voltage;
    uint8_t percent;
  };
  static const Point points[] = {
      {1.24f, 0},  {1.25f, 5},  {1.26f, 10}, {1.27f, 15}, {1.28f, 20}, {1.29f, 25},
      {1.30f, 30}, {1.31f, 35}, {1.32f, 30}, {1.33f, 35}, {1.34f, 40}, {1.35f, 45},
      {1.36f, 50}, {1.37f, 55}, {1.38f, 60}, {1.39f, 65}, {1.40f, 70}, {1.45f, 75},
      {1.46f, 80}, {1.471f, 85}, {1.49f, 90}, {1.49075f, 93}, {1.547f, 94},
      {1.548f, 95}, {1.549f, 97}, {1.55f, 100},
  };
  size_t pos = 0;
  while (pos < sizeof(points) / sizeof(points[0]) && points[pos].voltage < single_cell_voltage)
    pos++;
  if (pos != 0)
    pos--;
  return points[pos].percent;
}

void YaleXSBLE::update_lock_status_(YaleLockStatus status) {
  if (this->lock_status_ == status)
    return;
  this->lock_status_ = status;
  this->lock_seen_this_session_ = true;
  this->last_state_change_ms_ = millis();
  this->publish_lock_state_();
}

void YaleXSBLE::update_door_status_(YaleDoorStatus status) {
  if (this->door_status_ == status)
    return;
  this->door_status_ = status;
  this->door_seen_this_session_ = true;
  this->last_state_change_ms_ = millis();
  this->publish_door_state_();
}

void YaleXSBLE::update_battery_(const YaleBatteryState &battery) {
  if (!battery.valid)
    return;
  this->battery_ = battery;
  this->battery_seen_this_session_ = true;
  this->publish_battery_state_();
}

void YaleXSBLE::publish_lock_state_() {
  if (this->lock_entity_ == nullptr)
    return;
  if (this->is_jammed_state_(this->lock_status_)) {
    this->lock_entity_->publish_state(lock::LOCK_STATE_JAMMED);
    return;
  }
  switch (this->lock_status_) {
    case YaleLockStatus::LOCKED:
    case YaleLockStatus::SECUREMODE:
      this->lock_entity_->publish_state(lock::LOCK_STATE_LOCKED);
      break;
    case YaleLockStatus::UNLOCKED:
      this->lock_entity_->publish_state(lock::LOCK_STATE_UNLOCKED);
      break;
    case YaleLockStatus::LOCKING:
      this->lock_entity_->publish_state(lock::LOCK_STATE_LOCKING);
      break;
    case YaleLockStatus::UNLOCKING:
      this->lock_entity_->publish_state(lock::LOCK_STATE_UNLOCKING);
      break;
    default:
      this->lock_entity_->publish_state(lock::LOCK_STATE_NONE);
      break;
  }
}

void YaleXSBLE::publish_door_state_() {
  if (this->door_status_sensor_ != nullptr)
    this->door_status_sensor_->publish_state(this->door_status_to_string_(this->door_status_));
  if (this->door_binary_sensor_ == nullptr)
    return;
  if (this->door_status_ == YaleDoorStatus::OPENED)
    this->door_binary_sensor_->publish_state(true);
  else if (this->door_status_ == YaleDoorStatus::CLOSED)
    this->door_binary_sensor_->publish_state(false);
  else
    this->door_binary_sensor_->invalidate_state();
}

void YaleXSBLE::publish_battery_state_() {
  if (!this->battery_.valid)
    return;
  if (this->battery_level_sensor_ != nullptr)
    this->battery_level_sensor_->publish_state(this->battery_.percentage);
  if (this->battery_voltage_sensor_ != nullptr)
    this->battery_voltage_sensor_->publish_state(this->battery_.voltage);
}

bool YaleXSBLE::is_expected_complete_() const {
  if (this->current_operation_ == OperationType::LOCK)
    return this->lock_status_ == YaleLockStatus::LOCKED || this->lock_status_ == YaleLockStatus::SECUREMODE;
  if (this->current_operation_ == OperationType::UNLOCK)
    return this->lock_status_ == YaleLockStatus::UNLOCKED;
  return true;
}

bool YaleXSBLE::is_bad_lock_state_(YaleLockStatus status) const {
  return status == YaleLockStatus::UNKNOWN_01 || status == YaleLockStatus::UNKNOWN_06;
}

bool YaleXSBLE::is_jammed_state_(YaleLockStatus status) const {
  return status == YaleLockStatus::UNKNOWN_01 || status == YaleLockStatus::UNKNOWN_06 ||
         status == YaleLockStatus::JAMMED;
}

const char *YaleXSBLE::lock_status_to_string_(YaleLockStatus status) const {
  switch (status) {
    case YaleLockStatus::UNKNOWN_01:
      return "unknown_01";
    case YaleLockStatus::UNLOCKING:
      return "unlocking";
    case YaleLockStatus::UNLOCKED:
      return "unlocked";
    case YaleLockStatus::LOCKING:
      return "locking";
    case YaleLockStatus::LOCKED:
      return "locked";
    case YaleLockStatus::UNKNOWN_06:
      return "unknown_06";
    case YaleLockStatus::SECUREMODE:
      return "securemode";
    case YaleLockStatus::JAMMED:
      return "jammed";
    default:
      return "unknown";
  }
}

const char *YaleXSBLE::door_status_to_string_(YaleDoorStatus status) const {
  switch (status) {
    case YaleDoorStatus::CLOSED:
      return "closed";
    case YaleDoorStatus::AJAR:
      return "ajar";
    case YaleDoorStatus::OPENED:
      return "opened";
    case YaleDoorStatus::UNKNOWN_04:
      return "unknown_04";
    default:
      return "unknown";
  }
}

const char *YaleXSBLE::operation_to_string_(OperationType operation) const {
  switch (operation) {
    case OperationType::UPDATE:
      return "update";
    case OperationType::REFRESH_DOOR:
      return "refresh_door";
    case OperationType::LOCK:
      return "lock";
    case OperationType::UNLOCK:
      return "unlock";
    default:
      return "none";
  }
}

const char *YaleXSBLE::step_to_string_(StepType step) const {
  switch (step) {
    case StepType::READ_INFO_MANUFACTURER:
      return "read_info_manufacturer";
    case StepType::READ_INFO_MODEL:
      return "read_info_model";
    case StepType::READ_INFO_SERIAL:
      return "read_info_serial";
    case StepType::READ_INFO_FIRMWARE:
      return "read_info_firmware";
    case StepType::SECURE_AUTH_1:
      return "secure_auth_1";
    case StepType::SECURE_AUTH_2:
      return "secure_auth_2";
    case StepType::BATTERY_STATUS:
      return "battery_status";
    case StepType::DOOR_STATUS:
      return "door_status";
    case StepType::LOCK_STATUS:
      return "lock_status";
    case StepType::FORCE_LOCK:
      return "force_lock";
    case StepType::FORCE_UNLOCK:
      return "force_unlock";
    case StepType::SHUTDOWN:
      return "shutdown";
    default:
      return "none";
  }
}

const char *YaleXSBLE::notify_setup_to_string_(NotifySetup setup) const {
  switch (setup) {
    case NotifySetup::SECURE:
      return "secure";
    case NotifySetup::NORMAL:
      return "normal";
    default:
      return "none";
  }
}

const char *YaleXSBLE::response_channel_to_string_(ResponseChannel channel) const {
  switch (channel) {
    case ResponseChannel::SECURE_NOTIFY:
      return "secure_notify";
    case ResponseChannel::NORMAL_NOTIFY:
      return "normal_notify";
    case ResponseChannel::READ_CHAR:
      return "read_char";
    default:
      return "none";
  }
}

}  // namespace esphome::yalexs_ble

#endif  // USE_ESP32
