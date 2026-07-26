import esphome.codegen as cg
from esphome.components import (
    binary_sensor,
    ble_client,
    button,
    esp32_ble,
    esp32_ble_tracker,
    lock,
    sensor,
    text_sensor,
)
import esphome.config_validation as cv
import esphome.final_validate as fv
from esphome.const import (
    CONF_BATTERY_LEVEL,
    CONF_BATTERY_VOLTAGE,
    CONF_ID,
    CONF_KEY,
    DEVICE_CLASS_BATTERY,
    DEVICE_CLASS_DOOR,
    DEVICE_CLASS_SIGNAL_STRENGTH,
    DEVICE_CLASS_VOLTAGE,
    ENTITY_CATEGORY_DIAGNOSTIC,
    ICON_BLUETOOTH,
    STATE_CLASS_MEASUREMENT,
    UNIT_DECIBEL_MILLIWATT,
    UNIT_PERCENT,
    UNIT_VOLT,
)

CODEOWNERS = ["@splitice"]
DEPENDENCIES = ["ble_client", "esp32_ble_tracker"]
AUTO_LOAD = ["binary_sensor", "button", "lock", "sensor", "text_sensor"]
MULTI_CONF = True

CONF_DOOR = "door"
CONF_DOOR_STATUS = "door_status"
CONF_LAST_SEEN_BROADCAST = "last_seen_broadcast"
CONF_LOCK = "lock"
CONF_LOCAL_NAME = "local_name"
CONF_OPERATION_RETRIES = "operation_retries"
CONF_CONNECT_TIMEOUT = "connect_timeout"
CONF_COMMAND_TIMEOUT = "command_timeout"
CONF_OPERATION_TIMEOUT = "operation_timeout"
CONF_REFRESH_DOOR_STATUS = "refresh_door_status"
CONF_RSSI = "rssi"
CONF_SLOT = "slot"

yalexs_ble_ns = cg.esphome_ns.namespace("yalexs_ble")
YaleXSBLE = yalexs_ble_ns.class_(
    "YaleXSBLE",
    cg.Component,
    ble_client.BLEClientNode,
    esp32_ble_tracker.ESPBTDeviceListener,
)
YaleXSBLELock = yalexs_ble_ns.class_("YaleXSBLELock", lock.Lock)
YaleXSBLERefreshButton = yalexs_ble_ns.class_(
    "YaleXSBLERefreshButton", button.Button
)

ConnectionType = esp32_ble_tracker.esp32_ble_tracker_ns.enum(
    "ConnectionType", is_class=True
)


def validate_key(value):
    value = cv.string_strict(value)
    if len(value) != 32:
        raise cv.Invalid("key must be exactly 32 hexadecimal characters")
    try:
        bytes.fromhex(value)
    except ValueError as err:
        raise cv.Invalid("key must be hexadecimal") from err
    return value.lower()


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(YaleXSBLE),
            cv.Required(CONF_KEY): validate_key,
            cv.Required(CONF_SLOT): cv.int_range(min=0, max=255),
            cv.Optional(CONF_LOCAL_NAME): cv.string,
            cv.Optional(CONF_OPERATION_RETRIES, default=5): cv.int_range(
                min=0, max=20
            ),
            cv.Optional(CONF_CONNECT_TIMEOUT, default="8s"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_COMMAND_TIMEOUT, default="10s"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_OPERATION_TIMEOUT, default="20s"): cv.positive_time_period_milliseconds,
            cv.Required(CONF_LOCK): lock.lock_schema(YaleXSBLELock),
            cv.Optional(CONF_REFRESH_DOOR_STATUS): button.button_schema(
                YaleXSBLERefreshButton,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                icon="mdi:refresh",
            ),
            cv.Optional(CONF_DOOR): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_DOOR
            ),
            cv.Optional(CONF_DOOR_STATUS): text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                icon="mdi:door",
            ),
            cv.Optional(CONF_LAST_SEEN_BROADCAST): text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                icon=ICON_BLUETOOTH,
            ),
            cv.Optional(CONF_BATTERY_LEVEL): sensor.sensor_schema(
                unit_of_measurement=UNIT_PERCENT,
                device_class=DEVICE_CLASS_BATTERY,
                state_class=STATE_CLASS_MEASUREMENT,
                accuracy_decimals=0,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_BATTERY_VOLTAGE): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                device_class=DEVICE_CLASS_VOLTAGE,
                state_class=STATE_CLASS_MEASUREMENT,
                accuracy_decimals=3,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_RSSI): sensor.sensor_schema(
                unit_of_measurement=UNIT_DECIBEL_MILLIWATT,
                device_class=DEVICE_CLASS_SIGNAL_STRENGTH,
                state_class=STATE_CLASS_MEASUREMENT,
                accuracy_decimals=0,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(ble_client.BLE_CLIENT_SCHEMA)
    .extend(esp32_ble_tracker.ESP_BLE_DEVICE_SCHEMA),
    esp32_ble.consume_connection_slots(1, "yalexs_ble"),
)


def _final_validate(config):
    full_config = fv.full_config.get()
    client_path = full_config.get_path_for_id(config[ble_client.CONF_BLE_CLIENT_ID])[
        :-1
    ]
    client_config = full_config.get_config_for_path(client_path)
    if client_config.get(ble_client.CONF_AUTO_CONNECT, True):
        raise cv.Invalid(
            "yalexs_ble requires the parent ble_client to be configured with "
            "auto_connect: false"
        )
    return config


FINAL_VALIDATE_SCHEMA = _final_validate


async def to_code(config):
    cg.add_define("USE_ESP32_BLE_UUID")

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await ble_client.register_ble_node(var, config)
    await esp32_ble_tracker.register_ble_device(var, config)

    parent = await cg.get_variable(config[ble_client.CONF_BLE_CLIENT_ID])
    cg.add(parent.set_auto_connect(False))
    cg.add(parent.set_connection_type(ConnectionType.V3_WITHOUT_CACHE))

    cg.add(var.set_key(config[CONF_KEY]))
    cg.add(var.set_slot(config[CONF_SLOT]))
    cg.add(var.set_operation_retries(config[CONF_OPERATION_RETRIES]))
    cg.add(var.set_connect_timeout(config[CONF_CONNECT_TIMEOUT]))
    cg.add(var.set_command_timeout(config[CONF_COMMAND_TIMEOUT]))
    cg.add(var.set_operation_timeout(config[CONF_OPERATION_TIMEOUT]))
    if local_name := config.get(CONF_LOCAL_NAME):
        cg.add(var.set_local_name(local_name))

    lock_var = await lock.new_lock(config[CONF_LOCK])
    cg.add(lock_var.set_parent(var))
    cg.add(var.set_lock(lock_var))

    if refresh_config := config.get(CONF_REFRESH_DOOR_STATUS):
        refresh = await button.new_button(refresh_config)
        cg.add(refresh.set_parent(var))
        cg.add(var.set_refresh_button(refresh))

    if door_config := config.get(CONF_DOOR):
        door = await binary_sensor.new_binary_sensor(door_config)
        cg.add(var.set_door_binary_sensor(door))

    if door_status_config := config.get(CONF_DOOR_STATUS):
        door_status = await text_sensor.new_text_sensor(door_status_config)
        cg.add(var.set_door_status_text_sensor(door_status))

    if last_seen_config := config.get(CONF_LAST_SEEN_BROADCAST):
        last_seen = await text_sensor.new_text_sensor(last_seen_config)
        cg.add(var.set_last_seen_broadcast_text_sensor(last_seen))

    if battery_level_config := config.get(CONF_BATTERY_LEVEL):
        battery_level = await sensor.new_sensor(battery_level_config)
        cg.add(var.set_battery_level_sensor(battery_level))

    if battery_voltage_config := config.get(CONF_BATTERY_VOLTAGE):
        battery_voltage = await sensor.new_sensor(battery_voltage_config)
        cg.add(var.set_battery_voltage_sensor(battery_voltage))

    if rssi_config := config.get(CONF_RSSI):
        rssi = await sensor.new_sensor(rssi_config)
        cg.add(var.set_rssi_sensor(rssi))
