# ESPHome Yale XS BLE

External ESPHome component for Yale/August XS BLE locks. It ports the local
Bluetooth protocol behavior from Home Assistant's `yalexs_ble` integration and
the upstream `yalexs-ble==3.2.4` Python package.

This initial version uses manual credentials in YAML and exposes:

- Lock/unlock
- Door binary state and detailed door status text
- Refresh door status button
- Last seen BLE broadcast text
- Battery level, battery voltage, and RSSI diagnostics

## Example

```yaml
external_components:
  - source:
      type: local
      path: components

esp32_ble_tracker:
  scan_parameters:
    active: false
    continuous: true

ble_client:
  - id: yale_ble_client
    mac_address: AA:BB:CC:DD:EE:FF
    auto_connect: false

yalexs_ble:
  id: front_door_yale
  ble_client_id: yale_ble_client
  local_name: "AB12345"
  key: !secret yale_ble_key
  slot: 1

  lock:
    name: Front Door Lock
  refresh_door_status:
    name: Front Door Refresh Door Status
  door:
    name: Front Door
  door_status:
    name: Front Door Status
  last_seen_broadcast:
    name: Front Door Last Broadcast
  battery_level:
    name: Front Door Battery
  battery_voltage:
    name: Front Door Battery Voltage
  rssi:
    name: Front Door RSSI
```

## License

GPL-3.0-only. Protocol logic is derived from `yalexs-ble==3.2.4`, which is also
GPL-3.0-only.
