import bisect
import unittest


COMMAND_GETSTATUS = 0x02
COMMAND_LOCK = 0x0B
COMMAND_UNLOCK = 0x0A
STATUS_BATTERY = 0x0F
STATUS_DOOR_ONLY = 0x2E
STATUS_LOCK_ONLY = 0x02

FIRST_UPDATE_COALESCE_MS = 10
ADV_UPDATE_COALESCE_MS = 50
HK_UPDATE_COALESCE_MS = 25


def simple_checksum(buffer: bytes) -> int:
    return (-sum(buffer[:0x12])) & 0xFF


def security_checksum(buffer: bytes) -> int:
    val1 = int.from_bytes(buffer[0x00:0x04], "little")
    val2 = int.from_bytes(buffer[0x04:0x08], "little")
    val3 = int.from_bytes(buffer[0x08:0x12], "little")
    return (0 - (val1 + val2 + val3)) & 0xFFFFFFFF


def build_secure_command(opcode: int, payload: bytes = b"", slot: int = 1) -> bytes:
    command = bytearray(0x12)
    command[0] = opcode
    command[0x10] = 0x0F
    command[0x11] = slot
    command[4 : 4 + min(len(payload), 8)] = payload[:8]
    command[0x0C:0x10] = security_checksum(command).to_bytes(4, "little")
    return bytes(command)


def build_normal_command(opcode: int, command_byte: int | None = None) -> bytes:
    command = bytearray(0x12)
    command[0] = 0xEE
    command[1] = opcode
    command[0x10] = 0x02
    if command_byte is not None:
        command[4] = command_byte
    command[3] = simple_checksum(command)
    return bytes(command)


AA_BATTERY_VOLTAGE_TO_PERCENTAGE = (
    (1.55, 100),
    (1.549, 97),
    (1.548, 95),
    (1.547, 94),
    (1.49075, 93),
    (1.49, 90),
    (1.471, 85),
    (1.46, 80),
    (1.45, 75),
    (1.40, 70),
    (1.39, 65),
    (1.38, 60),
    (1.37, 55),
    (1.36, 50),
    (1.35, 45),
    (1.34, 40),
    (1.33, 35),
    (1.32, 30),
    (1.31, 35),
    (1.30, 30),
    (1.29, 25),
    (1.28, 20),
    (1.27, 15),
    (1.26, 10),
    (1.25, 5),
    (1.24, 0),
)
AA_BATTERY_VOLTAGE_LIST = sorted(v for v, _ in AA_BATTERY_VOLTAGE_TO_PERCENTAGE)
AA_BATTERY_VOLTAGE_MAP = dict(AA_BATTERY_VOLTAGE_TO_PERCENTAGE)


def convert_voltage_to_percentage(voltage: float) -> int:
    pos = bisect.bisect_left(AA_BATTERY_VOLTAGE_LIST, voltage)
    if pos != 0:
        pos -= 1
    return AA_BATTERY_VOLTAGE_MAP[AA_BATTERY_VOLTAGE_LIST[pos]]


class AdvertisementDebouncer:
    def __init__(self) -> None:
        self.last_adv_value = -1
        self.last_hk_state = -1

    def next_delay(self, yale_data: bytes = b"", apple_data: bytes = b"") -> int:
        next_update = 0
        if apple_data:
            first_byte = apple_data[0]
            if first_byte == 0x06 and len(apple_data) >= 13:
                hk_state = apple_data[11] | (apple_data[12] << 8)
                if self.last_hk_state == -1:
                    next_update = FIRST_UPDATE_COALESCE_MS
                elif hk_state != self.last_hk_state:
                    next_update = HK_UPDATE_COALESCE_MS
                self.last_hk_state = hk_state
            elif first_byte == 0x11:
                next_update = HK_UPDATE_COALESCE_MS

        first_yale_adv = self.last_adv_value == -1
        if yale_data and (len(yale_data) == 1 or first_yale_adv):
            current_value = yale_data[0]
            if next_update == 0:
                if first_yale_adv:
                    next_update = FIRST_UPDATE_COALESCE_MS
                elif current_value in (0, 1) and current_value != self.last_adv_value:
                    next_update = ADV_UPDATE_COALESCE_MS
            self.last_adv_value = current_value
        return next_update


class ProtocolVectorTest(unittest.TestCase):
    def test_plaintext_command_construction_and_checksums(self) -> None:
        self.assertEqual(
            build_normal_command(COMMAND_GETSTATUS, STATUS_LOCK_ONLY).hex(),
            "ee02000c0200000000000000000000000200",
        )
        self.assertEqual(
            build_normal_command(COMMAND_GETSTATUS, STATUS_DOOR_ONLY).hex(),
            "ee0200e02e00000000000000000000000200",
        )
        self.assertEqual(
            build_normal_command(COMMAND_LOCK).hex(),
            "ee0b00050000000000000000000000000200",
        )
        self.assertEqual(simple_checksum(build_normal_command(COMMAND_UNLOCK)), 0)

        secure = build_secure_command(0x01, bytes.fromhex("0001020304050607"), 1)
        self.assertEqual(secure.hex(), "010000000001020304050607fbf9f7f50f01")
        self.assertEqual(security_checksum(secure), 0xF5F7F9FB)

    def test_response_parsing_vectors(self) -> None:
        locked = bytearray.fromhex("bb02003c0200000005000000000000000000")
        door_ajar = bytearray.fromhex("bb0200132e00000002000000000000000000")
        battery = bytearray.fromhex("bb0200ad0f00000070170000000000000000")
        self.assertEqual(simple_checksum(locked), 0)
        self.assertEqual(locked[8], 0x05)
        self.assertEqual(simple_checksum(door_ajar), 0)
        self.assertEqual(door_ajar[8], 0x02)
        self.assertEqual(simple_checksum(battery), 0)
        self.assertEqual(int.from_bytes(battery[8:10], "little") / 1000, 6.0)

    def test_lock_and_door_enum_mapping(self) -> None:
        lock_status = {
            0x00: "unknown",
            0x01: "unknown_01",
            0x02: "unlocking",
            0x03: "unlocked",
            0x04: "locking",
            0x05: "locked",
            0x06: "unknown_06",
            0x0C: "securemode",
            0x1B: "jammed",
        }
        door_status = {
            0x00: "unknown",
            0x01: "closed",
            0x02: "ajar",
            0x03: "opened",
            0x04: "unknown_04",
        }
        self.assertEqual(lock_status.get(0xFF, "unknown"), "unknown")
        self.assertEqual(lock_status[0x1B], "jammed")
        self.assertEqual(door_status[0x02], "ajar")
        self.assertIsNone({"opened": True, "closed": False}.get(door_status[0x04]))

    def test_battery_conversion_matches_upstream_bisect_behavior(self) -> None:
        self.assertEqual(convert_voltage_to_percentage(1.24), 0)
        self.assertEqual(convert_voltage_to_percentage(1.25), 0)
        self.assertEqual(convert_voltage_to_percentage(1.315), 35)
        self.assertEqual(convert_voltage_to_percentage(1.32), 35)
        self.assertEqual(convert_voltage_to_percentage(1.500), 93)
        self.assertEqual(convert_voltage_to_percentage(1.551), 100)

    def test_advertisement_debounce_decisions(self) -> None:
        debouncer = AdvertisementDebouncer()
        self.assertEqual(debouncer.next_delay(yale_data=b"\x00"), FIRST_UPDATE_COALESCE_MS)
        self.assertEqual(debouncer.next_delay(yale_data=b"\x00"), 0)
        self.assertEqual(debouncer.next_delay(yale_data=b"\x01"), ADV_UPDATE_COALESCE_MS)

        debouncer = AdvertisementDebouncer()
        self.assertEqual(
            debouncer.next_delay(apple_data=bytes.fromhex("06000000000000000000000100")),
            FIRST_UPDATE_COALESCE_MS,
        )
        self.assertEqual(
            debouncer.next_delay(apple_data=bytes.fromhex("06000000000000000000000200")),
            HK_UPDATE_COALESCE_MS,
        )
        self.assertEqual(debouncer.next_delay(apple_data=b"\x11"), HK_UPDATE_COALESCE_MS)

    def test_aes_cbc_first_and_chained_block_vectors(self) -> None:
        try:
            from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
        except ImportError as err:
            self.skipTest(f"cryptography unavailable: {err}")

        key = bytes.fromhex("000102030405060708090a0b0c0d0e0f")
        first = build_normal_command(COMMAND_GETSTATUS, STATUS_LOCK_ONLY)[:16]
        second = build_normal_command(COMMAND_UNLOCK)[:16]
        encryptor = Cipher(algorithms.AES(key), modes.CBC(bytes(16))).encryptor()
        self.assertEqual(encryptor.update(first).hex(), "0ebc4ce47909ac442f890b0e6ce86b9b")
        self.assertEqual(encryptor.update(second).hex(), "149e8f01fbfa672522a339fb7f4c46c8")


if __name__ == "__main__":
    unittest.main()
