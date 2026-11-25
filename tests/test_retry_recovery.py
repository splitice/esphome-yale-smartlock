"""Run the component's retry and response methods with a fake clock and BLE client.

Only the platform/session helpers are stubbed; method bodies and retry state
members are extracted from production sources so this catches timer starvation.
Requires a host C++17 compiler (g++).
"""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
METHODS = (
    "loop", "queue_operation_", "maybe_start_next_operation_", "start_current_attempt_",
    "connect_current_attempt_", "retry_current_operation_",
    "fail_current_operation_", "force_disconnect_", "write_command_",
    "build_operation_steps_", "matches_normal_response_", "handle_normal_response_",
    "handle_command_timeout_", "parse_state_response_", "parse_lock_status_",
    "parse_door_status_", "validate_simple_response_", "update_lock_status_",
    "is_lock_operation_", "is_expected_complete_", "is_bad_lock_state_",
)


def method(source, name):
    start = re.search(rf"^\w+ YaleXSBLE::{name}\(", source, re.M).start()
    opening = source.index("{", start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


class RetryRecoveryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source = (ROOT / "components/yalexs_ble/yalexs_ble.cpp").read_text()
        header = (ROOT / "components/yalexs_ble/yalexs_ble.h").read_text()
        bodies = [method(source, name) for name in METHODS]
        enums = "\n".join(
            re.search(rf"enum class {name}[^}}]+}};", header).group()
            for name in ("OperationType", "StepType", "MotorProgress", "ResponseChannel", "NotifySetup")
        )
        state = header[header.index("  std::deque<OperationType> operation_queue_;"):
                       header.index("  YaleLockInfo lock_info_;")]
        settings = "\n".join(
            re.search(rf"  uint\d+_t {name}\{{.*?;", header).group()
            for name in ("operation_retries_", "connect_timeout_ms_", "operation_timeout_ms_")
        )
        declarations = "\n".join(body[:body.index("{")].replace("YaleXSBLE::", "") + ";"
                                 for body in bodies)
        fixture = (ROOT / "tests/retry_recovery_fixture.cpp").read_text()
        protocol = "\n".join(
            re.search(rf"^enum class {name}.*?\n}};", header, re.M | re.S).group()
            for name in ("YaleLockStatus", "YaleDoorStatus")
        )
        protocol += "\n" + source[source.index("enum YaleCommand"):source.index("bool YaleLockInfo::door_sense()")]
        fixture = fixture.replace("// PRODUCTION_PROTOCOL", protocol)
        fixture = fixture.replace("// PRODUCTION_DECLARATIONS", enums + state + settings + declarations)
        fixture = fixture.replace("// PRODUCTION_METHODS", "\n".join(bodies))
        cls.temp = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.temp.cleanup)
        cpp = Path(cls.temp.name) / "retry.cpp"
        cpp.write_text(fixture)
        cls.binary = Path(cls.temp.name) / "retry"
        subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
                        str(cpp), "-o", str(cls.binary)], check=True, text=True)

    def run_case(self, case):
        subprocess.run([str(self.binary), case], cwd=self.temp.name, check=True, text=True)

    def test_operation_timeout_does_not_starve_retry(self):
        self.run_case("timeout")

    def test_duplicate_failure_does_not_postpone_retry(self):
        self.run_case("duplicate")

    def test_stuck_disconnect_times_out_and_queue_recovers(self):
        self.run_case("disconnect")

    def test_retry_waits_for_connecting_client_to_become_idle(self):
        self.run_case("connecting")

    def test_zero_retries_fails_once(self):
        self.run_case("zero")

    def test_timeout_across_millis_wraparound(self):
        self.run_case("wrap")

    def test_lock_command_preempts_queued_background_work(self):
        self.run_case("priority")

    def test_authenticated_connection_is_reused(self):
        self.run_case("reuse")

    def test_cooldown_is_per_channel_and_post_auth_only(self):
        self.run_case("cooldown")

    def test_cached_connection_uses_fast_setup_parameters(self):
        self.run_case("aggressive")

    def test_unlock_waits_for_motor_result_and_lock_status(self):
        self.run_case("motor_sequence")

    def test_confirmed_jam_is_terminal(self):
        self.run_case("jam")

    def test_acknowledged_command_is_not_replayed_after_result_timeout(self):
        self.run_case("ack_timeout")

    def test_background_poll_ignores_other_status_frames(self):
        self.run_case("poll_matching")

    def test_lost_verification_refreshes_without_replaying_motor(self):
        self.run_case("verification_timeout")
