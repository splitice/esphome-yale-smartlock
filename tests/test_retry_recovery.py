"""Run the component's actual retry methods with a fake clock and BLE client.

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
    "loop", "maybe_start_next_operation_", "start_current_attempt_",
    "connect_current_attempt_", "retry_current_operation_",
    "fail_current_operation_", "force_disconnect_",
)


def method(source, name):
    start = source.index(f"void YaleXSBLE::{name}(")
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
            re.search(rf"enum class {name}.*?\n  }};", header, re.S).group()
            for name in ("OperationType", "StepType", "ResponseChannel")
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
