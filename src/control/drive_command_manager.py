import _thread
import utime

from src.control.drive_target_store import DriveTargetStore
from src.control.drivetrain_constants import SafetyTiming
from src.protocol.request_models import ApiDriveRequest
from src.support.logging import logi, logw

drive_command_lock = _thread.allocate_lock()


class DriveCommandManager:
    """Admit drive commands and track the active command source.

    This class sits between transport ingress and the drivetrain controller. It
    validates command-source ownership, stale-sequence rules, and timeout
    restart guards, while delegating accepted target state storage to
    `DriveTargetStore`.
    """

    def __init__(self):
        self._drive_target_store = DriveTargetStore()
        self._active_command_source_id = None
        self._timeout_fence_seq = -1
        self._timeout_guard_until_ms = utime.ticks_ms()

    def drive(self, drive_request: ApiDriveRequest, command_source_id=None):
        result = self.accept_drive_request(drive_request, command_source_id=command_source_id)
        return result["ok"]

    def accept_drive_request(self, drive_request: ApiDriveRequest, command_source_id=None):
        with drive_command_lock:
            if (
                command_source_id is not None
                and self._active_command_source_id not in (None, command_source_id)
            ):
                return {"ok": False, "code": "ic", "detail": "command source does not own drivetrain"}
            if drive_request.seq is not None and drive_request.seq <= max(
                self._drive_target_store.last_seq, self._timeout_fence_seq
            ):
                return {"ok": False, "code": "st", "detail": "stale sequence"}

            now_ms = utime.ticks_ms()
            if utime.ticks_diff(self._timeout_guard_until_ms, now_ms) > 0:
                return {"ok": False, "code": "si", "detail": "timeout stop settling"}

            self._drive_target_store.update(drive_request, now_ms=now_ms)
            self._active_command_source_id = command_source_id
            return {"ok": True}

    def revoke_drive_request(self, seq=None, command_source_id=None, reason="drive_rejected"):
        with drive_command_lock:
            if (
                command_source_id is not None
                and self._active_command_source_id not in (None, command_source_id)
            ):
                return False
            if seq is not None and self._drive_target_store.last_seq != seq:
                return False
            self._drive_target_store.reset()
            self._active_command_source_id = None
            logw("Drive request revoked: %s" % reason)
            return True

    def turn(self, angle):
        with drive_command_lock:
            self._drive_target_store.set_turn_angle(angle)

    def stop(self, command_source_id=None, reason="manual_stop"):
        with drive_command_lock:
            if (
                command_source_id is not None
                and self._active_command_source_id not in (None, command_source_id)
            ):
                logw("Ignoring stop from non-active command source")
                return False
            self._drive_target_store.reset()
            self._active_command_source_id = None
            logi("Drive stop requested: %s" % reason)
            return True

    def get_drive_target(self):
        with drive_command_lock:
            return self._drive_target_store.snapshot()

    def is_timeout(self, timeout_ms=SafetyTiming.COMMAND_TIMEOUT_MS):
        with drive_command_lock:
            if self._drive_target_store.target_rpm == 0:
                return False
            return self._drive_target_store.command_age_ms() > timeout_ms

    def command_age_ms(self):
        with drive_command_lock:
            return self._drive_target_store.command_age_ms()

    def is_fresh_sequence(self, seq, command_source_id=None):
        with drive_command_lock:
            if seq is None:
                return True
            if (
                command_source_id is not None
                and self._active_command_source_id not in (None, command_source_id)
            ):
                return False
            return seq > max(self._drive_target_store.last_seq, self._timeout_fence_seq)

    def clear_command_source_if_active(self, command_source_id, reason="disconnect_stop"):
        with drive_command_lock:
            if self._active_command_source_id != command_source_id:
                return False
            self._drive_target_store.reset()
            self._active_command_source_id = None
            logw("Active command source lost: %s" % reason)
            return True

    def timeout_stop(self, timeout_ms=SafetyTiming.COMMAND_TIMEOUT_MS):
        with drive_command_lock:
            if self._drive_target_store.target_rpm == 0:
                return None
            age_ms = self._drive_target_store.command_age_ms()
            if age_ms <= timeout_ms:
                return None
            snapshot = {
                "age_ms": age_ms,
                "last_seq": self._drive_target_store.last_seq,
                "command_source_id": self._active_command_source_id,
            }
            self._drive_target_store.reset()
            if self._drive_target_store.last_seq is not None:
                self._timeout_fence_seq = max(
                    self._timeout_fence_seq, self._drive_target_store.last_seq
                )
            self._active_command_source_id = None
            now_ms = utime.ticks_ms()
            self._timeout_guard_until_ms = utime.ticks_add(
                now_ms, SafetyTiming.TIMEOUT_RESTART_GUARD_MS
            )
            logw("Command timeout reached; stopping drive command manager")
            return snapshot


drive_command_manager = DriveCommandManager()
