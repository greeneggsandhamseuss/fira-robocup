"""Reusable video-health and flight-control helpers."""

import time

import cv2
import numpy as np


class FrameFreezeDetector:
    """Return whether video is still while the Tello should be moving."""

    RC_AXES = ('x', 'y', 'z', 'r')
    VELOCITY_AXES = ('vgx', 'vgy', 'vgz')
    COMPARISON_SIZE = (64, 48)
    STILL_MEAN_DIFFERENCE_MAX = 0.80
    STILL_FRAME_CONFIRMATIONS = 2
    REPEATED_FRAME_SECONDS = 0.10
    MOTION_START_GRACE_SECONDS = 0.08
    RC_MOTION_THRESHOLD = 5
    VELOCITY_MOTION_THRESHOLD = 3

    def __init__(self):
        self.last_frame_object = None
        self.last_small_gray = None
        self.last_new_frame_time = None
        self.motion_started_at = None
        self.still_frame_count = 0
        self.frozen = False

    @staticmethod
    def _absolute_value(values, key):
        try:
            return abs(float(values.get(key, 0)))
        except (AttributeError, TypeError, ValueError):
            return 0.0

    def _motion_expected(self, rc_command, telemetry):
        return any(self._absolute_value(rc_command, key) >= self.RC_MOTION_THRESHOLD for key in self.RC_AXES) or any(self._absolute_value(telemetry, key) >= self.VELOCITY_MOTION_THRESHOLD for key in self.VELOCITY_AXES)

    def _small_gray(self, frame):
        if not isinstance(frame, np.ndarray) or frame.ndim != 3 or frame.shape[2] < 3 or frame.size == 0:
            return None

        try:
            small = cv2.resize(frame[:, :, :3], self.COMPARISON_SIZE, interpolation=cv2.INTER_AREA)
            return cv2.cvtColor(small, cv2.COLOR_RGB2GRAY)
        except (cv2.error, TypeError, ValueError):
            return None

    def is_frozen(self, frame, rc_command, telemetry, now=None):
        """Return one boolean; ``True`` blocks this frame from vision use."""
        now = time.monotonic() if now is None else float(now)
        motion_expected = self._motion_expected(rc_command, telemetry)

        if motion_expected:
            if self.motion_started_at is None:
                self.motion_started_at = now
        else:
            self.motion_started_at = None
            self.still_frame_count = 0
            self.frozen = False

        is_new_frame = frame is not None and frame is not self.last_frame_object
        if is_new_frame:
            small_gray = self._small_gray(frame)
            if small_gray is None:
                return False

            if self.last_small_gray is None:
                self.still_frame_count = 0
            else:
                difference = cv2.absdiff(small_gray, self.last_small_gray)
                mean_difference = float(np.mean(difference))
                if motion_expected and mean_difference <= self.STILL_MEAN_DIFFERENCE_MAX:
                    self.still_frame_count += 1
                else:
                    self.still_frame_count = 0

            self.last_frame_object = frame
            self.last_small_gray = small_gray
            self.last_new_frame_time = now

        if not motion_expected or self.motion_started_at is None:
            return False

        if now - self.motion_started_at < self.MOTION_START_GRACE_SECONDS:
            self.frozen = False
            return False

        no_new_frame = False
        if not is_new_frame and self.last_new_frame_time is not None:
            repeated_since = max(self.last_new_frame_time, self.motion_started_at)
            no_new_frame = now - repeated_since >= self.REPEATED_FRAME_SECONDS

        still_content = self.still_frame_count >= self.STILL_FRAME_CONFIRMATIONS
        self.frozen = no_new_frame or still_content
        return self.frozen


class KeyboardController:
    """Poll movement keys and return the currently held manual RC command."""

    SPEED = 40
    ROTATION_SPEED = 30
    HEIGHT_SPEED = 40
    INITIAL_HOLD = 0.60
    REPEAT_HOLD = 0.12
    LEFT_KEYS = (2, 81, 2424832, 63234, 65361)
    RIGHT_KEYS = (3, 83, 2555904, 63235, 65363)
    FORWARD_KEYS = (0, 82, 2490368, 63232, 65362)
    BACK_KEYS = (1, 84, 2621440, 63233, 65364)

    def __init__(self):
        self.key_name = None
        self.command = self._zero_command()
        self.command_until = 0.0

    @staticmethod
    def _zero_command():
        return {'x':0,'y':0,'z':0,'r':0}

    def _command_for_key(self, key):
        if key in self.LEFT_KEYS:
            return 'left', {'x':-self.SPEED,'y':0,'z':0,'r':0}
        if key in self.RIGHT_KEYS:
            return 'right', {'x':self.SPEED,'y':0,'z':0,'r':0}
        if key in self.FORWARD_KEYS:
            return 'forward', {'x':0,'y':0,'z':self.SPEED,'r':0}
        if key in self.BACK_KEYS:
            return 'back', {'x':0,'y':0,'z':-self.SPEED,'r':0}
        if key in (ord('a'), ord('A')):
            return 'yaw_left', {'x':0,'y':0,'z':0,'r':-self.ROTATION_SPEED}
        if key in (ord('d'), ord('D')):
            return 'yaw_right', {'x':0,'y':0,'z':0,'r':self.ROTATION_SPEED}
        if key in (ord('w'), ord('W')):
            return 'up', {'x':0,'y':self.HEIGHT_SPEED,'z':0,'r':0}
        if key in (ord('s'), ord('S')):
            return 'down', {'x':0,'y':-self.HEIGHT_SPEED,'z':0,'r':0}
        return None, None

    def poll(self, key=None, now=None):
        """Return ``(key_code, command_or_none)`` without changing drone state."""
        key = cv2.waitKeyEx(1) if key is None else int(key)
        now = time.monotonic() if now is None else float(now)
        key_name, command = self._command_for_key(key)

        if command is not None:
            is_repeat = self.key_name == key_name and now < self.command_until
            self.key_name = key_name
            self.command = command
            self.command_until = now + (self.REPEAT_HOLD if is_repeat else self.INITIAL_HOLD)

        active_command = self.command.copy() if now < self.command_until else None
        return key, active_command

    def reset(self):
        """Clear any held manual command."""
        self.key_name = None
        self.command = self._zero_command()
        self.command_until = 0.0


class AirBrake:
    """Apply counter-motion and report when a requested stop has settled."""

    AXES = ('x', 'y', 'z', 'r')
    TRANSLATION_AXES = ('x', 'y', 'z')
    DURATION = 0.30
    YAW_DURATION = 0.15
    FORWARD_DURATION = 0.35
    MIN_SPEED = 1
    VELOCITY_GAIN = 8
    FORWARD_VELOCITY_GAIN = 9
    MIN_COMMAND = 15
    MAX_COMMAND = 50
    FORWARD_MAX_COMMAND = 50
    STOP_SPEED = 2
    STOP_CONFIRMATIONS = 4

    def __init__(self):
        self.previous_requested = self._zero_command()
        self.brake_output = self._zero_command()
        self.brake_until = {key: 0.0 for key in self.AXES}
        self.finish_waiting = False
        self.stop_count = 0

    @classmethod
    def _zero_command(cls):
        return {key: 0 for key in cls.AXES}

    @classmethod
    def _normalize_command(cls, command):
        return {key: int(command.get(key, 0)) for key in cls.AXES}

    @staticmethod
    def _telemetry_value(telemetry, key):
        try:
            return float(telemetry.get(key, 0))
        except (AttributeError, TypeError, ValueError):
            return 0.0

    def _control_velocity(self, telemetry, yaw):
        yaw_radians = np.radians(float(yaw))
        world_x = self._telemetry_value(telemetry, 'vgx')
        world_y = self._telemetry_value(telemetry, 'vgy')

        return {'x':int(round(-world_x * np.sin(yaw_radians) + world_y * np.cos(yaw_radians))), 'y':-int(self._telemetry_value(telemetry, 'vgz')), 'z':int(round(world_x * np.cos(yaw_radians) + world_y * np.sin(yaw_radians))), 'r':0}

    def _clear_brake(self):
        for key in self.AXES:
            self.brake_output[key] = 0
            self.brake_until[key] = 0.0

    def _apply(self, requested, telemetry, yaw, now):
        command = requested.copy()
        velocity = self._control_velocity(telemetry, yaw)

        if any(requested.values()):
            self._clear_brake()
            self.previous_requested = requested.copy()
            return command

        if any(now < self.brake_until[key] for key in self.AXES):
            for key in self.AXES:
                if now < self.brake_until[key]:
                    command[key] = self.brake_output[key]
                elif self.brake_until[key] != 0:
                    self.brake_output[key] = 0
                    self.brake_until[key] = 0.0
                self.previous_requested[key] = 0
            return command

        self._clear_brake()

        if any(self.previous_requested.values()):
            for key in self.AXES:
                measured_speed = velocity[key]

                if key != 'r' and abs(measured_speed) < self.MIN_SPEED:
                    continue

                if key == 'r':
                    if self.previous_requested[key] == 0:
                        continue
                    measured_speed = self.previous_requested[key]
                    brake_speed = min(abs(measured_speed), self.MAX_COMMAND)
                    brake_duration = self.YAW_DURATION
                else:
                    is_forward_brake = key == 'z' and measured_speed > 0
                    velocity_gain = self.FORWARD_VELOCITY_GAIN if is_forward_brake else self.VELOCITY_GAIN
                    max_command = self.FORWARD_MAX_COMMAND if is_forward_brake else self.MAX_COMMAND
                    brake_duration = self.FORWARD_DURATION if is_forward_brake else self.DURATION
                    brake_speed = int(abs(measured_speed) * velocity_gain)
                    brake_speed = max(self.MIN_COMMAND, brake_speed)
                    brake_speed = min(brake_speed, max_command)

                self.brake_output[key] = -brake_speed if measured_speed > 0 else brake_speed
                self.brake_until[key] = now + brake_duration
                command[key] = self.brake_output[key]

        self.previous_requested = requested.copy()
        return command

    def _finished(self, telemetry, yaw, now):
        brake_active = any(now < self.brake_until[key] for key in self.AXES)

        if brake_active:
            self.finish_waiting = True
            self.stop_count = 0
            return False

        if not self.finish_waiting:
            return True

        velocity = self._control_velocity(telemetry, yaw)
        velocity_stopped = all(abs(velocity[key]) <= self.STOP_SPEED for key in self.TRANSLATION_AXES)
        self.stop_count = self.stop_count + 1 if velocity_stopped else 0

        if self.stop_count < self.STOP_CONFIRMATIONS:
            return False

        self.finish_waiting = False
        self.stop_count = 0
        return True

    def apply(self, command, telemetry, yaw):
        """Return the command after applying any required counter-motion."""
        now = time.monotonic()
        requested = self._normalize_command(command)
        transition_to_zero = any(self.previous_requested.values()) and not any(requested.values())

        if transition_to_zero:
            self.finish_waiting = True
            self.stop_count = 0
        elif any(requested.values()):
            self.finish_waiting = False
            self.stop_count = 0

        return self._apply(requested, telemetry, yaw, now)

    def is_finished(self, telemetry, yaw, stop_requested=True):
        """Poll whether braking is finished without changing any drone task."""
        now = time.monotonic()
        brake_active = any(now < self.brake_until[key] for key in self.AXES)

        if not stop_requested:
            if brake_active:
                self.finish_waiting = True
                self.stop_count = 0
                return False
            self.finish_waiting = False
            self.stop_count = 0
            return True

        brake_scheduled = any(self.brake_until[key] != 0 for key in self.AXES)
        brake_activation_pending = any(self.previous_requested.values()) and not brake_scheduled

        if brake_activation_pending or brake_scheduled:
            self.finish_waiting = True

        if not self.finish_waiting:
            return True

        if brake_activation_pending or brake_active:
            self.stop_count = 0
            return False

        return self._finished(telemetry, yaw, now)
