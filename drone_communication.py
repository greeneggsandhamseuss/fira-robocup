"""A small, standalone interface to the FIRA ESP32 radio.

Copy this file next to the drone program, install ``pyserial``, and import
``DroneCommunication``.  The ESP32 must be running the ``fira_link`` firmware.
"""

import atexit
import collections
import os
import re
import threading
import time

import serial
from serial.tools import list_ports


class DroneCommunication:
    """Send and receive messages through one FIRA ESP32 radio.

    Construction opens and configures the radio.  ``receive()`` is
    non-blocking by default and returns either::

        {"sender": "drone_b", "text": "ready"}

    or ``None`` when no message is waiting.
    """

    _BAUD = 115200
    _MAX_PAYLOAD_BYTES = 249
    _COMMAND_TIMEOUT = 3.0
    _READY_TIMEOUT = 8.0
    _IDENTIFIER = re.compile(r"[A-Za-z0-9_-]{1,15}")
    _MAC_ADDRESS = re.compile(r"(?:[0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}")
    _KNOWN_USB_IDS = {
        (0x10C4, 0xEA60),  # Silicon Labs CP210x
        (0x1A86, 0x7523),  # WCH CH340
        (0x1A86, 0x55D4),  # WCH CH9102
        (0x0403, 0x6001),  # FTDI FT232
        (0x303A, 0x1001),  # Espressif native USB
        (0x303A, 0x0002),  # Espressif native USB
    }

    def __init__(
        self,
        name,
        team,
        port=None,
        *,
        join_seconds=15,
        reset_board=True,
    ):
        """Open the radio and join ``team`` as ``name``.

        ``name`` and ``team`` must contain 1-15 letters, numbers, ``_`` or
        ``-``. When ``port`` is omitted, detected ESP32 ports are tried in
        order and the first one not already in use is selected automatically.
        """
        auto_detect_port = port is None
        self.name = str(name)
        self.team = str(team)
        self.port = port
        self._validate_identifier(self.name, "name")
        self._validate_identifier(self.team, "team")
        if not isinstance(join_seconds, int) or not 0 <= join_seconds <= 255:
            raise ValueError("join_seconds must be an integer from 0 to 255")

        self._serial = None
        self._reader = None
        self._running = False
        self._closed = False
        self._reader_error = None
        self._read_buffer = b""
        self._messages = collections.deque()
        self._replies = collections.deque()
        self._send_failures = collections.deque()
        self._mac_names = {}
        self._message_ready = threading.Condition()
        self._reply_ready = threading.Condition()
        self._read_lock = threading.Lock()
        self._command_lock = threading.RLock()
        self._ready = threading.Event()

        try:
            if auto_detect_port:
                self.port, self._serial = self._open_first_available_port()
            else:
                self.port = port
                self._serial = self._open_port(self.port)
            if reset_board:
                self._reset_board()
            else:
                self._serial.reset_input_buffer()
            self._running = True
            self._reader = threading.Thread(
                target=self._read_loop,
                name="fira-radio-reader",
                daemon=True,
            )
            self._reader.start()
            if not self._ready.wait(self._READY_TIMEOUT):
                raise RuntimeError(
                    f"The radio on {self.port} did not start. Check that "
                    "it has the fira_link firmware."
                )

            self._command(f"NAME {self.name}")
            info = self._get_info()
            if info.get("group") != self.team:
                self._command(f"JOIN {self.team} {join_seconds}")

            port_source = (
                "Auto-detected and connected"
                if auto_detect_port
                else "Connected"
            )
            print(f"DroneCommunication: {port_source} radio on {self.port}")
        except Exception:
            self.close()
            raise

        atexit.register(self.close)

    def receive(self, timeout=0):
        """Return the next queued message, or ``None`` when none is ready.

        A message is a dictionary with ``sender`` and ``text`` keys.  The
        default ``timeout=0`` never blocks the drone loop.  Pass a positive
        number of seconds to wait, or ``None`` to wait indefinitely.
        """
        if timeout is not None and timeout < 0:
            raise ValueError("timeout cannot be negative")
        self._ensure_open()
        deadline = None if timeout is None else time.monotonic() + timeout

        with self._message_ready:
            while not self._messages:
                self._raise_reader_error()
                if timeout == 0:
                    return None
                remaining = (
                    None if deadline is None else deadline - time.monotonic()
                )
                if remaining is not None and remaining <= 0:
                    return None
                self._message_ready.wait(remaining)
            return self._messages.popleft()

    def send(self, text, to="*"):
        """Send ``text`` to every teammate, or to one teammate/MAC.

        Returns ``True`` once the ESP32 accepts the message for delivery.
        A later wireless delivery failure is retained by ``failures()``.
        """
        self._ensure_open()
        text = str(text).strip()
        if not text:
            raise ValueError("message cannot be empty")
        if "\n" in text or "\r" in text:
            raise ValueError("message cannot contain line breaks")
        size = len(text.encode("utf-8"))
        if size > self._MAX_PAYLOAD_BYTES:
            raise ValueError(
                f"message is {size} UTF-8 bytes; the maximum is "
                f"{self._MAX_PAYLOAD_BYTES}"
            )

        target = str(to)
        valid_target = (
            target == "*"
            or self._IDENTIFIER.fullmatch(target)
            or self._MAC_ADDRESS.fullmatch(target)
        )
        if not valid_target:
            raise ValueError("recipient must be '*', a teammate name, or a MAC")
        self._command(f"SEND {target} {text}")
        return True

    def flush(self):
        """Discard all unread messages and pending serial input.

        Returns the number of already-queued messages that were discarded.
        The command lock prevents this from throwing away a command reply.
        """
        self._ensure_open()
        with self._command_lock:
            with self._read_lock:
                self._serial.reset_input_buffer()
                self._read_buffer = b""
                with self._message_ready:
                    discarded = len(self._messages)
                    self._messages.clear()
        return discarded

    def pending(self):
        """Return the number of unread messages currently in the queue."""
        with self._message_ready:
            return len(self._messages)

    def failures(self):
        """Return and clear recipients with recent wireless send failures."""
        self._ensure_open()
        with self._reply_ready:
            failed = list(self._send_failures)
            self._send_failures.clear()
        return failed

    def teammates(self):
        """Return ``{name: online}`` for the radio's known teammates."""
        peers = {}
        for line in self._command("PEERS"):
            parts = line.split()
            if len(parts) == 4 and parts[0] == "PEER":
                _, mac, name, state = parts
                self._mac_names[mac] = name
                peers[name] = state == "ONLINE"
        return peers

    def close(self):
        """Stop the reader and close the serial port. Safe to call twice."""
        if self._closed:
            return
        self._closed = True
        self._running = False
        with self._message_ready:
            self._message_ready.notify_all()
        with self._reply_ready:
            self._reply_ready.notify_all()
        if self._reader is not None and self._reader is not threading.current_thread():
            self._reader.join(timeout=1.0)
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
        try:
            atexit.unregister(self.close)
        except Exception:
            pass

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    def _find_ports(self):
        ports = list(list_ports.comports())
        candidates = [
            item.device
            for item in ports
            if item.vid is not None
            and (item.vid, item.pid) in self._KNOWN_USB_IDS
        ]
        if not candidates:
            visible = ", ".join(item.device for item in ports) or "none"
            raise RuntimeError(
                "No ESP32 radio was found. Plug it in or provide port=. "
                f"Visible serial ports: {visible}"
            )
        return candidates

    def _open_first_available_port(self):
        errors = []
        for candidate in self._find_ports():
            try:
                return candidate, self._open_port(candidate)
            except RuntimeError as error:
                errors.append(f"{candidate}: {error}")

        details = "; ".join(errors)
        raise RuntimeError(
            "ESP32 radios were detected, but none could be opened. "
            f"They may already be connected by another program. {details}"
        )

    def _open_port(self, port):
        try:
            options = {"timeout": 0.05}
            if os.name != "nt":
                options["exclusive"] = True
            return serial.Serial(port, self._BAUD, **options)
        except (OSError, serial.SerialException) as error:
            raise RuntimeError(
                f"Could not open radio on {port}: {error}"
            ) from error

    def _reset_board(self):
        self._serial.dtr = False
        self._serial.rts = True
        time.sleep(0.1)
        self._serial.rts = False
        self._serial.reset_input_buffer()

    def _read_loop(self):
        while self._running:
            try:
                with self._read_lock:
                    waiting = self._serial.in_waiting
                    chunk = self._serial.read(waiting if waiting else 1)
                    if chunk:
                        self._read_buffer += chunk
                        self._process_buffer()
            except (OSError, serial.SerialException) as error:
                self._reader_error = RuntimeError(
                    f"The radio connection was lost: {error}"
                )
                self._running = False
                with self._message_ready:
                    self._message_ready.notify_all()
                with self._reply_ready:
                    self._reply_ready.notify_all()
                return

    def _process_buffer(self):
        while b"\n" in self._read_buffer:
            raw, self._read_buffer = self._read_buffer.split(b"\n", 1)
            line = raw.decode("utf-8", errors="replace").strip("\r ")
            if line:
                self._route(line)
        if len(self._read_buffer) > 65536:
            self._read_buffer = b""

    def _route(self, line):
        token = line.split(" ", 1)[0]
        if token == "RX":
            parts = line.split(" ", 3)
            if len(parts) == 4:
                _, mac, sender, text = parts
                self._mac_names[mac] = sender
                with self._message_ready:
                    self._messages.append({"sender": sender, "text": text})
                    self._message_ready.notify()
        elif token in ("OK", "ERR", "PEER", "INFO"):
            with self._reply_ready:
                self._replies.append(line)
                self._reply_ready.notify()
        elif token == "READY":
            self._ready.set()
        elif token == "EVT":
            parts = line.split()
            if len(parts) >= 3 and parts[1] == "SEND_FAIL":
                mac = parts[2]
                with self._reply_ready:
                    self._send_failures.append(self._mac_names.get(mac, mac))

    def _command(self, command):
        with self._command_lock:
            self._ensure_open()
            with self._reply_ready:
                self._replies.clear()
            try:
                self._serial.write((command + "\n").encode("utf-8"))
            except (OSError, serial.SerialException) as error:
                raise RuntimeError(f"Could not write to the radio: {error}") from error

            data = []
            deadline = time.monotonic() + self._COMMAND_TIMEOUT
            while True:
                self._raise_reader_error()
                with self._reply_ready:
                    if self._replies:
                        line = self._replies.popleft()
                    else:
                        remaining = deadline - time.monotonic()
                        if remaining <= 0:
                            raise RuntimeError(
                                f"The radio did not answer command: {command}"
                            )
                        self._reply_ready.wait(min(remaining, 0.1))
                        continue
                if line == "OK" or line.startswith("OK "):
                    return data
                if line.startswith("ERR"):
                    reason = line[3:].strip() or "unknown error"
                    raise RuntimeError(f"The radio rejected the command: {reason}")
                data.append(line)

    def _get_info(self):
        for line in self._command("INFO"):
            if line.startswith("INFO "):
                return {
                    key: value
                    for key, value in (
                        item.split("=", 1) for item in line[5:].split()
                    )
                }
        raise RuntimeError("The radio returned an invalid INFO response")

    def _ensure_open(self):
        if self._closed or self._serial is None:
            raise RuntimeError("The radio connection is closed")
        self._raise_reader_error()

    def _raise_reader_error(self):
        if self._reader_error is not None:
            raise self._reader_error
        if self._closed:
            raise RuntimeError("The radio connection is closed")

    def _validate_identifier(self, value, label):
        if not self._IDENTIFIER.fullmatch(value):
            raise ValueError(
                f"{label} must contain 1-15 letters, numbers, '_' or '-'"
            )
