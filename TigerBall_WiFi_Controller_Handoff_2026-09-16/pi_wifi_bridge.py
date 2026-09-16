#!/usr/bin/env python3
"""TigerBall bench-only Wi-Fi to Teensy safety bridge.

The Raspberry Pi serves controller_wifi.html and is the only process allowed to
write to the Teensy. A browser must hold a short-lived control lease and send a
heartbeat. Loss of the browser, Wi-Fi, Pi process, or serial stream results in
STOP/DISARM here or in the Teensy's independent 400 ms command timeout.
"""

from __future__ import annotations

import argparse
import glob
import json
import secrets
import signal
import sys
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, urlparse

try:
    import serial  # type: ignore
    from serial.tools import list_ports  # type: ignore
except ImportError:  # pragma: no cover - exercised on a Pi before setup
    serial = None
    list_ports = None


ROOT = Path(__file__).resolve().parent
GUI_FILE = ROOT / "controller_wifi.html"


class ControlError(RuntimeError):
    def __init__(self, message: str, status: HTTPStatus = HTTPStatus.BAD_REQUEST):
        super().__init__(message)
        self.status = status


def discover_teensy_port(explicit: str | None = None) -> str:
    """Return one Teensy port, refusing an ambiguous multi-device guess."""
    if explicit:
        return explicit

    candidates: list[str] = []
    for path in sorted(glob.glob("/dev/serial/by-id/*Teensy*")):
        candidates.append(path)

    if list_ports is not None:
        for port in list_ports.comports():
            identity = " ".join(
                str(value or "")
                for value in (port.description, port.manufacturer, port.product, port.hwid)
            ).lower()
            if "teensy" in identity or getattr(port, "vid", None) == 0x16C0:
                candidates.append(port.device)

    unique = list(dict.fromkeys(candidates))
    if len(unique) == 1:
        return unique[0]
    if not unique:
        raise ControlError(
            "No Teensy serial port was identified. Connect it or pass --serial-port /dev/ttyACM0."
        )
    raise ControlError(
        "More than one possible Teensy port was found; choose one with --serial-port: "
        + ", ".join(unique)
    )


class BenchBroker:
    def __init__(
        self,
        *,
        control_key: str,
        serial_port: str | None,
        baud: int,
        browser_timeout_ms: int,
        refresh_ms: int,
        simulate: bool,
    ) -> None:
        self.control_key = control_key
        self.serial_port_option = serial_port
        self.baud = baud
        self.browser_timeout_s = browser_timeout_ms / 1000.0
        self.refresh_s = refresh_ms / 1000.0
        self.simulate = simulate

        self.lock = threading.RLock()
        self.write_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.serial_handle: Any = None
        self.serial_port_name: str | None = "SIMULATED" if simulate else None
        self.serial_connected = simulate
        self.active_client: str | None = None
        self.last_browser_heartbeat = 0.0
        self.armed = False
        self.estop_latched = False
        self.desired_left = 0
        self.desired_right = 0
        self.last_teensy_line = "SIMULATION READY" if simulate else "Waiting for Teensy"
        self.last_fault = ""

        self.connection_thread = threading.Thread(
            target=self._connection_loop, name="teensy-connection", daemon=True
        )
        self.refresh_thread = threading.Thread(
            target=self._refresh_loop, name="safety-refresh", daemon=True
        )

    def start(self) -> None:
        self.connection_thread.start()
        self.refresh_thread.start()

    def _connection_loop(self) -> None:
        if self.simulate:
            while not self.stop_event.wait(0.2):
                pass
            return

        while not self.stop_event.is_set():
            if serial is None:
                with self.lock:
                    self.last_fault = "pyserial is not installed; run: python3 -m pip install -r requirements.txt"
                self.stop_event.wait(2.0)
                continue

            try:
                port_name = discover_teensy_port(self.serial_port_option)
                handle = serial.Serial(port_name, self.baud, timeout=0.2, write_timeout=0.2)
                with self.lock:
                    self.serial_handle = handle
                    self.serial_port_name = port_name
                    self.serial_connected = True
                    self.last_fault = ""
                    self.last_teensy_line = "Serial connected; waiting for Teensy"
                handle.reset_input_buffer()
                while not self.stop_event.is_set() and handle.is_open:
                    raw = handle.readline()
                    if raw:
                        self._handle_teensy_line(raw.decode("utf-8", errors="replace").strip())
            except Exception as exc:
                with self.lock:
                    self.last_fault = f"Teensy serial unavailable: {exc}"
                    self.serial_connected = False
                    self.serial_handle = None
                    self.armed = False
                    self.desired_left = 0
                    self.desired_right = 0
                self.stop_event.wait(1.0)
            finally:
                try:
                    if self.serial_handle is not None:
                        self.serial_handle.close()
                except Exception:
                    pass
                with self.lock:
                    self.serial_handle = None
                    self.serial_connected = False

    def _handle_teensy_line(self, line: str) -> None:
        if not line:
            return
        with self.lock:
            self.last_teensy_line = line
            if line == "OK,ARMED":
                self.armed = True
                self.estop_latched = False
            elif line in {"OK,DISARMED", "OK,STOPPED"}:
                if line == "OK,DISARMED":
                    self.armed = False
                self.desired_left = 0
                self.desired_right = 0
            elif line in {"OK,ESTOP_LATCHED", "ERR,ESTOP_LATCHED"}:
                self.armed = False
                self.estop_latched = True
                self.desired_left = 0
                self.desired_right = 0
            elif line == "OK,ESTOP_RESET_DISARMED":
                self.armed = False
                self.estop_latched = False
            elif line.startswith("TIMEOUT") or line.startswith("READY"):
                self.armed = False
                self.desired_left = 0
                self.desired_right = 0

    def _write(self, command: str) -> None:
        if self.simulate:
            simulated_reply = {
                "ARM": "OK,ARMED",
                "STOP": "OK,STOPPED",
                "DISARM": "OK,DISARMED",
                "ESTOP": "OK,ESTOP_LATCHED",
                "RESET_ESTOP": "OK,ESTOP_RESET_DISARMED",
                "PING": "OK,PONG",
            }.get(command, f"OK,{command}")
            self._handle_teensy_line(simulated_reply)
            return

        with self.lock:
            handle = self.serial_handle
            connected = self.serial_connected
        if not connected or handle is None:
            raise ControlError("The Pi is not connected to the Teensy", HTTPStatus.SERVICE_UNAVAILABLE)
        try:
            with self.write_lock:
                handle.write((command + "\n").encode("ascii"))
                handle.flush()
        except Exception as exc:
            with self.lock:
                self.serial_connected = False
                self.armed = False
                self.desired_left = 0
                self.desired_right = 0
                self.last_fault = f"Serial write failed: {exc}"
            raise ControlError("Serial write to Teensy failed", HTTPStatus.SERVICE_UNAVAILABLE) from exc

    def _refresh_loop(self) -> None:
        while not self.stop_event.wait(self.refresh_s):
            now = time.monotonic()
            with self.lock:
                client = self.active_client
                stale = bool(client) and now - self.last_browser_heartbeat > self.browser_timeout_s
                armed = self.armed
                left = self.desired_left
                right = self.desired_right

            if stale:
                try:
                    self._write("STOP")
                    self._write("DISARM")
                except ControlError:
                    pass
                with self.lock:
                    self.active_client = None
                    self.armed = False
                    self.desired_left = 0
                    self.desired_right = 0
                    self.last_fault = "Browser/Wi-Fi heartbeat expired; STOP and DISARM requested"
                continue

            if armed and client:
                try:
                    command = f"MOVE,{left},{right}" if left or right else "PING"
                    self._write(command)
                except ControlError:
                    with self.lock:
                        self.armed = False
                        self.desired_left = 0
                        self.desired_right = 0

    def _check_key(self, supplied: Any) -> None:
        if not isinstance(supplied, str) or not secrets.compare_digest(supplied, self.control_key):
            raise ControlError("Incorrect control key", HTTPStatus.FORBIDDEN)

    @staticmethod
    def _client_id(value: Any) -> str:
        if not isinstance(value, str) or not 8 <= len(value) <= 100:
            raise ControlError("A valid browser client_id is required")
        return value

    def _require_owner(self, client_id: str) -> None:
        with self.lock:
            if self.active_client != client_id:
                raise ControlError("This browser does not hold the control lease", HTTPStatus.CONFLICT)

    def command(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._check_key(payload.get("key"))
        client_id = self._client_id(payload.get("client_id"))
        action = payload.get("action")
        now = time.monotonic()

        if action == "estop":
            self._write("ESTOP")
            with self.lock:
                self.armed = False
                self.estop_latched = True
                self.desired_left = 0
                self.desired_right = 0
            return self.status(client_id)

        if action == "claim":
            with self.lock:
                if not self.serial_connected:
                    raise ControlError(
                        "The Pi is not connected to the Teensy",
                        HTTPStatus.SERVICE_UNAVAILABLE,
                    )
                if (
                    self.active_client is not None
                    and self.active_client != client_id
                    and now - self.last_browser_heartbeat <= self.browser_timeout_s
                ):
                    raise ControlError("Another browser currently holds control", HTTPStatus.CONFLICT)
                self.active_client = client_id
                self.last_browser_heartbeat = now
                self.desired_left = 0
                self.desired_right = 0
                self.last_fault = ""
            self._write("STOP")
            self._write("DISARM")
            return self.status(client_id)

        self._require_owner(client_id)
        with self.lock:
            self.last_browser_heartbeat = now

        if action == "heartbeat":
            try:
                left = int(payload.get("left", 0))
                right = int(payload.get("right", 0))
            except (TypeError, ValueError) as exc:
                raise ControlError("left and right must be integer percentages") from exc
            if not -100 <= left <= 100 or not -100 <= right <= 100:
                raise ControlError("left and right must be between -100 and 100")
            with self.lock:
                self.desired_left = left if self.armed else 0
                self.desired_right = right if self.armed else 0
        elif action == "arm":
            with self.lock:
                if self.estop_latched:
                    raise ControlError("Reset the E-stop latch before arming", HTTPStatus.CONFLICT)
                self.desired_left = 0
                self.desired_right = 0
            self._write("ARM")
        elif action == "stop":
            with self.lock:
                self.desired_left = 0
                self.desired_right = 0
            self._write("STOP")
        elif action == "disarm":
            self._write("STOP")
            self._write("DISARM")
            with self.lock:
                self.armed = False
                self.desired_left = 0
                self.desired_right = 0
        elif action == "reset_estop":
            self._write("RESET_ESTOP")
            with self.lock:
                self.armed = False
                self.estop_latched = False
        elif action == "release":
            try:
                self._write("STOP")
                self._write("DISARM")
            finally:
                with self.lock:
                    self.active_client = None
                    self.armed = False
                    self.desired_left = 0
                    self.desired_right = 0
        else:
            raise ControlError("Unknown control action")
        return self.status(client_id)

    def status(self, client_id: str | None = None) -> dict[str, Any]:
        with self.lock:
            age_ms = (
                int((time.monotonic() - self.last_browser_heartbeat) * 1000)
                if self.active_client
                else None
            )
            return {
                "ok": True,
                "serial_connected": self.serial_connected,
                "serial_port": self.serial_port_name,
                "control_owned": bool(client_id and self.active_client == client_id),
                "control_busy": bool(self.active_client and self.active_client != client_id),
                "heartbeat_age_ms": age_ms,
                "browser_timeout_ms": int(self.browser_timeout_s * 1000),
                "armed": self.armed,
                "estop_latched": self.estop_latched,
                "left": self.desired_left,
                "right": self.desired_right,
                "last_teensy_line": self.last_teensy_line,
                "fault": self.last_fault,
                "simulation": self.simulate,
            }

    def shutdown(self) -> None:
        try:
            self._write("STOP")
            self._write("DISARM")
        except ControlError:
            pass
        self.stop_event.set()
        with self.lock:
            handle = self.serial_handle
            self.serial_handle = None
            self.serial_connected = False
        try:
            if handle is not None:
                handle.close()
        except Exception:
            pass


class ControllerServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address: tuple[str, int], broker: BenchBroker):
        super().__init__(address, ControllerHandler)
        self.broker = broker


class ControllerHandler(BaseHTTPRequestHandler):
    server: ControllerServer

    def log_message(self, fmt: str, *args: Any) -> None:
        sys.stderr.write("[%s] %s\n" % (self.log_date_time_string(), fmt % args))

    def _headers(self, status: HTTPStatus, content_type: str, length: int) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(length))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Content-Security-Policy", "default-src 'self'; style-src 'self' 'unsafe-inline'; script-src 'self' 'unsafe-inline'; connect-src 'self'")
        self.end_headers()

    def _json(self, value: dict[str, Any], status: HTTPStatus = HTTPStatus.OK) -> None:
        body = json.dumps(value, separators=(",", ":")).encode("utf-8")
        self._headers(status, "application/json; charset=utf-8", len(body))
        self.wfile.write(body)

    def do_GET(self) -> None:  # noqa: N802
        parsed = urlparse(self.path)
        if parsed.path in {"/", "/controller_wifi.html"}:
            body = GUI_FILE.read_bytes()
            self._headers(HTTPStatus.OK, "text/html; charset=utf-8", len(body))
            self.wfile.write(body)
            return
        if parsed.path == "/api/status":
            client_id = parse_qs(parsed.query).get("client_id", [None])[0]
            self._json(self.server.broker.status(client_id))
            return
        self._json({"ok": False, "error": "Not found"}, HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:  # noqa: N802
        if urlparse(self.path).path != "/api/control":
            self._json({"ok": False, "error": "Not found"}, HTTPStatus.NOT_FOUND)
            return
        if self.headers.get_content_type() != "application/json":
            self._json({"ok": False, "error": "Content-Type must be application/json"}, HTTPStatus.UNSUPPORTED_MEDIA_TYPE)
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if length <= 0 or length > 4096:
                raise ControlError("Invalid request size")
            payload = json.loads(self.rfile.read(length))
            if not isinstance(payload, dict):
                raise ControlError("JSON body must be an object")
            self._json(self.server.broker.command(payload))
        except ControlError as exc:
            self._json({"ok": False, "error": str(exc)}, exc.status)
        except (json.JSONDecodeError, ValueError):
            self._json({"ok": False, "error": "Invalid JSON"}, HTTPStatus.BAD_REQUEST)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="0.0.0.0", help="listen address (default: all Pi interfaces)")
    parser.add_argument("--port", type=int, default=8000, help="HTTP port (default: 8000)")
    parser.add_argument("--serial-port", help="Teensy port; default safely auto-detects Teensy identity")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--control-key", help="shared GUI key; default generates a new one at startup")
    parser.add_argument("--browser-timeout-ms", type=int, default=350)
    parser.add_argument("--refresh-ms", type=int, default=100)
    parser.add_argument("--simulate", action="store_true", help="never open serial or touch motors")
    args = parser.parse_args()
    if not 200 <= args.browser_timeout_ms <= 2000:
        parser.error("--browser-timeout-ms must be 200..2000")
    if not 50 <= args.refresh_ms < args.browser_timeout_ms:
        parser.error("--refresh-ms must be >=50 and less than the browser timeout")
    return args


def main() -> int:
    args = parse_args()
    control_key = args.control_key or secrets.token_urlsafe(8)
    broker = BenchBroker(
        control_key=control_key,
        serial_port=args.serial_port,
        baud=args.baud,
        browser_timeout_ms=args.browser_timeout_ms,
        refresh_ms=args.refresh_ms,
        simulate=args.simulate,
    )
    server = ControllerServer((args.bind, args.port), broker)
    broker.start()

    mode = "SIMULATION — NO MOTOR OUTPUT" if args.simulate else "BENCH CONTROL"
    print(f"TigerBall Pi Wi-Fi bridge: {mode}", flush=True)
    print(f"On the Pi: http://127.0.0.1:{args.port}/?key={control_key}", flush=True)
    print(f"From laptop: http://tigerball-pi.local:{args.port}/?key={control_key}", flush=True)
    print("Keep this terminal open. Ctrl+C requests STOP and DISARM.", flush=True)

    def request_shutdown(_signum: int, _frame: Any) -> None:
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)
    try:
        server.serve_forever(poll_interval=0.2)
    finally:
        broker.shutdown()
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
