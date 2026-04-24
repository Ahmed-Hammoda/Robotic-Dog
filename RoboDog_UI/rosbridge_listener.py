import json
from datetime import datetime

from websocket import WebSocketApp

STATUS_TOPIC = "/robodog/interface/status_json"
STATUS_TYPE = "std_msgs/msg/String"
DEFAULT_PORT = 9090
DEFAULT_IP = "127.0.0.1"


def stamp() -> str:
    return datetime.now().strftime("%H:%M:%S")


def log(message: str) -> None:
    print(f"[{stamp()}] {message}", flush=True)


def prompt_for_ip() -> str:
    entered = input(f"ROS bridge host/IP or ws URL [{DEFAULT_IP}]: ").strip()
    return entered or DEFAULT_IP


def build_url(value: str) -> str:
    value = value.strip()
    if value.startswith("ws://") or value.startswith("wss://"):
        return value
    return f"ws://{value}:{DEFAULT_PORT}"


def pretty_print_telemetry(raw_payload: str) -> None:
    try:
        payload = json.loads(raw_payload)
    except json.JSONDecodeError as exc:
        log(f"JSON parse error: {exc}")
        log(f"Raw payload: {raw_payload}")
        return

    if isinstance(payload, dict) and "msg" in payload and isinstance(payload["msg"], dict) and "data" in payload["msg"]:
        nested = payload["msg"]["data"]
        try:
            telemetry = json.loads(nested) if isinstance(nested, str) else nested
        except json.JSONDecodeError as exc:
            log(f"Telemetry parse error: {exc}")
            log(f"Raw nested data: {nested}")
            return

        log("Telemetry JSON:")
        print(json.dumps(telemetry, indent=2, ensure_ascii=False), flush=True)
        return

    log("Message:")
    print(json.dumps(payload, indent=2, ensure_ascii=False), flush=True)


def main() -> int:
    rpi_ip = prompt_for_ip()
    ws_url = build_url(rpi_ip)
    log(f"Connecting to {ws_url}")

    packet_count = 0

    def on_open(ws):
        log("Connected")
        ws.send(
            json.dumps(
                {
                    "op": "subscribe",
                    "topic": STATUS_TOPIC,
                    "type": STATUS_TYPE,
                }
            )
        )
        log(f"Subscribed to {STATUS_TOPIC} as {STATUS_TYPE}")
        log("Waiting for telemetry packets...")

    def on_message(_ws, message):
        nonlocal packet_count
        packet_count += 1
        log(f"Packet #{packet_count}")
        pretty_print_telemetry(message)

    def on_error(_ws, error):
        log(f"Error: {error}")

    def on_close(_ws, close_status_code, close_msg):
        log(f"Closed: {close_status_code} {close_msg or ''}".strip())

    ws = WebSocketApp(
        ws_url,
        on_open=on_open,
        on_message=on_message,
        on_error=on_error,
        on_close=on_close,
    )

    try:
        ws.run_forever()
    except KeyboardInterrupt:
        log("Interrupted by user")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
