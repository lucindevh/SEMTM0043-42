"""Print telemetry from the StampC3 robot over its WiFi access point.

Before running:
  1. Join your computer to the robot's WiFi network (SSID "MelonAP",
     password "MelonismyPassword").
  2. Run:  python robot_reader.py
     Optional: python robot_reader.py --save readings.csv

Uses only the Python standard library.
"""

import argparse
import socket
import time

ROBOT_IP = "192.168.4.1"   # default ESP32 access-point address
ROBOT_PORT = 80            # matches RobotWifiAP_c server("MelonAP", ..., 80)

COLUMNS = [
    "timestamp_ms", "x_mm", "y_mm", "theta_rad",
    "left_pwm", "right_pwm", "left_count", "right_count",
    "DN1", "DN2", "DN3", "DN4", "DN5", "signal",
]


def connect():
    """Keep trying until the robot's TCP server accepts the connection."""
    while True:
        try:
            sock = socket.create_connection((ROBOT_IP, ROBOT_PORT), timeout=5)
            sock.settimeout(None)
            print(f"Connected to {ROBOT_IP}:{ROBOT_PORT}")
            return sock
        except OSError as error:
            print(f"Could not connect ({error}). Is this computer on the MelonAP WiFi? Retrying...")
            time.sleep(2)


def print_record(line):
    # Encoder-test messages are plain text, not CSV records.
    if line.startswith("ENC"):
        print(line)
        return
    values = line.split(",")
    if len(values) != len(COLUMNS):
        print(f"(unexpected line) {line}")
        return
    print("  ".join(f"{name}={value}" for name, value in zip(COLUMNS, values)))


def main():
    parser = argparse.ArgumentParser(description="Print robot telemetry over WiFi.")
    parser.add_argument("--save", metavar="FILE", help="also write the records to a CSV file")
    args = parser.parse_args()

    out = None
    if args.save:
        out = open(args.save, "w", newline="")
        out.write(",".join(COLUMNS) + "\n")

    try:
        while True:
            sock = connect()
            buffer = ""
            try:
                while True:
                    data = sock.recv(1024)
                    if not data:
                        print("Robot closed the connection.")
                        break
                    buffer += data.decode("ascii", errors="replace")
                    # TCP can split or join lines, so only handle complete ones.
                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        line = line.strip()
                        if not line:
                            continue
                        print_record(line)
                        if out and len(line.split(",")) == len(COLUMNS):
                            out.write(line + "\n")
                            out.flush()
            except OSError as error:
                print(f"Connection lost ({error}).")
            finally:
                sock.close()
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        if out:
            out.close()


if __name__ == "__main__":
    main()
