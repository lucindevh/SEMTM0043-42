"""Receive everything the StampC3 robot reports over its WiFi access point.

The robot sends exactly the lines it prints to the Serial Monitor:
  - CSV headers and rows from the combined (red) test, the odometry test and
    the sensor tests, and telemetry records when telemetry is switched on;
  - plain-text messages: "ENC ..." lines from the encoder test and "# ..."
    lines when a move fails or times out.

Before running:
  1. Join this computer to the robot's WiFi network. The network name and
     password are the first two arguments of RobotWifiAP_c server(...) in
     Team_777.ino (currently "lvhAP").
  2. Run:  python Wireless.py
     Save as well:  python Wireless.py --save run1
       Each kind of record goes to its own file, created when its first row
       arrives: run1_combined.csv, run1_odometry_translation.csv,
       run1_odometry_rotation.csv, run1_offset.csv, run1_sensor_raw.csv,
       run1_telemetry.csv, and run1_messages.txt for the text messages.
     Show only headers and messages on screen:  python Wireless.py --quiet

Uses only the Python standard library.
"""

import argparse
import socket
import time

ROBOT_IP = "192.168.4.1"   # default ESP32 access-point address
ROBOT_PORT = 80            # matches RobotWifiAP_c server(..., 80) in Team_777.ino

# Column names for each kind of record. The robot prints a header line
# (starting "test_name,") when a logged test starts, and that header replaces
# the default here. The defaults only matter if this script connects after the
# header has already been sent.
DEFAULT_COLUMNS = {
    "combined": [
        "test_name", "trial_num", "condition", "controller_ms", "imu_timestamp_us",
        "sequence", "gyro_x_raw", "gyro_y_raw", "gyro_z_raw", "yaw_rate_dps",
        "left_encoder_count", "right_encoder_count", "odometry_heading_rad",
        "encoders_fresh", "odometry_x_mm", "odometry_y_mm", "accel_x_raw",
        "accel_y_raw", "accel_z_raw", "surface_dn1", "surface_dn2", "surface_dn3",
        "surface_dn4", "surface_dn5",
    ],
    "odometry_translation": [
        "test_name", "trial_num", "commanded_distance_mm", "sample_time_ms",
        "left_encoder_count", "right_encoder_count", "odometry_x_mm",
        "odometry_y_mm", "odometry_theta_rad",
    ],
    "odometry_rotation": [
        "test_name", "trial_num", "commanded_angle_rad", "sample_time_ms",
        "left_encoder_count", "right_encoder_count", "odometry_x_mm",
        "odometry_y_mm", "odometry_theta_rad",
    ],
    # testOffset(): one row per sensor for each new surface snapshot.
    "offset": [
        "test_name", "trial_num", "sample_num", "sample_time_us",
        "sensor_num", "sensor_reading",
    ],
    # sensorTest(): the five raw readings on one line.
    "sensor_raw": ["DN1", "DN2", "DN3", "DN4", "DN5"],
    # publishTelemetry(): sent without a header line.
    "telemetry": [
        "timestamp_ms", "x_mm", "y_mm", "theta_rad",
        "left_pwm", "right_pwm", "left_count", "right_count",
        "DN1", "DN2", "DN3", "DN4", "DN5", "signal",
    ],
}

# CSV rows are recognised by how many comma-separated values they contain.
ODOMETRY_FIELDS = 9
KIND_BY_FIELDS = {
    len(DEFAULT_COLUMNS["combined"]): "combined",
    len(DEFAULT_COLUMNS["offset"]): "offset",
    len(DEFAULT_COLUMNS["sensor_raw"]): "sensor_raw",
    len(DEFAULT_COLUMNS["telemetry"]): "telemetry",
}


class Recorder:
    """Sorts incoming lines by kind, prints them, and optionally saves them."""

    def __init__(self, prefix, quiet):
        self.prefix = prefix
        self.quiet = quiet
        self.columns = {kind: list(names) for kind, names in DEFAULT_COLUMNS.items()}
        # Nine-value rows could be either odometry test; the latest odometry
        # header says which. Translation is the firmware's default.
        self.odometry_kind = "odometry_translation"
        self.files = {}
        self.messages = None

    def handle(self, line):
        # Text messages may contain commas, so recognise them first.
        if line.startswith("ENC") or line.startswith("#"):
            print(line)
            self._save_message(line)
            return

        values = line.split(",")

        if values[0] == "test_name":
            kind = self._header_kind(values)
            print(f"[header: {kind or 'unknown'}] {line}")
            if kind:
                self.columns[kind] = values
            else:
                self._save_message(line)
            return

        if len(values) == ODOMETRY_FIELDS:
            kind = self.odometry_kind
        else:
            kind = KIND_BY_FIELDS.get(len(values))

        if kind is None:
            print(f"(unrecognised line) {line}")
            self._save_message(line)
            return

        if not self.quiet:
            print(line)
        self._save_row(kind, line)

    def _header_kind(self, values):
        if len(values) == len(DEFAULT_COLUMNS["combined"]):
            return "combined"
        if len(values) == ODOMETRY_FIELDS:
            if values[2] == "commanded_angle_rad":
                self.odometry_kind = "odometry_rotation"
            else:
                self.odometry_kind = "odometry_translation"
            return self.odometry_kind
        return None

    def _save_row(self, kind, line):
        if not self.prefix:
            return
        out = self.files.get(kind)
        if out is None:
            out = open(f"{self.prefix}_{kind}.csv", "w", newline="")
            out.write(",".join(self.columns[kind]) + "\n")
            self.files[kind] = out
            print(f"(saving {kind} rows to {out.name})")
        out.write(line + "\n")
        out.flush()

    def _save_message(self, line):
        if not self.prefix:
            return
        if self.messages is None:
            self.messages = open(f"{self.prefix}_messages.txt", "w")
        # The computer's clock time helps match messages to the CSV rows.
        self.messages.write(f"{time.strftime('%H:%M:%S')} {line}\n")
        self.messages.flush()

    def close(self):
        for out in self.files.values():
            out.close()
        if self.messages:
            self.messages.close()


def connect(ip, port):
    """Keep trying until the robot's TCP server accepts the connection."""
    while True:
        try:
            sock = socket.create_connection((ip, port), timeout=5)
            sock.settimeout(None)
            print(f"Connected to {ip}:{port}. Waiting for the robot to send readings...")
            return sock
        except OSError as error:
            print(f"Could not connect ({error}). Is this computer on the robot's WiFi? Retrying...")
            time.sleep(2)


def main():
    parser = argparse.ArgumentParser(description="Receive robot readings over WiFi.")
    parser.add_argument("--save", metavar="PREFIX",
                        help="also save each kind of record to PREFIX_<kind>.csv")
    parser.add_argument("--quiet", action="store_true",
                        help="show only headers and text messages, not every CSV row")
    parser.add_argument("--ip", default=ROBOT_IP, help=f"robot address (default {ROBOT_IP})")
    parser.add_argument("--port", type=int, default=ROBOT_PORT,
                        help=f"robot TCP port (default {ROBOT_PORT})")
    args = parser.parse_args()

    recorder = Recorder(args.save, args.quiet)

    try:
        while True:
            sock = connect(args.ip, args.port)
            buffer = ""
            try:
                while True:
                    data = sock.recv(4096)
                    if not data:
                        print("Robot closed the connection.")
                        break
                    buffer += data.decode("ascii", errors="replace")
                    # TCP can split or join lines, so only handle complete ones.
                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        line = line.strip()
                        if line:
                            recorder.handle(line)
            except OSError as error:
                print(f"Connection lost ({error}).")
            finally:
                sock.close()
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        recorder.close()


if __name__ == "__main__":
    main()
