
"""Log radar TTC tracker UART records from hwa.c to CSV.

Expected firmware records:

Frame,TID,X_mm,Y_mm,Vy_mmps,TTC_ms,result

Target:
120,1,250,1800,-650,2769,WARNING

No target:
121,-1,,,,,NO_TARGET
"""

import argparse
import csv

import serial


def parse_integer(value):
    """Convert a numeric field or preserve an empty field."""
    value = value.strip()

    if value == "":
        return ""

    return int(value)


def main():
    parser = argparse.ArgumentParser(
        description="Log radar TTC tracker UART data to CSV"
    )

    parser.add_argument(
        "port",
        help="Serial port, for example COM6"
    )

    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="UART baud rate"
    )

    args = parser.parse_args()

    filename = "radar_tracker.csv"

    try:
        with serial.Serial(
            args.port,
            args.baud,
            timeout=1
        ) as ser, open(
            filename,
            "w",
            newline="",
            encoding="utf-8"
        ) as stream:

            writer = csv.writer(stream)

            # CSV header matching hwa.c
            writer.writerow([
                "Frame",
                "TID",
                "X_mm",
                "Y_mm",
                "Vy_mmps",
                "TTC_ms",
                "Result"
            ])

            print(f"Logging {args.port} @ {args.baud} baud")
            print(f"CSV file: {filename}")
            print("Waiting for radar tracker data...")
            print("Press Ctrl+C to stop logging.\n")

            try:
                while True:

                    line = ser.readline().decode(
                        "ascii",
                        errors="replace"
                    ).strip()

                    if not line:
                        continue

                    fields = line.split(",")

                    # Firmware must send exactly 7 fields
                    if len(fields) != 7:
                        print(f"INVALID FORMAT: {line}")
                        continue

                    # Ignore firmware header
                    if fields[0].strip() == "Frame":
                        continue

                    try:
                        frame = int(fields[0].strip())
                        tid = int(fields[1].strip())

                        x_mm = parse_integer(fields[2])
                        y_mm = parse_integer(fields[3])
                        vy_mmps = parse_integer(fields[4])
                        ttc_ms = parse_integer(fields[5])

                        result = fields[6].strip()

                    except ValueError:
                        print(f"INVALID DATA: {line}")
                        continue

                    # Validate no-target record
                    if tid == -1:
                        if result != "NO_TARGET":
                            continue

                    row = [
                        frame,
                        tid,
                        x_mm,
                        y_mm,
                        vy_mmps,
                        ttc_ms,
                        result
                    ]

                    writer.writerow(row)
                    stream.flush()

                    print(line)

            except KeyboardInterrupt:
                print("\nLogging stopped by user.")

    except serial.SerialException as e:
        print(f"Serial port error: {e}")

    print(f"CSV saved: {filename}")


if __name__ == "__main__":
    main()
