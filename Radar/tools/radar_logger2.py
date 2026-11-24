
"""Log radar TTC tracker UART records from hwa.c to CSV.

Expected firmware record:
Frame,TID,X_mm,Y_mm,Vy_mmps,TTC_ms,result

Example:
120,1,250,1800,-650,2769,WARNING
"""

import argparse
import csv
from datetime import datetime

import serial


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

    filename = datetime.now().strftime(
        "radar_%Y%m%d_%H%M%S.csv"
    )

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
                "PC_Time",
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
                        errors="ignore"
                    ).strip()

                    if not line:
                        continue

                    fields = line.split(",")

                    # Firmware data contains exactly 7 fields
                    if len(fields) != 7:
                        continue

                    # Ignore UART header
                    if fields[0].strip() == "Frame":
                        continue

                    # Convert numeric fields
                    try:
                        frame = int(fields[0])
                        tid = int(fields[1])
                        x_mm = int(fields[2])
                        y_mm = int(fields[3])
                        vy_mmps = int(fields[4])
                        ttc_ms = int(fields[5])
                        result = fields[6].strip()

                    except ValueError:
                        continue

                    timestamp = datetime.now().isoformat(
                        timespec="milliseconds"
                    )

                    row = [
                        timestamp,
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
