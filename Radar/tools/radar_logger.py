"""Log radar tracker UART records to CSV.

Expected target record:
T,FRAME,NUM_TARGETS,TID,X_MM,Y_MM,VX_MMPS,VY_MMPS
"""

import argparse
import csv
from datetime import datetime

import serial


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("port", help="Serial port, for example COM7")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    filename = datetime.now().strftime("radar_%Y%m%d_%H%M%S.csv")

    with serial.Serial(args.port, args.baud, timeout=1) as ser, open(
        filename, "w", newline="", encoding="utf-8"
    ) as stream:
        writer = csv.writer(stream)
        writer.writerow(
            ["PC_Time", "Frame", "NumTargets", "TID", "X_mm", "Y_mm", "Vx_mmps", "Vy_mmps"]
        )

        print(f"Logging {args.port} @ {args.baud} baud to {filename}")
        try:
            while True:
                line = ser.readline().decode("ascii", errors="ignore").strip()
                fields = line.split(",")
                if len(fields) != 8 or fields[0] != "T":
                    continue

                timestamp = datetime.now().isoformat(timespec="milliseconds")
                writer.writerow([timestamp] + fields[1:])
                stream.flush()
                print(line)
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()