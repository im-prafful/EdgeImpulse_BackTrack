import serial, argparse, time, csv


def stream(port, baud, out):
    ser = serial.Serial(port, baud, timeout=1)
    started = False
    with open(out, "w", newline="") as f:
        writer = csv.writer(f)
        while True:
            line = ser.readline().decode(errors="ignore").strip()
            if not line:
                continue
            if line == "START_CSV":
                started = True
                print("Start seen")
                continue
            if not started:
                continue
            if line == "END_CSV":
                print("End seen")
                break
            parts = line.split(",")
            writer.writerow(parts)
            print(parts)


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--port", required=True)
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--out", default="data/raw/session.csv")
    args = p.parse_args()
    stream(args.port, args.baud, args.out)
