import serial, csv, time, argparse


def stream(out_file, port, baud):
    ser = serial.Serial(port, baud)
    with open(out_file, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["ax", "ay", "az", "gx", "gy", "gz"])
        while True:
            line = ser.readline().decode().strip().split(",")
            if len(line) == 6:
                writer.writerow(line)
                print(line)


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--port", required=True)
    p.add_argument("--baud", default=115200)
    p.add_argument("--out", default="data/raw/session.csv")
    args = p.parse_args()
    stream(args.out, args.port, args.baud)
