import sys, time
from datetime import datetime

try:
    import serial
    from serial.tools import list_ports
    print("serial module:", getattr(serial, "__file__", serial))
except Exception as e:
    print("PySerial import failed; install with: python -m pip install pyserial")
    raise

def pick_port(preferred="COM5"):
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found. Plug the board, install drivers, or try another cable.")
        return None
    names = [p.device for p in ports]
    print("Available ports:", names)
    if preferred in names:
        return preferred
    # Heuristic: pick first with "USB" or "UART"
    for p in ports:
        desc = (p.description or "").lower()
        if "usb" in desc or "uart" in desc or "cp210" in desc or "ch340" in desc or "silabs" in desc:
            return p.device
    return names[0]

PORT = pick_port("COM5")
if PORT is None:
    sys.exit(1)

BAUD = 115200
TIMEOUT = 2

# Confirm Serial exists
if not hasattr(serial, "Serial"):
    print("serial.Serial not found; pyserial install is broken. Reinstall pyserial.")
    sys.exit(1)

ts = datetime.now().strftime("%Y%m%d_%H%M%S")
fname = f"D:\\FolderA\\Datasets\ \mpu6050_run_{ts}.csv"

try:
    ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
except Exception as e:
    print(f"Could not open {PORT}: {e}")
    sys.exit(1)

print(f"Opened {PORT} at {BAUD}. Waiting for START_CSV...")
start_seen = False
line_count = 0

with ser:
    with open(fname, "w", buffering=1, encoding="utf-8") as f:
        while True:
            try:
                raw = ser.readline()
            except KeyboardInterrupt:
                print("Interrupted.")
                break
            if not raw:
                continue
            line = raw.decode("utf-8", errors="ignore").strip()

            if not start_seen:
                if line == "START_CSV":
                    start_seen = True
                    print(f"Started logging to {fname}")
                continue

            if line == "END_CSV":
                print("END_CSV received. Closing file.")
                break

            f.write(line + "\n")
            line_count += 1

print(f"Wrote {line_count} data lines to {fname}")
