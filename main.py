import serial
import time

# -- config --
PORT = "/dev/tty.usbmodem2048327E55531"
BAUD = 115200
FEED_RATE = 5000  # mm/min, tune to taste

points = [
    (10, 20, 5),
    (50, 80, 10),
    (100, 50, 0),
    (0, 0, 0),
]

def send(ser, cmd):
    line = cmd.strip() + "\n"
    ser.write(line.encode())
    time.sleep(0.05)
    response = ser.read_all().decode(errors="ignore").strip()
    if response:
        print(f"  >> {response}")
    return response

def wait_for_ok(ser, timeout=30):
    """Block until 'ok' comes back — Marlin/Klipper both send it."""
    start = time.time()
    buf = ""
    while time.time() - start < timeout:
        buf += ser.read_all().decode(errors="ignore")
        if "ok" in buf.lower():
            return True
        time.sleep(0.05)
    raise TimeoutError(f"No 'ok' received within {timeout}s. Last buffer: {buf!r}")

with serial.Serial(PORT, BAUD, timeout=1) as ser:
    time.sleep(2)  # let the board wake up after connect
    ser.read_all()  # flush startup noise

    # print("Homing...")
    # send(ser, "G28")           # home all axes
    # wait_for_ok(ser, timeout=60)

    send(ser, "G90")           # absolute positioning
    wait_for_ok(ser)

    for x, y, z in points:
        cmd = f"G1 X{x} Y{y} Z{z} F{FEED_RATE}"
        print(f"Moving to X{x} Y{y} Z{z}")
        send(ser, cmd)
        send(ser, "M400")      # wait for moves to finish
        wait_for_ok(ser)

    print("Done.")