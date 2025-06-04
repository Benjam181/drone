import serial
import csv
from datetime import datetime

# === CONFIG ===
# To find port : ls /dev/tty.*
PORT = '/dev/tty.usbmodem1303'  # Change to your serial port (e.g., '/dev/ttyUSB0' on Linux)
BAUDRATE = 115200
CSV_FILE = "logFile.csv"

# === INIT ===
ser = serial.Serial(PORT, BAUDRATE, timeout=1)
print(f"Logging from {PORT} to {CSV_FILE}...")

with open(CSV_FILE, 'w', newline='') as csvfile:
    csv_writer = csv.writer(csvfile)
    # Write header
    csv_writer.writerow(["timestamp", "M1", "M2", "M3", "M4", "pitch", "roll", "yaw", "thrust", "accel_x", "accel_y", "accel_z","angle_x", "angle_y", "angle_z"])

    try:
        while True:
            line = ser.readline().decode('utf-8').strip()
            if line:
                try:
                    values = line.split(",")
                    csv_writer.writerow(values)
                    print(values)
                except ValueError:
                    print(f"Skipped malformed line: {line}")
    except KeyboardInterrupt:
        print("Logging stopped.")

ser.close()