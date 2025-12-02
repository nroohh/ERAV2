import serial
import time

port = "/dev/tty.ESP32test"
baudrate = 9600

ser = serial.Serial(port, baudrate, timeout=1)
time.sleep(2)  # Allow time for connection

try:
    while True:
        msg = input("Send: ")
        if msg == "q":
            break
        ser.write(msg.encode())
        print("Waiting for response...")
        response = ser.readline().decode().strip()
        print(f"Received: {response}")
except KeyboardInterrupt:
    ser.close()
    print("Disconnected.")
