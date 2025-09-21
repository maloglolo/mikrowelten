#!/usr/bin/env python3
import serial
import time

# ----------------------------
# Configuration
# ----------------------------
PORT = '/dev/cu.usbserial-0001'
BAUD = 115200
TIMEOUT = 2  # seconds

# STK500 constants
STK_START = 0x1B
STK_END   = 0x20

STK_INSYNC = 0x14
STK_OK     = 0x10
STK_FAILED = 0x11

STK_GET_SYNC       = 0x30
STK_GET_SIGN_ON    = 0x31
STK_ENTER_PROGMODE = 0x50
STK_LEAVE_PROGMODE = 0x51

# ----------------------------
# Helper functions
# ----------------------------
def calc_checksum(cmd, payload):
    csum = cmd
    for b in payload:
        csum ^= b
    csum ^= (len(payload) >> 8) & 0xFF
    csum ^= len(payload) & 0xFF
    return csum

def build_frame(cmd, payload=b''):
    frame = bytearray()
    frame.append(STK_START)
    frame.append(cmd)
    frame.append((len(payload) >> 8) & 0xFF)
    frame.append(len(payload) & 0xFF)
    frame.extend(payload)
    frame.append(calc_checksum(cmd, payload))
    frame.append(STK_END)
    return frame

def send_frame(ser, cmd, payload=b''):
    frame = build_frame(cmd, payload)
    ser.write(frame)
    print("Sending frame:", ' '.join(f"{b:02X}" for b in frame))

def read_response(ser):
    resp = ser.read(64)  # read up to 64 bytes
    if not resp:
        print("No response")
        return

    print("Raw bytes:", ' '.join(f"{b:02X}" for b in resp))

    if resp[0] == STK_INSYNC:
        status = resp[1]
        if status == STK_OK:
            print("Response: INSYNC OK")
        elif status == STK_FAILED:
            print("Response: INSYNC FAILED")
        else:
            # Could be GET_SIGN_ON string
            try:
                s = bytes(resp[1:-1]).decode()
                print("Response string:", s)
            except:
                print(f"Response: INSYNC unknown 0x{status:02X}")

# ----------------------------
# Main test sequence
# ----------------------------
def main():
    with serial.Serial(PORT, BAUD, timeout=TIMEOUT) as ser:
        print("ESP32 STK500 tester connected\n")

        for cmd, name in [(STK_GET_SYNC, "GET_SYNC"),
                          (STK_GET_SIGN_ON, "GET_SIGN_ON"),
                          (STK_ENTER_PROGMODE, "ENTER_PROGMODE"),
                          (STK_LEAVE_PROGMODE, "LEAVE_PROGMODE")]:
            print(f"\nSending {name}...")
            send_frame(ser, cmd)
            read_response(ser)
            time.sleep(0.1)

if __name__ == "__main__":
    main()
