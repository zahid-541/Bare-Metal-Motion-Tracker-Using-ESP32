import cv2
import serial
import struct
import time
import numpy as np

# ===== 1. CONFIGURATION =====
PORT        = 'COM13'         # <-- CHANGE THIS to your ESP32's COM Port
BAUD        = 4000000         # Must match the ESP32 setup()
VIDEO_PATH  = 'video2.mp4'    
WIDTH       = 160
HEIGHT      = 120
FRAME_SIZE  = WIDTH * HEIGHT
MAX_BOXES   = 8

# Protocol Bytes
SYNC      = bytes([0xAA, 0xBB, 0xCC, 0xDD])
ACK_BYTE  = 0x5A
BOOT_BYTE = 0xEE

# ===== 2. HELPER FUNCTIONS =====
def xor_sum(data: bytes) -> int:
    arr = np.frombuffer(data, dtype=np.uint8)
    return int(np.bitwise_xor.reduce(arr))

def read_exact(ser, n, timeout=2.0):
    buf = bytearray()
    deadline = time.time() + timeout
    while len(buf) < n:
        chunk = ser.read(n - len(buf))
        if chunk:
            buf.extend(chunk)
        elif time.time() > deadline:
            return None
    return bytes(buf)

def recv_response(ser):
    # Read the 5-byte header from ESP32
    hdr = read_exact(ser, 5, timeout=2.0)
    if hdr is None or hdr[0] != ACK_BYTE:
        return None
    
    status = hdr[3]
    num_boxes = hdr[4]
    
    boxes = []
    if num_boxes > 0 and num_boxes <= MAX_BOXES:
        # Read the exact number of box bytes (8 bytes per box)
        body = read_exact(ser, num_boxes * 8, timeout=2.0)
        if body is not None:
            for i in range(num_boxes):
                off = i * 8
                mn_x, mn_y, mx_x, mx_y = struct.unpack_from('<HHHH', body, off)
                boxes.append((mn_x, mn_y, mx_x, mx_y))
    return (status, boxes)

# ===== 3. MAIN PIPELINE =====
def main():
    print(f"[INFO] Connecting to ESP32 on {PORT}...")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.5)
    except serial.SerialException as e:
        print(f"Cannot open {PORT}: {e}")
        return

    # Wait for ESP32 to reboot and settle
    time.sleep(2)
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    print(f"[INFO] Opening video: {VIDEO_PATH}")
    cap = cv2.VideoCapture(VIDEO_PATH)
    if not cap.isOpened():
        print("Cannot open video file.")
        ser.close()
        return

    frame_id = 1
    t_start = time.time()
    frames_done = 0

    try:
        while True:
            ret, color_frame = cap.read()
            if not ret:
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                continue

            # 1. Prepare the payload
            gray = cv2.cvtColor(color_frame, cv2.COLOR_BGR2GRAY)
            small = cv2.resize(gray, (WIDTH, HEIGHT))
            payload = small.tobytes()
            s = xor_sum(payload)

            # 2. Blast the frame to the ESP32
            ser.write(SYNC + struct.pack('<H', frame_id) + payload + bytes([s]))

            # 3. Wait for the ESP32 to do the math and reply
            result = recv_response(ser)
            
            # 4. Draw the results on the PC screen
            display = cv2.resize(color_frame, (640, 480))
            sx = 640 / WIDTH
            sy = 480 / HEIGHT

            if result is None:
                cv2.putText(display, "WAITING FOR ESP32...", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            else:
                status, boxes = result
                if status == 0xFF:
                    cv2.putText(display, "CHECKSUM FAIL", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                else:
                    for i, (mn_x, mn_y, mx_x, mx_y) in enumerate(boxes):
                        x1, y1 = int(mn_x * sx), int(mn_y * sy)
                        x2, y2 = int(mx_x * sx), int(mx_y * sy)
                        cv2.rectangle(display, (x1, y1), (x2, y2), (0, 255, 0), 3)

            # FPS Math
            frames_done += 1
            fps = frames_done / (time.time() - t_start)
            cv2.putText(display, f"Overall FPS: {fps:.1f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

            cv2.imshow("Hardware-in-the-Loop Test", display)
            
            # Give the serial port a tiny breathing room
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            frame_id = (frame_id + 1) & 0xFFFF
            if frame_id == 0: frame_id = 1

    finally:
        cap.release()
        ser.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()