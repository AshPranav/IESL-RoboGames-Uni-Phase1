import socket
import struct
import numpy as np
import cv2

HOST = "127.0.0.1"   # or drone IP
PORT = 5599


def recvall(sock, size):
    """Receive exactly size bytes"""
    data = b""
    while len(data) < size:
        packet = sock.recv(size - len(data))
        if not packet:
            return None
        data += packet
    return data


# Create TCP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((HOST, PORT))

print("✅ Connected to camera stream")

while True:

    # ---- Read header (4 bytes) ----
    header = recvall(sock, 4)

    if header is None:
        print("❌ Connection lost")
        break

    # Unpack: 2 unsigned shorts (little endian)
    width, height = struct.unpack("<HH", header)

    # ---- Read image payload ----
    img_size = width * height
    payload = recvall(sock, img_size)

    if payload is None:
        print("❌ Frame lost")
        break

    # ---- Convert to image ----
    frame = np.frombuffer(payload, dtype=np.uint8)
    frame = frame.reshape((height, width)).copy()

    # 1. Clean the image a bit to remove 'grain'
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)

    # 2. Thresholding: Look for the brightest parts (The Line)
    # We use a lower threshold (150) to make sure we catch it
    _, mask = cv2.threshold(blurred, 150, 255, cv2.THRESH_BINARY)

    # 3. Show the results
    cv2.imshow("Drone Camera", frame)
    cv2.imshow("Mask (Grayscale Detection)", mask)

    if cv2.waitKey(1) == 27:  
        break


sock.close()
cv2.destroyAllWindows()
