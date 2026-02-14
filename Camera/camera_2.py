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

    # 1. Create a "Portrait" crop (Focus on the middle 50% of the screen)
    # This cuts out the banners on the left and right edges
    offset = width // 4
    roi = frame[:, offset : width - offset]

    # 2. Use Adaptive Thresholding (Shadow Resistant)
    # It calculates different thresholds for different parts of the image
    mask_roi = cv2.adaptiveThreshold(roi, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                     cv2.THRESH_BINARY, 11, 2)

    # 3. (Optional) Invert if the line is black, but for Yellow-on-Gray usually:
    # If the line looks black in the mask, uncomment the next line:
    # mask_roi = cv2.bitwise_not(mask_roi)

    # 4. Show the result
    cv2.imshow("Drone Camera", frame)
    cv2.imshow("Smarter Mask", mask_roi)

    if cv2.waitKey(1) == 27:  
        break


sock.close()
cv2.destroyAllWindows()
