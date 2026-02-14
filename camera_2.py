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

    # 1. Define the portrait slice (center 50% of the screen)
    # This ignores the yellow banners on the walls
    start_col = width // 4
    end_col = width - (width // 4)
    
    # 2. Create the mask ONLY for that center slice
    # We use your successful Gaussian + Threshold logic here
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    _, mask = cv2.threshold(blurred, 150, 255, cv2.THRESH_BINARY)
    
    # Zero out everything outside our 'Portrait' zone
    mask[:, 0:start_col] = 0
    mask[:, end_col:width] = 0

    # 3. Find the center of the line (Centroid)
    M = cv2.moments(mask)
    if M["m00"] > 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        
        # Draw a circle on the original frame so you can see the 'Target'
        cv2.circle(frame, (cx, cy), 10, (255), -1)
        
        # Calculate Error: How far is the line from the screen center?
        error = cx - (width // 2)
        print(f"Error: {error}")

    if cv2.waitKey(1) == 27:  
        break


sock.close()
cv2.destroyAllWindows()
