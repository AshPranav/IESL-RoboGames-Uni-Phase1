import socket
import struct
import numpy as np
import cv2

HOST = "127.0.0.1" 
PORT = 5599

def recvall(sock, size):
    data = b""
    while len(data) < size:
        packet = sock.recv(size - len(data))
        if not packet: return None
        data += packet
    return data

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((HOST, PORT))
print("✅ Connected to camera stream")

while True:
    header = recvall(sock, 4)
    if header is None: break
    width, height = struct.unpack("<HH", header)

    img_size = width * height
    payload = recvall(sock, img_size)
    if payload is None: break

    # 1. Step 1: Create writable copy [cite: 136]
    frame = np.frombuffer(payload, dtype=np.uint8).reshape((height, width)).copy()

    # 2. Step 2: Portrait Masking (Ignore side banners) [cite: 155]
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    _, mask = cv2.threshold(blurred, 150, 255, cv2.THRESH_BINARY)
    
    margin = int(width * 0.25)
    mask[:, 0:margin] = 0
    mask[:, width-margin:width] = 0

    # 3. Step 3: Differentiate Line vs Pad 
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    line_target = None
    for cnt in contours:
        area = cv2.contourArea(cnt)
        # Narrow strip is the line [cite: 155]
        if 100 < area < 5000:
            line_target = cnt
        # Large block is the Takeoff/Landing Pad [cite: 154]
        elif area >= 5000:
            cv2.drawContours(frame, [cnt], -1, (127), 2) # Outline the pad

    # 4. Step 4: Calculate Error for ArduPilot 
    if line_target is not None:
        M = cv2.moments(line_target)
        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            cv2.circle(frame, (cx, cy), 10, (255), -1)
            
            error = cx - (width // 2)
            # You will send this 'error' to ArduPilot via MAVLink [cite: 219]
            print(f"Line Center: {cx} | Steering Error: {error}")

    cv2.imshow("Drone Camera", frame)
    cv2.imshow("Mask", mask)

    if cv2.waitKey(1) == 27: break

sock.close()
cv2.destroyAllWindows()