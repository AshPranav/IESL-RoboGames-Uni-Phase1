import socket
import struct
import numpy as np
import cv2

# Simulation / Drone Address
HOST = "127.0.0.1" 
PORT = 5599

def get_video_feed():
    # 1. Create the socket connection
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect((HOST, PORT))
        print(f"✅ Connected to video server at {HOST}:{PORT}")
    except Exception as e:
        print(f"❌ Connection failed: {e}")
        return

    while True:
        # 2. Read the Header (Width and Height - 4 bytes)
        header = sock.recv(4)
        if not header: break
        width, height = struct.unpack('<HH', header)

        # 3. Read the Payload (The actual pixels)
        # Note: If your sim sends Grayscale, use size = w*h
        # If it sends Color, use size = w*h*3
        size = width * height 
        data = b""
        while len(data) < size:
            packet = sock.recv(size - len(data))
            if not packet: break
            data += packet

        # 4. Convert Bytes to Image Matrix
        frame = np.frombuffer(data, dtype=np.uint8).reshape((height, width))

        # --- STEP 2: FIND THE LINE (THE MATH) ---
        
        # A. Region of Interest (ROI) - Look at the bottom 1/3 of the screen
        roi_start = int(height * 0.7)
        roi = frame[roi_start:height, :]

        # B. Thresholding (Finding White Line on Dark Ground)
        # If your line is black, change THRESH_BINARY to THRESH_BINARY_INV
        _, mask = cv2.threshold(roi, 200, 255, cv2.THRESH_BINARY)

        # C. Calculate Centroid (Center of the white pixels)
        M = cv2.moments(mask)
        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])
            # Calculate Error (Distance from the center of the screen)
            error = cx - (width // 2)
            
            # Visuals
            cv2.circle(frame, (cx, roi_start + 20), 5, (255), -1)
            cv2.putText(frame, f"Error: {error}", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255), 2)
        else:
            print("Line Lost!")

        # 5. Show the windows
        cv2.imshow("Drone Feed", frame)
        cv2.imshow("Binary Mask (What the drone sees)", mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    sock.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    get_video_feed()