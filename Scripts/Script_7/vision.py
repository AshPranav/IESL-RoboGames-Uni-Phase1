"""Vision and camera socket helpers."""
import socket
import struct
import numpy as np
import cv2


def recvall(sock: socket.socket, size: int):
    data = b""
    while len(data) < size:
        packet = sock.recv(size - len(data))
        if not packet:
            return None
        data += packet
    return data


def connect_camera(host: str, port: int):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((host, port))
    return sock


def read_frame(sock: socket.socket):
    header = recvall(sock, 4)
    if not header:
        return None
    width, height = struct.unpack("<HH", header)
    payload = recvall(sock, width * height)
    if not payload:
        return None
    frame = np.frombuffer(payload, dtype=np.uint8).reshape((height, width)).copy()
    return frame, int(width), int(height)


def detect_line(frame, thresh=150, area_min=100, area_max=6000, margin_frac=0.25):
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    _, mask = cv2.threshold(blurred, thresh, 255, cv2.THRESH_BINARY)

    height, width = frame.shape[:2]
    margin = int(width * margin_frac)
    mask[:, 0:margin] = 0
    mask[:, width - margin:width] = 0

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    error = 0
    line_found = False

    # convert to color for annotation
    annotated = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < area_min or area > area_max:
            continue
        M = cv2.moments(cnt)
        if M.get("m00", 0) == 0:
            continue
        cx = int(M["m10"] / M["m00"])
        error = cx - (width // 2)
        cv2.circle(annotated, (cx, height // 2), 10, (0, 255, 0), -1)
        line_found = True
        break

    return error, mask, annotated, line_found
