import cv2
import numpy as np

# === 1. GENERATE CHESSBOARD PATTERN (9x7 Kotak) ===
squares_x = 9
squares_y = 7
square_size = 150 # pixel per kotak

# Buat papan kosong
board = np.zeros((squares_y * square_size, squares_x * square_size), dtype=np.uint8)
for y in range(squares_y):
    for x in range(squares_x):
        if (x + y) % 2 == 1:
            board[y*square_size:(y+1)*square_size, x*square_size:(x+1)*square_size] = 255
            
# Beri border putih tipis agar deteksi OpenCV lebih mudah di area tepi
border_size = 50
bordered_board = cv2.copyMakeBorder(board, border_size, border_size, border_size, border_size, cv2.BORDER_CONSTANT, value=255)

cv2.imwrite("/home/vtol/vtol_ros2/chessboard.png", bordered_board)
print("[SUKSES] Pola Chessboard berhasil digenerate -> /home/vtol/vtol_ros2/chessboard.png")

# === 2. GENERATE ARUCO MARKER 7x7 (DICT_7X7_50 - ID 0) ===
try:
    # OpenCV 4.7.0+
    dict_7x7 = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_50)
    marker_img = cv2.aruco.generateImageMarker(dict_7x7, 0, 400)
except AttributeError:
    # OpenCV versi lama
    dict_7x7 = cv2.aruco.Dictionary_get(cv2.aruco.DICT_7X7_50)
    marker_img = cv2.aruco.drawMarker(dict_7x7, 0, 400)
    
cv2.imwrite("/home/vtol/vtol_ros2/aruco_marker_id0.png", marker_img)
print("[SUKSES] ArUco Marker ID 0 berhasil digenerate -> /home/vtol/vtol_ros2/aruco_marker_id0.png")
