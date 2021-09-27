import cv2
import mediapipe as mp
import time
import numpy as np
from numpy.core.numeric import zeros_like

cap = cv2.VideoCapture(0)

mpHands = mp.solutions.hands
hands = mpHands.Hands(static_image_mode=False,
                      max_num_hands=1,
                      min_detection_confidence=0.5,
                      min_tracking_confidence=0.5)
mpDraw = mp.solutions.drawing_utils

pTime = 0
cTime = 0

hand_coords = [np.zeros(2) for _ in range(21)]
hand_screen_coords = np.zeros_like(hand_coords)
scaled_screen_coords = np.zeros_like(hand_coords)

def set_coordinates(id, val) :
    hand_coords[id] = np.array([val.x * w, val.y * h])
    #np.matmul(screen_mat, hand_coords[id], hand_screen_coords[id])

def translate(vec):
    for i, coord in enumerate(hand_coords):
        hand_coords[i] = coord + vec

def get_rot_matrix(angle):
    return np.array(
        [[np.cos(angle), -np.sin(angle)],
        [np.sin(angle), np.cos(angle)]])

def rotate(angle):
    mat = get_rot_matrix(-angle)
    for i, coord in enumerate(hand_coords):
        np.matmul(mat, coord, hand_coords[i])

def normalize_all(draw_length: int):
    base_length = hand_coords[middle_knuckle_index][0]
    for i, coord in enumerate(hand_coords):
        hand_coords[i] = coord * (draw_length / base_length)

while True:
    success, img = cap.read()
    imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    #imgblank = np.zeros(img.shape, np.uint8)
    imgblank = np.zeros(img.shape, np.uint8)
    results = hands.process(imgRGB)
    
    h, w, c = imgblank.shape
    
    base_offset = np.array([0, h // 2])
    wrist_index = 0
    middle_knuckle_index = 9

    base_vec_draw_length = 300

    if results.multi_hand_landmarks:
        for handLms in results.multi_hand_landmarks:

            # Convert to vectors
            for id, lm in enumerate(handLms.landmark):
                set_coordinates(id, lm)

            # Get the wrist vector
            wrist_pos = hand_coords[wrist_index]

            # Set the wrist position as origo
            translate(-wrist_pos)

            # Get the angle of the wrist-to-middle-knucle vector
            knuckle_pos = hand_coords[middle_knuckle_index]
            angle = np.arctan2(knuckle_pos[1],knuckle_pos[0])
            
            # Rotate all nodes around origo
            rotate(angle)

            # Normalize all
            normalize_all(base_vec_draw_length)

            # Get the screen coordinates
            translate(base_offset)
            
            for vec in hand_coords:
                clr = (255,0,255) if id == 1 else (255,255,255)
                cv2.circle(imgblank, (int(vec[0]), int(vec[1])), 3, clr, cv2.FILLED)

            cv2.line(imgblank, hand_coords[wrist_index].astype(int), hand_coords[middle_knuckle_index].astype(int), (255,0,0), 4)
            
    cTime = time.time()
    fps = 1/(cTime-pTime)
    pTime = cTime

    cv2.putText(imgblank,str(int(fps)), (10,70), cv2.FONT_HERSHEY_PLAIN, 3, (255,0,255), 3)

    cv2.imshow("Image", imgblank)
    cv2.waitKey(1)