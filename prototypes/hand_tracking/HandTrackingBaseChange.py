import cv2
import mediapipe as mp
import time
import numpy as np
from numpy.core.numeric import zeros_like
from mediapipe.python.solutions.hands import HandLandmark

cap = cv2.VideoCapture(0)

mpHands = mp.solutions.hands
hands = mpHands.Hands(static_image_mode=False,
                      max_num_hands=1,
                      min_detection_confidence=0.5,
                      min_tracking_confidence=0.5)
mpDraw = mp.solutions.drawing_utils

# Parameters and constants
pTime = 0
cTime = 0
n_nodes = 21

# Data structures
pinky_val = 0.
ring_val = 0.
middle_val = 0.
index_val = 0.
thumb_val = 0.
hand_nodes = np.ones((3,21))

# Vectorized functions
def lms_to_vectors(nodes):
    for i, val in enumerate(nodes):
        hand_nodes[:,i] = np.array([val.x * w, val.y * h, 1])

def translate_vectors(vec):
    trans_mat = np.array([[1, 0, vec[0]],[0, 1, vec[1]],[0, 0, 1]])
    np.matmul(trans_mat, hand_nodes, hand_nodes)

def rotate_vectors(angle):
    rot_mat = np.array(
        [[np.cos(angle), -np.sin(angle), 0],
        [np.sin(angle), np.cos(angle), 0],
        [0, 0, 1]])
    np.matmul(rot_mat, hand_nodes, hand_nodes)

def normalize_vectors(draw_length):
    base_length = hand_nodes[0,HandLandmark.MIDDLE_FINGER_MCP]
    scale_factor = draw_length/base_length
    scaling_mat = np.array([
        [scale_factor,0,0],
        [0,scale_factor,0],
        [0,0,1]])
    np.matmul(scaling_mat, hand_nodes, hand_nodes)

# ===== MAIN LOOP ======
while True:
    success, img = cap.read()
    imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img_canvas = np.zeros(img.shape, np.uint8)
    results = hands.process(imgRGB)
    
    h, w, c = img_canvas.shape
    
    base_offset = np.array([0, h // 2])
    base_offset_vec = np.array([0, h // 2, 1])
    clr = (255,255,255)
    base_vec_draw_length = 300

    if results.multi_hand_landmarks:
        for handLms in results.multi_hand_landmarks:
            # Convert to vectors
            lms_to_vectors(handLms.landmark)

            # Get the wrist vector
            wrist_pos_vec = hand_nodes[:,HandLandmark.WRIST]

            # Set the wrist position as origo
            translate_vectors(-wrist_pos_vec)

            # Get the angle of the wrist-to-middle-knucle vector
            knuckle_pos_vec = hand_nodes[:,HandLandmark.MIDDLE_FINGER_MCP]
            angle_vec = np.arctan2(knuckle_pos_vec[1],knuckle_pos_vec[0])
            
            # Rotate all nodes around origo
            rotate_vectors(-angle_vec)

            # Normalize all
            normalize_vectors(base_vec_draw_length)

            # Get the screen coordinates
            translate_vectors(base_offset_vec)

            # Calculate rough values
            pinky_val = np.linalg.norm(hand_nodes[:,HandLandmark.PINKY_TIP] - hand_nodes[:,HandLandmark.WRIST]) / base_vec_draw_length
            ring_val = np.linalg.norm(hand_nodes[:,HandLandmark.RING_FINGER_TIP] - hand_nodes[:,HandLandmark.WRIST])  / base_vec_draw_length
            middle_val = np.linalg.norm(hand_nodes[:,HandLandmark.MIDDLE_FINGER_TIP] - hand_nodes[:,HandLandmark.WRIST])  / base_vec_draw_length
            index_val = np.linalg.norm(hand_nodes[:,HandLandmark.INDEX_FINGER_TIP] - hand_nodes[:,HandLandmark.WRIST])  / base_vec_draw_length
            thumb_val = np.linalg.norm(hand_nodes[:,HandLandmark.THUMB_TIP] - hand_nodes[:,HandLandmark.WRIST])  / base_vec_draw_length

            # Draw on the original image
            for id, lm in enumerate(handLms.landmark):
                cx, cy = int(lm.x *w), int(lm.y*h)
                cv2.circle(img, (cx,cy), 3, (255,0,255), cv2.FILLED)

            mpDraw.draw_landmarks(img, handLms, mpHands.HAND_CONNECTIONS)

            # Draw points and base line
            for i in range(n_nodes):
                vec = hand_nodes[:,i]
                cv2.circle(img_canvas, (int(vec[0]), int(vec[1])), 3, clr, cv2.FILLED)

            cv2.line(img_canvas, 
                hand_nodes[0:2,HandLandmark.WRIST].astype(int), 
                hand_nodes[0:2,HandLandmark.MIDDLE_FINGER_MCP].astype(int), 
                (255,0,0), 4)

    # Display performance
    cTime = time.time()
    fps = 1/(cTime-pTime)
    pTime = cTime
    
    # Display values
    v_offset = h // 12
    cv2.putText(img_canvas,str(int(fps)), (10,70), cv2.FONT_HERSHEY_PLAIN, 3, (255,0,255), 3)
    cv2.putText(img_canvas,f"Pinky: {pinky_val}", (7 * w//10, h // 10), cv2.FONT_HERSHEY_PLAIN, 2, (255,0,255), 3)
    cv2.putText(img_canvas,f"Ring: {ring_val}", (7 * w//10, 2 * h // 10), cv2.FONT_HERSHEY_PLAIN, 2, (255,0,255), 3)
    cv2.putText(img_canvas,f"Middle: {middle_val}", (7 * w//10, 3 * h // 10), cv2.FONT_HERSHEY_PLAIN, 2, (255,0,255), 3)
    cv2.putText(img_canvas,f"Index: {index_val}", (7 * w//10, 4 * h // 10), cv2.FONT_HERSHEY_PLAIN, 2, (255,0,255), 3)
    cv2.putText(img_canvas,f"Thumb: {thumb_val}", (7 * w//10, 5 * h // 10), cv2.FONT_HERSHEY_PLAIN, 2, (255,0,255), 3)

    # Overlay the camera image on the canvas
    img_scale_factor = 4
    scaled_width = w // img_scale_factor
    scaled_height = h // img_scale_factor
    scaled_img = cv2.resize(img, (scaled_width, scaled_height))
    imgpos_x = w - scaled_width
    imgpos_y = h - scaled_height
    img_canvas[imgpos_y:imgpos_y+scaled_img.shape[0],imgpos_x:imgpos_x+scaled_img.shape[1]] = scaled_img

    # Show the complete canvas
    cv2.imshow("Image", img_canvas)
    cv2.waitKey(1)