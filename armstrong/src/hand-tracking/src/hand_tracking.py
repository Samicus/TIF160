#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
import cv2
import mediapipe as mp
import time
import numpy as np
from numpy.core.numeric import zeros_like
from mediapipe.python.solutions.hands import HandLandmark

#init node and publisher
pub = rospy.Publisher('hand_tracking', Float64MultiArray, queue_size=1)
rospy.init_node('hand-tracking', anonymous=True)
rate = rospy.Rate(30) # 30hz

message_to_publish = Float64MultiArray()

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
wrist_capture_pos = np.zeros((2,1))
arm_angle = 0.
pinky_val = 0.
ring_val = 0.
middle_val = 0.
index_val = 0.
thumb_val = 0.
wrist_val = 0.
scaled_thumb = 0.
scaled_index  = 0.
scaled_middle = 0.
scaled_ring = 0.
scaled_pinky = 0.
scaled_wrist = 0.
max_pinky_val = 0.01 
max_ring_val = 0.01
max_middle_val = 0.01
max_index_val = 0.01
max_thumb_val = 0.01
max_wrist_val = 0.01

hand_nodes = np.ones((3,21))

def update_max_val(new_val, max_val, key):

    THRESHOLDS = {  "thumb":    1.4,
                    "index":    1.9,
                    "middle":   2.3,
                    "ring":     1.9,
                    "pinky":    1.6,
                    "wrist":    0.7  }

    if new_val > max_val and new_val < THRESHOLDS[key]:
        return new_val
    else:
        return max_val

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

max_thumb_val = 2
max_index_val = 2
max_middle_val = 2
max_ring_val = 2
max_pinky_val = 2

# ===== MAIN LOOP ======
while True: #not rospy.is_shutdown():
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
            wrist_pos_vec = np.copy(hand_nodes[:,HandLandmark.WRIST])

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

            # Convert to reference system where y axis is positive upwards (default is positive downwards)
            wrist_capture_pos = h - wrist_pos_vec

            # The more convoluted but accurate way:
            #print(wrist_capture_pos)
            #arm_angle = np.arctan2(wrist_capture_pos[1], wrist_capture_pos[0]) * 180 / pi

            ELBOW_COEFFICIENT = 90

            # The easier way: just scale the angle by the y-position vs image height (max is 90 degrees)
            arm_angle = ELBOW_COEFFICIENT * wrist_capture_pos[1] / h - 10

            # Elbow range
            arm_angle = max(arm_angle, 30)
            arm_angle = min(arm_angle, 60)

            # Calculate rough values
            pinky_val = np.linalg.norm(hand_nodes[:,HandLandmark.PINKY_TIP] - hand_nodes[:,HandLandmark.WRIST]) / base_vec_draw_length
            ring_val = np.linalg.norm(hand_nodes[:,HandLandmark.RING_FINGER_TIP] - hand_nodes[:,HandLandmark.WRIST])  / base_vec_draw_length
            middle_val = np.linalg.norm(hand_nodes[:,HandLandmark.MIDDLE_FINGER_TIP] - hand_nodes[:,HandLandmark.WRIST])  / base_vec_draw_length
            index_val = np.linalg.norm(hand_nodes[:,HandLandmark.INDEX_FINGER_TIP] - hand_nodes[:,HandLandmark.WRIST])  / base_vec_draw_length
            thumb_val = np.linalg.norm(hand_nodes[:,HandLandmark.THUMB_TIP] - hand_nodes[:,HandLandmark.PINKY_MCP])  / base_vec_draw_length
            wrist_val = np.linalg.norm(hand_nodes[:,HandLandmark.INDEX_FINGER_PIP] - hand_nodes[:,HandLandmark.PINKY_PIP]) / base_vec_draw_length
            
            max_thumb_val = update_max_val(thumb_val, max_thumb_val, "thumb")
            max_index_val = update_max_val(index_val, max_index_val, "index")
            max_middle_val = update_max_val(middle_val, max_middle_val, "middle")
            max_ring_val = update_max_val(ring_val, max_ring_val, "ring")
            max_pinky_val = update_max_val(pinky_val, max_pinky_val, "pinky")
            max_wrist_val = update_max_val(wrist_val, max_wrist_val, "wrist")

            WRIST_ZERO = 0.4

            scaled_thumb = (thumb_val / max_thumb_val) * 180
            scaled_index = (index_val / max_index_val) * 180
            scaled_middle = (middle_val / max_middle_val) * 180
            scaled_ring = (ring_val / max_ring_val) * 180
            scaled_pinky = (pinky_val / max_pinky_val) * 180
            scaled_wrist = max(0, (wrist_val - WRIST_ZERO) / (max_wrist_val-WRIST_ZERO)) * 180

            # PUBLISH HERE
            message_to_publish.data = [ scaled_thumb, 
                                        scaled_index, 
                                        scaled_middle, 
                                        scaled_ring, 
                                        scaled_pinky, 
                                        scaled_wrist,
                                        arm_angle]
            pub.publish(message_to_publish)
            rate.sleep()

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
    cv2.putText(img_canvas,f"Pinky: {pinky_val:.5f}", (7 * w//10, h // 10), cv2.FONT_HERSHEY_PLAIN, 2, (255,0,255), 3)
    cv2.putText(img_canvas,f"Ring: {ring_val:.5f}", (7 * w//10, 2 * h // 10), cv2.FONT_HERSHEY_PLAIN, 2, (255,0,255), 3)
    cv2.putText(img_canvas,f"Middle: {middle_val:.5f}", (7 * w//10, 3 * h // 10), cv2.FONT_HERSHEY_PLAIN, 2, (255,0,255), 3)
    cv2.putText(img_canvas,f"Index: {index_val:.5f}", (7 * w//10, 4 * h // 10), cv2.FONT_HERSHEY_PLAIN, 2, (255,0,255), 3)
    cv2.putText(img_canvas,f"Thumb: {thumb_val:.5f}", (7 * w//10, 5 * h // 10), cv2.FONT_HERSHEY_PLAIN, 2, (255,0,255), 3)
    #cv2.putText(img_canvas,f"Angle: {arm_angle:.5f}", (7 * w//10, 6 * h // 10), cv2.FONT_HERSHEY_PLAIN, 2, (255,0,255), 3)
    cv2.putText(img_canvas,f"Wrist: {wrist_val:.5f}", (7 * w//10, 6 * h // 10), cv2.FONT_HERSHEY_PLAIN, 2, (255,0,255), 3)

    # Overlay the camera image on the canvas
    img_scale_factor = 3
    scaled_width = w // img_scale_factor
    scaled_height = h // img_scale_factor
    scaled_img = cv2.resize(img, (scaled_width, scaled_height))
    imgpos_x = w - scaled_width
    imgpos_y = h - scaled_height
    img_canvas[imgpos_y:imgpos_y+scaled_img.shape[0],imgpos_x:imgpos_x+scaled_img.shape[1]] = scaled_img

    # Show the complete canvas
    cv2.imshow("Image", img_canvas)
    cv2.waitKey(1)