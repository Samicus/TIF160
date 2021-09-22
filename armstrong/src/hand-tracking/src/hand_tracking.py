#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
import cv2
import mediapipe as mp
import time
import math

class handDetector():
    def __init__(self, mode = False, maxHands = 2, detectionCon = 0.5, trackCon = 0.5):
        self.mode = mode
        self.maxHands = maxHands
        self.detectionCon = detectionCon
        self.trackCon = trackCon

        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(self.mode, self.maxHands, self.detectionCon, self.trackCon)
        self.mpDraw = mp.solutions.drawing_utils

    def findHands(self,img, draw = True):
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(imgRGB)
        # print(results.multi_hand_landmarks)

        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:
                if draw:
                    self.mpDraw.draw_landmarks(img, handLms, self.mpHands.HAND_CONNECTIONS)
        return img

    def findPosition(self, img, handNo = 0, draw = True):

        lmlist = []
        if self.results.multi_hand_landmarks:
            myHand = self.results.multi_hand_landmarks[handNo]
            for id, lm in enumerate(myHand.landmark):
                h, w, c = img.shape
                cx, cy = int(lm.x * w), int(lm.y * h)
                lmlist.append([id, cx, cy])
                if draw:
                    cv2.circle(img, (cx, cy), 3, (255, 0, 255), cv2.FILLED)
        return lmlist

def get_distance(tip_x, tip_y, ref_x, ref_y):
    if (tip_x < ref_x) or (tip_y < ref_y):
        dist = math.sqrt((tip_x-ref_x)**2 + (tip_y-ref_y)**2)
    else:
        dist = 0
    return dist


def get_fingers_output(lmlist):

    thumb_tip_x = lmlist[4][1]
    thumb_tip_y = lmlist[4][2]
    thumb_ref_x = lmlist[1][1]
    thumb_ref_y = lmlist[1][2]
    thumb_angle = get_distance(thumb_tip_x, thumb_tip_y, thumb_ref_x, thumb_ref_y) 

    index_tip_x = lmlist[8][1]
    index_tip_y = lmlist[8][2]
    index_ref_x = lmlist[5][1]
    index_ref_y = lmlist[5][2]
    index_angle = get_distance(index_tip_x, index_tip_y, index_ref_x, index_ref_y)

    middle_tip_x = lmlist[12][1]
    middle_tip_y = lmlist[12][2]
    middle_ref_x = lmlist[9][1]
    middle_ref_y = lmlist[9][2]
    middle_angle = get_distance(middle_tip_x, middle_tip_y, middle_ref_x, middle_ref_y)

    ring_tip_x = lmlist[16][1]
    ring_tip_y = lmlist[16][2]
    ring_ref_x = lmlist[13][1]
    ring_ref_y = lmlist[13][2]
    ring_angle = get_distance(ring_tip_x, ring_tip_y, ring_ref_x, ring_ref_y)

    pinky_tip_x = lmlist[20][1]
    pinky_tip_y = lmlist[20][2]
    pinky_ref_x = lmlist[17][1]
    pinky_ref_y = lmlist[17][2]
    pinky_angle = get_distance(pinky_tip_x, pinky_tip_y, pinky_ref_x, pinky_ref_y)

    return thumb_angle, index_angle, middle_angle, ring_angle, pinky_angle


def main():
    #init node and publisher
    pub = rospy.Publisher('hand_tracking', Float64MultiArray, queue_size=1)
    rospy.init_node('hand-tracking', anonymous=True)
    rate = rospy.Rate(30) # 30hz

    message_to_publish = Float64MultiArray()

    pTime = 0
    cTime = 0
    cap = cv2.VideoCapture(0)
    detector = handDetector()

    while not rospy.is_shutdown():
        success, img = cap.read()
        img = detector.findHands(img)
        lmlist = detector.findPosition(img)
        if len(lmlist) != 0:
            thumb_angle, index_angle, middle_angle, ring_angle, pinky_angle = get_fingers_output(lmlist)
            #print(f"Thumb angle: {thumb_angle}\nIndex angle: {index_angle}\nMiddle angle: {middle_angle}\nRing angle: {ring_angle}\nPinky angle: {pinky_angle}\n")
            message_to_publish.data = [thumb_angle, index_angle, middle_angle, ring_angle, pinky_angle]
            pub.publish(message_to_publish)
            rate.sleep()

        cTime = time.time()
        fps = 1 / (cTime - pTime)
        pTime = cTime

        cv2.putText(img, str(int(fps)), (10, 70), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3)

        cv2.imshow("Image", img)
        cv2.waitKey(1)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
