import cv2
import mediapipe as mp

class GestureDetector:
    def __init__(self):
        self.hands = mp.solutions.hands.Hands(max_num_hands=1)
        self.mp_draw = mp.solutions.drawing_utils

    def detect(self, frame):
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = self.hands.process(frame_rgb)
        gesture = None

        if result.multi_hand_landmarks:
            for hand_landmarks in result.multi_hand_landmarks:
                landmarks = hand_landmarks.landmark

                # Index (8) and Middle (12) fingertips
                ix, iy = landmarks[8].x, landmarks[8].y
                mx, my = landmarks[12].x, landmarks[12].y

                dx = mx - ix
                dy = my - iy

                if abs(dy) < 0.02 and dx < -0.05:
                    gesture = 'stir'
                elif dy < -0.05:
                    gesture = 'flip'
                elif dx > 0.05:
                    gesture = 'pour'

        return gesture
