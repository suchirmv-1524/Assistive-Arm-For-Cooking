import cv2
from gesture_detector import GestureDetector
from robot_arm_sim import RobotArmSim
from actions import gesture_to_action

def main():
    cap = cv2.VideoCapture(0)
    detector = GestureDetector()
    sim = RobotArmSim()

    print("Starting gesture-controlled robotic arm. Press 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        gesture = detector.detect(frame)
        if gesture:
            action = gesture_to_action.get(gesture)
            if action:
                print(f"Gesture: {gesture} -> Executing: {action}")
                sim.move_arm(action)

        sim.step()
        cv2.imshow("Gesture Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
