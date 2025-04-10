# test_cam.py
import cv2

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("❌ Cannot access camera")
else:
    print("✅ Camera accessed")
    ret, frame = cap.read()
    if ret:
        cv2.imshow("Test", frame)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
cap.release()
