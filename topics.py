import time
import random

def simulate_gesture_receiver():
    print("Gesture Receiver Node Started")
    print("Waiting for gesture messages from main.py...")
    
    # Simulated gestures that match what we see in main.py
    gestures = ['stir', 'pour', 'flip']
    
    try:
        while True:
            # Simulate receiving a gesture with some random delay
            time.sleep(random.uniform(0.5, 2.0))
            gesture = random.choice(gestures)
            print(f"Received gesture: {gesture} from main.py")
            
    except KeyboardInterrupt:
        print("\nGesture Receiver Node Shutting Down...")

if __name__ == "__main__":
    simulate_gesture_receiver() 