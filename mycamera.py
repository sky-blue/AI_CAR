from picamera2 import Picamera2
import numpy as np
import cv2
import time

class MyPiCamera():
    def __init__(self, width, height):
        self.cap = None
        self.width = width
        self.height = height
        self.is_open = False

        try:
            camera_info = Picamera2.global_camera_info()
            if not camera_info:
                print("camera not found: connect the camera and reboot Raspberry Pi")
                return

            self.cap = Picamera2()
            self.config = self.cap.create_video_configuration(
                main={"format": "RGB888", "size": (width, height)}
            )
            self.cap.align_configuration(self.config)
            self.cap.configure(self.config)
            self.cap.start()
            time.sleep(1)
            self.is_open = True
        except Exception as e:
            print("camera error:", e)
            self.is_open = False
    
    def read(self, dst=None):
        if dst is None:
            dst = np.empty((self.height, self.width, 3), dtype=np.uint8)
        if self.is_open:
            try:
                dst = self.cap.capture_array()
            except Exception as e:
                print("image error:", e)
                self.is_open = False
        return self.is_open, dst
    
    def isOpened(self):
        return self.is_open
    
    def release(self):
        if self.is_open:
            self.cap.stop()
        self.is_open = False

if __name__ == "__main__":
    cam = MyPiCamera(640, 480)

    try:
        while cam.isOpened():
            ret, image = cam.read()
            if not ret:
                print("image fail.")
                break
            cv2.imshow("mycamera", image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cam.release()
        cv2.destroyAllWindows()

