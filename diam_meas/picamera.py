import cv2
import picamera    

class Camera:
    def __init__(self):
        self.cap = None
        self.frame = None
        self.init_RP_camera()  #Initializing Picamera

    def init_web_camera(self):
        self.cap = cv2.VideoCapture(0)

    def init_RP_camera(self):
        self.raw_capture = picamera.PiRawCapture(camera=picamera.PiCamera())
        # Set camera resolution and frame rate.
        self.camera = self.raw_capture.camera
        self.camera.resolution = (640, 480)
        self.camera.framerate = 30   

    def get_camera_img(self):
        self.camera.capture(self.raw_capture, format='bgr')
        self.frame = self.raw_capture.array
        self.raw_capture.truncate(0)
        if self.frame is not None:
            return self.frame
        else:
            return -1