import cv2


class Camera:
    def __init__(self):
        self.cap = None
        self.frame = None
        self.init_web_camera()
                                  
    def init_web_camera(self):
        # Initialize the OpenCV camera.
        self.cap = cv2.VideoCapture(0)
    def init_raspi_camera(self):
        pass

    def get_camera_img(self):
        ret,img = self.cap.read()
        if ret:
            return img
        else:
            return -1
