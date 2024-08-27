import cv2
import numpy as np
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtCore import QObject

class measurement(QObject):
    minvalue = pyqtSignal(float)
    maxvalue = pyqtSignal(float)
    meaValue = pyqtSignal(float)
    '''
    input : image
    output : min_thickness, max_thickness, mean_thickness, median_thickness
    '''
    def __init__(self):
        self.img = None
        self.min_thickness = 0.0
        self.max_thickness = 0.0
        self.mean_thickness = 0.0
        self.median_thickness = 0.0
        self.pixel_to_micron_factor = np.load('pixel_to_micron_factor.npy')
        self.thresh_img = None
    def start_measurement(self,input_img):
        # read image
        # img = cv2.imread('D:\Pycharme\RPi_Opencv\Cali01 vertical.jpg')
        self.img = input_img

        # convert to grayscale
        gray = cv2.cvtColor(self.img , cv2.COLOR_BGR2GRAY)
        thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)[1]
        
        # get contours
        new_contours = []
        img2 = np.zeros_like(thresh, dtype=np.uint8)
        contour_img = thresh.copy()
        contour_img = cv2.merge([contour_img, contour_img, contour_img])
        contours = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = contours[0] if len(contours) == 2 else contours[1]
        for cntr in contours:
            area = cv2.contourArea(cntr)
            if area > 1000:
                cv2.drawContours(contour_img, [cntr], 0, (0, 0, 255), 1)
                cv2.drawContours(img2, [cntr], 0, (0), -1)
                new_contours.append(cntr)

        # sort contours by area
        cnts_sort = sorted(new_contours, key=lambda x: cv2.contourArea(x), reverse=False)

        # select first (smaller) sorted contour
        first_contour = cnts_sort[0]
        contour_first_img = np.zeros_like(thresh, dtype=np.uint8)
        cv2.drawContours(contour_first_img, [first_contour], 0, (255), -1)
        # thin smaller contour
        # thresh1 = (contour_first_img / 255).astype(np.float64)
        # Invert black and white
        inverted_thresh = cv2.bitwise_not(thresh)
        self.thresh_img = inverted_thresh
        # Rotate clockwise by 90 degrees
        vertical_inverted_img = cv2.rotate(inverted_thresh, cv2.ROTATE_90_CLOCKWISE)
        pixel_num = np.mean(np.count_nonzero(vertical_inverted_img, axis=1))
        # Pixel size * number of pixels, then convert to microns
        print("Mean pixel_num:", pixel_num)


        thickness_values = np.count_nonzero(vertical_inverted_img, axis=1) * self.pixel_to_micron_factor
        self.min_thickness = "{:.4f} microns".format(np.min(thickness_values))
        self.max_thickness = "{:.4f} microns".format(np.max(thickness_values))
        self.mean_thickness = "{:.4f} microns".format(np.mean(thickness_values))
        self.median_thickness = "{:.4f} microns".format(np.median(thickness_values))

        self.max = np.max(thickness_values)
        self.min = np.min(thickness_values)
        self.mean = np.mean(thickness_values)

        print("Minimum Thickness:", self.min_thickness)
        print("Maximum Thickness:", self.max_thickness)
        print("Mean Thickness:", self.mean_thickness)
        print("Median_thickness:", self.median_thickness)
        print("Thickness",self.mean_thickness)

        # # save resulting images
        # cv2.imwrite('thresh.jpg', thresh)
        # cv2.imwrite('lines_filtered.jpg', img2)
        # cv2.imwrite('contour.jpg', contour_img)

        # # show thresh and result
        # cv2.imshow("thresh", thresh)
        # cv2.imshow("contours", contour_img)
        # cv2.imshow("first_contour", contour_first_img)
        # cv2.imshow("inverted_thresh",inverted_thresh)
        # cv2.imshow("vertical_inverted",vertical_inverted_img)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        
        
    def get_result(self):
        return self.min_thickness, self.max_thickness, self.mean_thickness, self.median_thickness