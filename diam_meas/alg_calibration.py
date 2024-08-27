# Check if a folder is selected
# Create a new folder named "Calibration_Result" in the selected folder to save calibration results
# Modify glob path to use the user-selected folder
# Input parameters: number of columns and rows of the chessboard, and physical size of squares (millimeters)
# Convert input to integers
# Physical size of squares, kept as float
# Note: 'self' here refers to an instance of a PyQt5 window class
# Set the dialog title
# Set the dialog text
# Set the icon to an information icon, similar to Tkinter's showinfo
# Display the dialog and wait for the user to close it
import sys
import numpy as np
import cv2
import glob
import os
from PyQt5.QtCore import QObject
from PyQt5.QtWidgets import QMessageBox, QApplication

class calibration(QObject):
    def __init__(self):
        self.column_points = 0
        self.row_points = 0
        self.square_size_mm = 0
        self.pixel_to_micron_factor = 0
        
        self.cali_intermediate_results_list=[]
        
        
    def set_column_points(self, column_points):
        self.column_points = column_points
    
    def set_row_points(self, row_points):
        self.row_points = row_points
    
    def set_square_size_mm(self, square_size_mm):
        self.square_size_mm = square_size_mm
    
    def get_calib_intermediate_results(self):
        return self.cali_intermediate_results_list

    def run_cali(self,path):
        selected_folder = path
        # selected_folder = QFileDialog.getExistingDirectory(None, "Select folder with calibration images")

        # Check if a folder is selected
        if not selected_folder:
            print("No folder selected, exiting...")
            exit()

        # Create a new folder named "Calibration_Result" in the selected folder to save calibration results
        result_folder = os.path.join(selected_folder, "Calibration_Result")
        os.makedirs(result_folder, exist_ok=True)

        # Modify glob path to use the user-selected folder
        images = glob.glob(os.path.join(selected_folder, 'Cali*.jpg'))

        # Input parameters: number of columns and rows of the chessboard, and physical size of squares (millimeters)
        # self.column_points = int(input("Enter column points: "))  # Convert input to integers
        # self.row_points = int(input("Enter row points: "))  # Convert input to integers
        # self.square_size_mm = float(input("Enter checkerboard size in mm: "))  # Physical size of squares, kept as float
        if(self.column_points == 0 or self.row_points == 0 or self.square_size_mm == 0):
            msg_box = QMessageBox(self)  # Note: 'self' here refers to an instance of a PyQt5 window class
            msg_box.setWindowTitle("Warning")  # Set the dialog title
            msg_box.setText("Please set the column points, row points and square size first")  # Set the dialog text
            msg_box.setIcon(QMessageBox.Information)  # Set the icon to an information icon, similar to Tkinter's showinfo
            msg_box.exec_()  # Display the dialog and wait for the user to close it
            return
        

        # Termination criteria for the iterative algorithm
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # Prepare object points, like (0,0,0), (1,0,0), (2,0,0) ..., (7,4,0)
        objp = np.zeros((self.column_points * self.row_points, 3), np.float32)  # Change to 6x8 for a 7x9 square board
        objp[:, :2] = np.mgrid[0:self.row_points, 0:self.column_points].T.reshape(-1, 2)

        # Arrays to store object points and image points from all images
        objpoints = []  # 3d points in real world space
        imgpoints = []  # 2d points in image plane

        # Make sure the path is correct for your system
        # images = glob.glob('C:/Users/Lixiaoyu/Desktop/CV report/Calibration_0222/Cali*.jpg')

        for fname in images:
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, (self.row_points, self.column_points), None)  # Adjusted for 8x6 inner corners

            # If found, add object points, image points (after refining them)
            if ret == True:
                objpoints.append(objp)

                # Refine the corner positions
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                imgpoints.append(corners2)

                # Draw and display the corners
                cv2.drawChessboardCorners(img, (self.row_points, self.column_points), corners2, ret)
                self.cali_intermediate_results_list.append(img)
                # cv2.imshow('img', img)
                # cv2.waitKey(100)

        # cv2.destroyAllWindows()

        # Perform calibration only if there were any chessboards found
        if objpoints and imgpoints:
            ret, self.mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

            # Calculating the re-projection error
            mean_error = 0
            for i in range(len(objpoints)):
                imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], self.mtx, dist)
                error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
                mean_error += error

            total_error = mean_error / len(objpoints)
            print(f"Total re-projection error: {total_error}")

            # Print the camera calibration values
            print("Camera matrix:", self.mtx)
            print("Distortion coefficients:", dist)
            print("Rotation Vectors:", rvecs)
            print("Translation Vectors:", tvecs)

            # Print focal length and calibration target details
            fx = self.mtx[0, 0]
            fy = self.mtx[1, 1]
            cx = self.mtx[0, 2]
            cy = self.mtx[1, 2]
            print(f"Focal Length (fx, fy): {fx}, {fy}")
            print("Calibration Target Details:")
            print("- Chessboard Size: {} x {}".format(self.row_points, self.column_points))
            # print("- Square Size: 1 unit (assuming the squares are unit squares)")

            
            square_size_microns = self.square_size_mm * 1000  # Convert to microns

            # Calculate the average pixel size of chessboard squares
            # Assuming all images have similar dimensions and magnification
            if len(imgpoints) > 0:
                pixel_distances = []  # Create an empty list to store pixel distances between adjacent points

                # Iterate over pixel coordinates of each image
                # for corners2 in imgpoints:
                    # Iterate over each row in the chessboard
                for i in range(self.column_points - 1):
                # Iterate over each column
                    for j in range(self.row_points - 1):
                        # Get coordinates of the current point and the bottom right point
                        current_point = corners2[i * self.row_points + j][0]  # Extract pixel coordinates of the current point from each corner array
                        diagonal_point = corners2[(i + 1) * self.row_points + j + 1][0]  # Get pixel coordinates of the bottom right point
                        # Calculate the pixel distance between the current point and the bottom right point
                        distance = np.linalg.norm(diagonal_point - current_point)
                        print("i:", i, "j:", j, "current_point:", current_point, "diagonal_point:", diagonal_point, "distance:", distance)
                        # Add the distance to the list
                        pixel_distances.append(distance)#Should it be placed outside or inside the loop?
                        print("pixel_distances:",pixel_distances)
                
                # Calculate the average pixel distance
                avg_pixel_distance = np.mean(pixel_distances) 
                self.pixel_to_micron_factor = np.sqrt(2) * square_size_microns / avg_pixel_distance
                print(f"Pixel to Micron Conversion Factor: {self.pixel_to_micron_factor}")
                print("Average diagonal point pixel distance:", avg_pixel_distance)
            else:
                print("No image points found, cannot calculate conversion factor.")

        else:
            print("Chessboard corners were not found in any of the images.")


        # Assuming our conversion factor variable name is pixel_to_micron_factor
        np.save('pixel_to_micron_factor.npy', self.pixel_to_micron_factor)
        # After calibration
        #np.savez('calibration_data.npz', camera_matrix=mtx, dist_coeffs=dist)

# This will run the instantiation process only when it is the program's entry point
if __name__ == "__main__":
    app = QApplication(sys.argv)
    calib =calibration()
    # calib.run_cali()
    # calib.show()
    sys.exit(app.exec_())

