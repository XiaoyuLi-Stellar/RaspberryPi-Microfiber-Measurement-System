import glob
import os

import cv2
import numpy as np
from PyQt5 import QtWidgets
from PyQt5.QtCore import pyqtSlot, QDir, QTimer, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QMessageBox, QFileDialog
from Ui_calibrator import Ui_MainWindow
from alg_calibration import calibration
from alg_camera import Camera
from alg_measure import measurement

class myCalibrationWindow(QtWidgets.QMainWindow,Ui_MainWindow):
    update_image_signal = pyqtSignal(int)
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        # self.setFixedSize(1294,837)
        self.resize(1057,842)
        self.disabledWidget()
        #
        self.cali = calibration()
        #
        self.snap_saved_folder = ''
        #
        self.cam = Camera()
        #
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_display)
        self.timer.start(30)
        #
        self.cali_imm = None
        self.cali_imm_single = None
        #
        self.timer_cal = QTimer(self)
        self.timer_cal.timeout.connect(self.update_cali_window)
        #
        self.index = 0
        #
        self.timer_live = QTimer(self)
        self.timer_live.setInterval(30)
        self.timer_live.timeout.connect(self.live_cam_measure)


    def disabledWidget(self):
        self.spinBox_cbSize.setEnabled(False)
        self.spinBox_cbColum.setEnabled(False)
        self.spinBox_cbRow.setEnabled(False)
        self.lineEdit_folder.setEnabled(False)
        self.btnCalibrate.setEnabled(False)
        self.btnStart.setEnabled(False)
        self.btnStop.setEnabled(False)

    def enableWidget(self):
        self.spinBox_cbSize.setEnabled(True)
        self.spinBox_cbColum.setEnabled(True)
        self.spinBox_cbRow.setEnabled(True)
        self.lineEdit_folder.setEnabled(True)

    def update_display(self):
        live_cam_img = self.cam.get_camera_img()
        if isinstance(live_cam_img,np.ndarray):
            live_cam_img = cv2.cvtColor(live_cam_img, cv2.COLOR_BGR2RGB)
            self.frame_np = live_cam_img.copy()
            h = self.label_5.height()
            w = self.label_5.width()
            self.live_cam_img = cv2.resize(live_cam_img, (h, w))
            h, w, ch = live_cam_img.shape
            bytesPerLine = ch * w
            convertToQtFormat = QImage(live_cam_img.data, w, h, bytesPerLine, QImage.Format_RGB888).rgbSwapped()

            self.label_5.setPixmap(QPixmap.fromImage(convertToQtFormat))

            # self.measure.start_measurement(live_cam_img)
            # self.measure_results = self.measure.get_result()
            # print(self.measure_results)

        elif (live_cam_img == -1):
            print("No frame from camera.")
        else:
            print("Camera stopped.")

    @pyqtSlot()
    def on_btnStart_clicked(self):
        self.measure = measurement()
        self.measure_results = None
        self.timer_live.start()
        self.btnStart.setEnabled(False)
        self.btnStop.setEnabled(True)

    @pyqtSlot()
    def on_btnStop_clicked(self):
        self.measure = None
        self.measure_results = None
        self.timer_live.stop()
        self.btnStart.setEnabled(True)
        self.btnStop.setEnabled(False)

    def live_cam_measure(self):
        self.measure.start_measurement(self.live_cam_img)
        self.measure_results = self.measure.get_result()
        print(self.measure_results)

        self.get_Value(self.measure.max,self.measure.min,self.measure.mean)

    def get_Value(self,max,min,mean):
        self.label_6.setText(f"Max:{max}")
        self.label_8.setText(f"Min:{min}")
        self.label_9.setText(f"Mean:{mean}")

    def closeEvent(self, event):
        self.cam.cap.release()
        super().closeEvent(event)

    def wait_cali_finish(self):
        result_from_cali = self.cali.get_calib_intermediate_results()
        if len(result_from_cali) > 0:
            self.cali_imm = result_from_cali
            # self.root.after(1, self.update_cali_window)
            self.timer_cal.start(1000)
        else:
            # self.root.after(1, self.wait_cali_finish)
            self.timer_cal.stop()
            self.btnStart.setEnabled(True)

    def update_cali_window(self):
        if len(self.cali_imm) > self.index:  # Check if there are more images to display
            img = self.cali_imm[self.index]
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

            h = self.label_7.height()
            w = self.label_7.width()
            img = cv2.resize(img, (h, w))
            ''''''
            # OpenCV uses numpy array, so we need to convert it to QImage
            height, width, bytesPerComponent = img.shape
            bytesPerLine = bytesPerComponent * width
            qtImg = QImage(img.data, width, height, bytesPerLine, QImage.Format_RGB888)
            qtImg = qtImg.rgbSwapped()  # OpenCV uses BGR, Qt uses RGB, so we may need to swap color channels
            # Convert QImage to QPixmap for display in QLabel
            pixmap = QPixmap.fromImage(qtImg)
            self.label_7.setPixmap(pixmap)
            # Use QTimer to set a 1-second delay to call show_imm to display the next image
            self.index += 1
            # self.root.after(1000, lambda: self.update_cali_window(index + 1))
        else:
            # If there are no more images to display, some finishing processing may be needed
            print("All images have been displayed.")
            self.timer_cal.stop()
            self.btnStart.setEnabled(True)
            self.btnCalibrate.setEnabled(True)

    @pyqtSlot()
    def on_btnBrowse_clicked(self):
        root_dir = './'  # Replace with your root directory path
        # Use QFileDialog to get the folder path
        self.folder_path = QFileDialog.getExistingDirectory(self, 'Select Folder', root_dir)
        # If a folder is selected (i.e., path is not empty)
        if self.folder_path:
            # Display the path in LineEdit
            self.lineEdit_folder.setText(self.folder_path)
            # Calculate the number of files in the folder
            file_count = self.count_files_in_dir(self.folder_path)
            # Display the file count in the status bar
            self.statusBar().showMessage(f'There are {file_count} files in the folder')
            #
            # Create a new folder named "Calibration_Result" in the selected folder to save calibration results
            result_folder = os.path.join(self.folder_path, "Calibration_Result")
            os.makedirs(result_folder, exist_ok=True)
            # Modify glob path to use the user-selected folder
            self.images = glob.glob(os.path.join(self.folder_path, 'Cali*.jpg'))
            self.btnCalibrate.setEnabled(True)
            self.enableWidget()
        else:
            QMessageBox(QMessageBox.Warning, "Warning", "Please choose the correct directory")
        # self.enableWidget()

    def count_files_in_dir(self, folder_path):
        # Traverse the folder using QDir and calculate the number of files
        dir = QDir(folder_path)
        file_count = 0
        for entry in dir.entryInfoList(['*'], QDir.Files | QDir.Dirs | QDir.NoDotAndDotDot):
            if entry.isFile():
                file_count += 1
            # If recursive calculation of file count in subfolders is needed, corresponding logic can be added
        return file_count


    @pyqtSlot()
    def on_btnCalibrate_clicked(self):
        self.btnCalibrate.setEnabled(False)
        self.column_points = int(self.spinBox_cbColum.value())
        self.row_points = int(self.spinBox_cbRow.value())
        self.square_size_mm = float(self.spinBox_cbSize.value())

        self.cali.set_column_points(self.column_points)
        self.cali.set_row_points(self.row_points)
        self.cali.set_square_size_mm(self.square_size_mm)
        self.cali.run_cali(self.folder_path)

        self.cali_intermediate_results_list = []
        self.pixel_to_micron_factor = 0
        # Termination criteria for the iterative algorithm
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        # Prepare object points, like (0,0,0), (1,0,0), (2,0,0) ..., (7,4,0)
        objp = np.zeros((self.column_points * self.row_points, 3), np.float32)  # Change to 6x8 for a 7x9 square board
        objp[:, :2] = np.mgrid[0:self.row_points, 0:self.column_points].T.reshape(-1, 2)
        # Arrays to store object points and image points from all images
        objpoints = []  # 3d points in real world space
        imgpoints = []  # 2d points in image plane

        for fname in self.images:
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, (self.row_points, self.column_points),None)  # Adjusted for 8x6 inner corners
            # If found, add object points, image points (after refining them)
            if ret == True:
                objpoints.append(objp)
                # Refine the corner positions
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                imgpoints.append(corners2)
                # Draw and display the corners
                cv2.drawChessboardCorners(img, (self.row_points, self.column_points), corners2, ret)
                self.cali_intermediate_results_list.append(img)
                '''可正确输出self.cali_intermediate_results_list'''
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
            print("Camera matrix:", self.mtx)
            print("Distortion coefficients:", dist)
            print("Rotation Vectors:", rvecs)
            print("Translation Vectors:", tvecs)

            # Print focal length and calibration target details
            fx = self.mtx[0, 0]
            fy = self.mtx[1, 1]
            cx = self.mtx[0, 2]
            cy = self.mtx[1, 2]

            self.plainTextEdit.appendPlainText(f"Focal Length (fx, fy): {fx}, {fy}\n")
            self.plainTextEdit.appendPlainText("Calibration Target Details:\n")
            self.plainTextEdit.appendPlainText("- Chessboard Size: {} x {}\n".format(self.row_points, self.column_points))
            # print(f"Focal Length (fx, fy): {fx}, {fy}")
            # print("Calibration Target Details:")
            # print("- Chessboard Size: {} x {}".format(self.row_points, self.column_points))
            # print("- Square Size: 1 unit (assuming the squares are unit squares)")
            square_size_microns = self.square_size_mm * 1000  
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
                        print("i:", i, "j:", j, "current_point:", current_point, "diagonal_point:", diagonal_point,
                            "distance:", distance)
                        # Add the distance to the list
                        pixel_distances.append(distance)  # Should it be placed outside or inside the loop?
                        print("pixel_distances:", pixel_distances)

                # Calculate the average pixel distance

                avg_pixel_distance = np.mean(pixel_distances)
                self.pixel_to_micron_factor = np.sqrt(2) * square_size_microns / avg_pixel_distance
                print(f"Pixel to Micron Conversion Factor: {self.pixel_to_micron_factor}")
                print("Average diagonal point pixel distance:", avg_pixel_distance)

                self.plainTextEdit.appendPlainText(f"Pixel to Micron Conversion Factor: {self.pixel_to_micron_factor}")
                self.plainTextEdit.appendPlainText(f"Average diagonal point pixel distance: {avg_pixel_distance}")

            else:
                print("No image points found, cannot calculate conversion factor.")

        else:
            print("Chessboard corners were not found in any of the images.")

        np.save('pixel_to_micron_factor.npy', self.pixel_to_micron_factor)




        self.wait_cali_finish()




if __name__=="__main__":
    import sys
    app=QtWidgets.QApplication(sys.argv)
    test=myCalibrationWindow()
    test.show()
    sys.exit(app.exec_())