import glob
import os
import cv2
import numpy as np
from PyQt5 import QtWidgets
from PyQt5.QtCore import pyqtSlot, QDir, QTimer
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QWidget, QLabel, QMessageBox, QFileDialog
from Ui_page1_calib import Ui_MainWindow
from alg_calibration import calibration
from alg_camera import Camera
from alg_measure import measurement
from main_measure import myCalibrationWindow

class myWindow(QtWidgets.QMainWindow,Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.pbt_stop.hide()
        # self.resize(900,710)
        self.setFixedSize(900,710)
        self.disabledWidget()
        #
        self.cali = calibration()
        #
        self.snap_saved_folder = ''
        #
        self.cam = Camera()
        # self.measure = measurement()
        self.measure_results = None
        #
        self.photo_num = 0
        self.cali_imgs = []
        self.frame_np = None
        #
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_display)
        self.timer.start(30)


    def disabledWidget(self):
        self.spinBox_num.setEnabled(False)
        self.pbt_shot.setEnabled(False)
        self.pbt_stop.setEnabled(False)

    def enableWidget(self):
        self.pbt_shot.setEnabled(True)

    @pyqtSlot()
    def on_pbt_browse_clicked(self):
        root_dir = './'  # Replace with your root directory path
        self.save_folder_path = QFileDialog.getExistingDirectory(self, '选择文件夹', root_dir)
        if self.save_folder_path:
            self.lineEdit_folder.setText(self.save_folder_path)
            self.enableWidget()
        else:
            print("wrong path")

    @pyqtSlot()
    def on_pbt_shot_clicked(self):
        self.path_text = self.lineEdit_folder.text()
        if self.path_text == '':
            info_box = QMessageBox(QMessageBox.Information, "Information", "Please first choose the folder to start")
            info_box.setIcon(QMessageBox.Information)
            info_box.exec_()
        else:
            self.set_photo_number()
            # take a snap
            self.save_snapshot()
            self.pbt_stop.setEnabled(True)

    # @pyqtSlot()
    # def on_pbt_stop_clicked(self):
    #     self.timer.stop()

    def save_snapshot(self):
        self.cali_imgs.append(self.frame_np)
        print(f"Frame {self.photo_num} saved.")
        capture_img = self.frame_np
        cv2.imwrite(f"{self.save_folder_path}/Cali{self.photo_num:02}.jpg", capture_img)

    def set_photo_number(self):
        self.photo_num += 1
        self.spinBox_num.setValue(self.photo_num)

    def update_display(self):
        live_cam_img = self.cam.get_camera_img()
        if isinstance(live_cam_img,np.ndarray):
            live_cam_img = cv2.cvtColor(live_cam_img, cv2.COLOR_BGR2RGB)
            self.frame_np = live_cam_img.copy()
            h = self.label_image.height()
            w = self.label_image.width()
            live_cam_img = cv2.resize(live_cam_img, (h, w))
            h, w, ch = live_cam_img.shape
            bytesPerLine = ch * w
            convertToQtFormat = QImage(live_cam_img.data, w, h, bytesPerLine, QImage.Format_RGB888).rgbSwapped()
            self.label_image.setPixmap(QPixmap.fromImage(convertToQtFormat))
            # self.measure.start_measurement(live_cam_img)
            # self.measure_results = self.measure.get_result()
            # print(self.measure_results)

        elif (live_cam_img == -1):
            print("No frame from camera.")
        else:
            print("Camera stopped.")

    def closeEvent(self, event):
        self.cam.cap.release()
        super().closeEvent(event)

    @pyqtSlot()
    def on_btnBrowse_clicked(self):
        root_dir = './'
        # use QFileDialog to get the folder path
        folder_path = QFileDialog.getExistingDirectory(self, '选择文件夹', root_dir)

        if folder_path:
            # display the path in LineEdit
            self.lineEdit_folder.setText(folder_path)
            # calculate the number of files in the folder
            file_count = self.count_files_in_dir(folder_path)
            # display the file count in the status bar
            self.statusBar().showMessage(f'文件夹中有 {file_count} 个文件')
            result_folder = os.path.join(folder_path, "Calibration_Result")
            os.makedirs(result_folder, exist_ok=True)
            # modify the glob path to use the user-selected folder
            self.images = glob.glob(os.path.join(folder_path, 'Cali*.jpg'))
        else:
            QMessageBox(QMessageBox.Warning, "Warning", "Please choose correct dic")
        self.enableWidget()


if __name__=="__main__":
    import sys
    app=QtWidgets.QApplication(sys.argv)
    test=myWindow()
    test.show()


    sys.exit(app.exec_())