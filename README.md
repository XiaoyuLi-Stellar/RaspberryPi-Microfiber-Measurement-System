# Raspberry Pi Microfiber Measurement System

A Raspberry Pi-based real-time system for measuring and controlling microfiber diameter with high precision. Features include a Python-implemented calibration protocol and a user-friendly GUI, achieving measurement accuracy within the 100-1000 micron range.

## Table of Contents
- [Scripts Overview](#scripts-overview)
- [Python Environment Setup](#python-environment-setup)
- [Capturing Environment Setup](#capturing-environment-setup)
- [Calibration & Measurement](#calibration--measurement)
  - [Calibration Principle](#calibration-principle)
  - [Calibration Steps](#calibration-steps)
  - [Measurement](#measurement)
- [URL References](#url-references)

## Scripts Overview

1. `main_calib_snap.py` - Entry script for capturing calibration images
2. `main_measure.py` - Processes calibration images and performs real-time measurements
3. `alg_calibration.py` - Camera calibration algorithm (gets intrinsic/extrinsic parameters)
4. `alg_camera.py` - Initializes camera for Windows systems
5. `picamera.py` - Initializes camera for Raspberry Pi
6. `alg_measure.py` - Measuring and calculating algorithm
7. `Ui_calibrator.py` and `Ui_page1_calib.py` - UI design for capturing/measuring processes

## Python Environment Setup

1. Update your Raspberry Pi system:
   ```bash
   sudo apt update
   sudo apt upgrade
   ```

2. Install Python 3 and pip:
   ```bash
   sudo apt install python3 python3-pip
   ```

3. Install required Python libraries:
   ```bash
   pip install numpy pillow PyQt5 matplotlib opencv-python python3-picamera
   ```

## Capturing Environment Setup

Environmental requirements for non-contact precision measurement:

1. **Stable Light Source**: Lighting should be stable and uniform
2. **Background Setup**: Avoid cluttered backgrounds, ensure contrast between background and object
3. **Stable Camera Position**: Camera should remain fixed during measurement

## Calibration & Measurement

### Calibration Principle

Establishes relationship between camera parameters and real-world coordinates. Corrects distortions and determines pixel-to-physical-dimension conversion factors.

### Calibration Steps

1. **Prepare calibration board**:
   - Use a rigid checkerboard pattern (generate at [calib.io](https://calib.io/pages/camera-calibration-pattern-generator))
   - Board should occupy >80% of camera view

2. **Environment setup**:
   - Adjust lighting and background
   - Find optimal distance for clear object capture

3. **Capture calibration images**:
   - Run `main_calib_snap.py`
   - Capture 20-30 images from different angles
   - Ensure all corners are visible

4. **Calibrate**:
   - Run `main_measure.py`
   - Select calibration image folder
   - Input calibration board parameters:
     - Columns (inner points)
     - Rows (inner points)
     - Checkerboard size
   - Click "Calibrate"

### Measurement

After calibration:
1. Click "Start" to begin measurements
2. System will display max, min, and average values

## URL References

- Raspberry Pi Setup:
  - [WiFi Configuration](https://www.seeedstudio.com/blog/2021/01/25/three-methods-to-configureraspberry-pi-wifi/)
  - [SSH Enable](https://phoenixnap.com/kb/enable-ssh-raspberry-pi#ftoc-heading-9)
  - [VNC Server Setup](https://www.youtube.com/watch?v=Zoxus1QoEZo)

- Camera Documentation:
  - [Raspberry Pi High Quality Camera](https://www.raspberrypi.com/documentation/accessories/camera.html)
  - [Lens Mount Drawing](https://www.raspberrypi.com/documentation/accessories/camera.html)

- Calibration Resources:
  - [Pattern Generator](https://calib.io/pages/camera-calibration-pattern-generator)
  - [Calibration Tutorial](https://youtube.com/calibration-method-tutorial) (starts at 6:40)
