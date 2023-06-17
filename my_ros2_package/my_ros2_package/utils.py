import cv2 as cv
import numpy as np

# Undistort Function
'''Function that undistorts images based on the camera info and distortion coefficients'''
def undistort(latest_img, camera_info, dist_coeffs):
    # https://github.com/Chrislai502/Lidar_Camera_Calibration/blob/main/pcd_image_overlay_Chris.py

    cam_image = latest_img
    camera_info = camera_info

    img_size = (cam_image.shape[1], cam_image.shape[0])
    dist_coeffs = dist_coeffs  
    new_K, _ = cv.getOptimalNewCameraMatrix(camera_info, dist_coeffs, img_size, alpha=1)
    image_undistorted = cv.undistort(cam_image, camera_info, dist_coeffs, None, new_K)
    return image_undistorted