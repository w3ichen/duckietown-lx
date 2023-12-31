from typing import Tuple

import numpy as np
import cv2


def get_steer_matrix_left_lane_markings(shape: Tuple[int, int]) -> np.ndarray:
    """
    Args:
        shape:              The shape of the steer matrix.

    Return:
        steer_matrix_left:  The steering (angular rate) matrix for Braitenberg-like control
                            using the masked left lane markings (numpy.ndarray)
    """

    # TODO: implement your own solution here
    # steer_matrix_left = np.random.rand(*shape)
    # ---
    steer_matrix_left = np.zeros(shape)
    steer_matrix_left[:, :shape[1]//2] = 1
    return steer_matrix_left


def get_steer_matrix_right_lane_markings(shape: Tuple[int, int]) -> np.ndarray:
    """
    Args:
        shape:               The shape of the steer matrix.

    Return:
        steer_matrix_right:  The steering (angular rate) matrix for Braitenberg-like control
                             using the masked right lane markings (numpy.ndarray)
    """

    # TODO: implement your own solution here
    # steer_matrix_right = np.random.rand(*shape)
    # ---
    steer_matrix_right = np.zeros(shape)
    steer_matrix_right[:, shape[1]//2:] = 1
    return steer_matrix_right

def detect_lane_markings(image: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Args:
        image: An image from the robot's camera in the BGR color space (numpy.ndarray)
    Return:
        mask_left_edge:   Masked image for the dashed-yellow line (numpy.ndarray)
        mask_right_edge:  Masked image for the solid-white line (numpy.ndarray)
    """
    h, w, _ = image.shape

    # TODO: implement your own solution here
    # mask_left_edge = np.random.rand(h, w)
    # mask_right_edge = np.random.rand(h, w)
    # ---
    white_lower_hsv = np.array([0, 0, 135])     
    white_upper_hsv = np.array([125, 42, 255])  
    yellow_lower_hsv = np.array([22, 114, 165]) 
    yellow_upper_hsv = np.array([29, 255, 236]) 

    imghsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) # BGR to HSV

    mask_white = cv2.inRange(imghsv, white_lower_hsv, white_upper_hsv)
    mask_yellow = cv2.inRange(imghsv, yellow_lower_hsv, yellow_upper_hsv)

    mask_left_edge = mask_yellow
    mask_right_edge = mask_white

    return mask_left_edge, mask_right_edge
