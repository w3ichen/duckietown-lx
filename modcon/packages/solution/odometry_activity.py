from typing import Tuple

import numpy as np


def delta_phi(ticks: int, prev_ticks: int, resolution: int) -> Tuple[float, float]:
    """
    Args:
        ticks: Current tick count from the encoders.
        prev_ticks: Previous tick count from the encoders.
        resolution: Number of ticks per full wheel rotation returned by the encoder.
    Return:
        dphi: Rotation of the wheel in radians.
        ticks: current number of ticks.
    """

    # TODO: these are random values, you have to implement your own solution in here
    # ticks = prev_ticks + int(np.random.uniform(0, 10))
    # dphi = np.random.random()
    # ---
    alpha = 2 * np.pi / resolution # wheel rotation per tick in radians
    dphi = ticks*alpha # delta ticks of right wheel 
    ticks = prev_ticks+dphi # total rotation of left wheel 

    return dphi, ticks


def pose_estimation(
    R: float,
    baseline: float,
    x_prev: float,
    y_prev: float,
    theta_prev: float,
    delta_phi_left: float,
    delta_phi_right: float,
) -> Tuple[float, float, float]:

    """
    Calculate the current Duckiebot pose using the dead-reckoning model.

    Args:
        R:                  radius of wheel (both wheels are assumed to have the same size) - this is fixed in simulation,
                            and will be imported from your saved calibration for the real robot
        baseline:           distance from wheel to wheel; 2L of the theory
        x_prev:             previous x estimate - assume given
        y_prev:             previous y estimate - assume given
        theta_prev:         previous orientation estimate - assume given
        delta_phi_left:     left wheel rotation (rad)
        delta_phi_right:    right wheel rotation (rad)

    Return:
        x_curr:                  estimated x coordinate
        y_curr:                  estimated y coordinate
        theta_curr:              estimated heading
    """

    # These are random values, replace with your own
    # x_curr = np.random.random()
    # y_curr = np.random.random()
    # theta_curr = np.random.random()
    # ---
    d_left = R*delta_phi_left
    d_right = R*delta_phi_right
    d_A = (d_left+d_right)/2 

    delta_theta = (d_right-d_left)/baseline
    theta_curr = theta_prev + delta_theta

    delta_x = d_A * np.cos(theta_curr)
    delta_y = d_A * np.sin(theta_curr)
    x_curr = x_prev + delta_x
    y_curr = y_prev + delta_y
    return x_curr, y_curr, theta_curr
