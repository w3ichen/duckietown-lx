from typing import Tuple

import numpy as np

# Apply offset so if duckiebot is in middle, it will go to one side
OFFSET = 100

def get_motor_left_matrix(shape: Tuple[int, int]) -> np.ndarray:
    # TODO: write your function instead of this one
    # res = np.zeros(shape=shape, dtype="float32")
    # # these are random values
    # res[100:150, 100:150] = 1
    # res[300:, 200:] = 1
    # ---
    # Set left half to 1 and everything else as 0
    res = np.zeros(shape)
    res[:, int(shape[1]/2)-OFFSET:] = 1
    return res


def get_motor_right_matrix(shape: Tuple[int, int]) -> np.ndarray:
    # # TODO: write your function instead of this one
    # res = np.zeros(shape=shape, dtype="float32")
    # # these are random values
    # res[100:150, 100:300] = -1
    # ---
    # Set right half to 1 and everything else as 0
    res = np.zeros(shape)
    res[:, :int(shape[1]/2)+OFFSET] = 1

    return res
