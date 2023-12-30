from typing import Tuple

import numpy as np


def PIDController(
        v_0: float,
        theta_ref: float,
        theta_hat: float,
        prev_e: float,
        prev_int: float,
        delta_t: float
) -> Tuple[float, float, float, float]:
    """
    PID performing heading control.
    Args:
        v_0:        linear Duckiebot speed (given).
        theta_ref:  reference heading pose.
        theta_hat:  the current estiamted theta.
        prev_e:     tracking error at previous iteration.
        prev_int:   previous integral error term.
        delta_t:    time interval since last call.
    Returns:
        v_0:     linear velocity of the Duckiebot
        omega:   angular velocity of the Duckiebot
        e:       current tracking error (automatically becomes prev_e at next iteration).
        e_int:   current integral error (automatically becomes prev_int at next iteration).
    """

    # TODO: these are random values, you have to implement your own PID controller in here
    # omega = np.random.uniform(-8.0, 8.0)
    # e = np.random.random()
    # e_int = np.random.random()
    # Hint: print for debugging
    # print(f"\n\nDelta time : {delta_t} \nE : {np.rad2deg(e)} \nE int : {e_int} \nPrev e : {prev_e} \nU : {u} \nTheta hat: {np.rad2deg(theta_hat)} \n")
    # ---
    Kp = 5
    Ki = 0.2
    Kd = 0.1

    e = theta_ref - theta_hat # Error of current theta from the ref

    e_int = prev_int + e * delta_t # integral error at the previous instant plus the new increment
    # anti-windup - preventing the integral error from growing too much
    e_int = max(min(e_int,2),-2)
    
    e_der = (prev_e - e)/delta_t

    omega = Kp*e + Ki*e_int + Kd*e_der
    return v_0, omega, e, e_int
