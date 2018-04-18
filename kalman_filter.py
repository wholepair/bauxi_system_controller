#!/usr/bin/python

import numpy as np
import matplotlib.pyplot as plt
import random

def kalman_xy(x, P, measurement, R
              , motion=np.matrix('0. 0. 0. 0.').T
              , Q=np.matrix(np.eye(4))):
    """
    Parameters:    
    x: initial state 4-tuple of location and velocity: (x0, x1, x0_dot, x1_dot)
    P: initial uncertainty convariance matrix
    measurement: observed position
    R: measurement noise 
    motion: external motion added to state vector x
    Q: motion noise (same shape as P)
    """
    return kalman(x, P, measurement, R, motion, Q
                  , F=np.matrix('''
                      1. 0. 1. 0.;
                      0. 1. 0. 1.;
                      0. 0. 1. 0.;
                      0. 0. 0. 1.
                      ''')
                  , H=np.matrix('''
                      1. 0. 0. 0.;
                      0. 1. 0. 0.'''))


def kalman(x, P, measurement, R, motion, Q, F, H):
    """
    Parameters:
    x: initial state
    P: initial uncertainty convariance matrix
    measurement: observed position (same shape as H*x)
    R: measurement noise (same shape as H)
    motion: external motion added to state vector x
    Q: motion noise (same shape as P)
    F: next state function: x_prime = F*x
    H: measurement function: position = H*x

    Return: the updated and predicted new values for (x, P)

    See also http://en.wikipedia.org/wiki/Kalman_filter

    This version of kalman can be applied to many different situations by
    appropriately defining F and H 
    """
    # UPDATE x, P based on measurement m    
    # distance between measured and current position-belief
    y = np.matrix(measurement).T - H * x
    S = H * P * H.T + R  # residual convariance
    K = P * H.T * S.I    # Kalman gain
    x = x + K * y
    I = np.matrix(np.eye(F.shape[0])) # identity matrix
    P = (I - K * H) * P

    # PREDICT x, P based on motion
    x = F * x + motion
    P = F * P * F.T + Q

    return x, P


class KalmanFilterXY(object):
    
    def __init__(self, x, y):
        matrix_text = str(x) + ' ' + str(y) + ' 0. 0.'
        self.__xy = np.matrix(matrix_text).T 
        self.__P = np.matrix(np.eye(4)) * 1000 # initial uncertainty
        self.__R = 0.01**2
        return
    
    
    def reset_initial_state(self, x, y):
        matrix_text = str(x) + ' ' + str(y) + ' 0. 0.'
        self.__xy = np.matrix(matrix_text).T 
    
    
    def update_predict(self, x, y):
        self.__xy, self.__P = kalman_xy(self.__xy, self.__P, (x, y), self.__R)
        #print self.__xy
        return (self.__xy[:2]).tolist()


def demo_kalman_xy():
    x = np.matrix('0. 0. 0. 0.').T 
    P = np.matrix(np.eye(4)) * 1000 # initial uncertainty

    N = 20
    sys_rand = random.SystemRandom()
    true_x = np.linspace(0.0, 10.0, N)
    true_y = true_x**2
    
    rand_x = []
    rand_y = []
    
    for i in range(N):
        rand_x.append(sys_rand.random())
        rand_y.append(sys_rand.random())
    
    observed_x = true_x + 0.05 * np.array(rand_x) * true_x
    observed_y = true_y + 0.05 * np.array(rand_y) * true_y
    
    plt.plot(observed_x, observed_y, 'ro')
    result = []
    R = 0.01**2
    
    for meas in zip(observed_x, observed_y):
        print meas
        x, P = kalman_xy(x, P, meas, R)
        result.append((x[:2]).tolist())
    
    for i in result:
        print i
    
    kalman_x, kalman_y = zip(*result)
    plt.plot(kalman_x, kalman_y, 'go')
    plt.show()
    return


if __name__ == "__main__":
    demo_kalman_xy()


