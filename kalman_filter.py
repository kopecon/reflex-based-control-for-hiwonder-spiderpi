import numpy as np
from filterpy.kalman import KalmanFilter
from weakref import ref


class Kalman:
    def __init__(self, parent, init_value=10000):
        self.parent = ref(parent)
        self.f = KalmanFilter(dim_x=1, dim_z=1)
        self.f.x = np.array([float(init_value)])
        self.f.F = np.array([[1.]])
        self.f.H = np.array([[1.]])
        self.f.P *= 1000.
        self.f.R = self.parent().kalman_R

    def filter_out_noise(self, data):
        z = data
        self.f.predict()
        self.f.update(z)
        output = self.f.x.tolist()[0]
        return output
