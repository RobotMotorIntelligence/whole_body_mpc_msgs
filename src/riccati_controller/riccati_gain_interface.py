import numpy as np
from riccati_controller.msg import RiccatiGain
import copy


class RiccatiGainInterface():
    def __init__(self, nx, nu):
        self._msg = RiccatiGain()
        self._msg.header.frame_id = "world"
        self._msg.nx = nx
        self._msg.nu = nu
        self._msg.data = [None] * (nx * nu)
        self._K = np.zeros([nu, nx])

    def writeToMessage(self, K):
        if K.shape[0] is not self._msg.nu:
            print("Couldn't convert the Riccati gain into a message since nu is not consistent")
            return
        if K.shape[1] is not self._msg.nx:
            print("Couldn't convert the Riccati gain into a message since nx is not consistent")
            return
        for i in range(self._msg.nu):
            for j in range(self._msg.nx):
                self._msg.data[i * self._msg.nx + j] = K[i, j]
        return copy.deepcopy(self._msg)

    def writeFromMessage(self, msg):
        if msg.nu is not self._K.shape[0]:
            print("Couldn't convert the message into a Riccati gain into since nu is not consistent")
            return
        if msg.nx is not self._K.shape[1]:
            print("Couldn't convert the message into a Riccati gain since nx is not consistent")
            return
        for i in range(msg.nu):
            for j in range(msg.nx):
                self._K[i, j] = msg.data[i * msg.nx + j]
        return copy.deepcopy(self._K)
