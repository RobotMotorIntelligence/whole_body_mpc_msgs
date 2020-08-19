import numpy as np
from riccati_controller.msg import RiccatiGain
import copy

class RiccatiGainInterface():
    def __init__(self, nx, nu):
        self.msg = RiccatiGain()
        self.msg.header.frame_id = "world"
        self.msg.nx = nx
        self.msg.nu = nu
        self.msg.data = [None] * (nx * nu)
        self.K = np.zeros([nu, nx])
    
    def writeToMessage(self, K):
        if K.shape[0] is not self.msg.nu:
            print("Couldn't convert the Riccati gain into a message since nu is not consistent")
            return
        if K.shape[1] is not self.msg.nx:
            print("Couldn't convert the Riccati gain into a message since nx is not consistent")
            return
        for i in range(self.msg.nu):
            for j in range(self.msg.nx):
                self.msg.data[i * self.msg.nx + j] = K[i, j]
        return copy.deepcopy(self.msg)

    def writeFromMessage(self, msg):
        if msg.nu is not self.K.shape[0]:
            print("Couldn't convert the message into a Riccati gain into since nu is not consistent")
            return
        if msg.nx is not self.K.shape[1]:
            print("Couldn't convert the message into a Riccati gain since nx is not consistent")
            return
        for i in range(msg.nu):
            for j in range(msg.nx):
                self.K[i, j] = msg.data[i * msg.nx + j]
        return copy.deepcopy(self.K)