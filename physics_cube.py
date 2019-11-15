import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
import matplotlib.patches as patches
import numpy as np

class Mass():
    def __init__(self, mass, position = 0, velocity=0, accel=0, external=0):

        self.mass = mass
        self.position = position
        self.velocity = velocity
        self.accel = accel
        self.external = external

    def update_pos(self, position):
        self.position = position
        return self.position

    def update(self, position, velocity, accel, external):
        self.position = position
        self.velocity = velocity
        self.accel = accel
        self.external = external

    def __repr__(self):
        return 'Mass({})'.format(', '.join(map(str, self.position)))

class Spring():
    def __init__(self, k, length, masses):
        self.original_length = length
        self.length = length
        self.k = k
        self.masses = masses

    # returns a LIST of masses [mass, mass]
    def getMasses(self):
        return np.ndarray.tolist(self.masses[0].position), np.ndarray.tolist(self.masses[1].position)

    def getMassOne(self):
        return self.masses[0]

    def getMassTwo(self):
        return self.masses[1]

    # returns float of current spring length
    def getLength(self):
        self.length = np.linalg.norm(self.masses[1].position - self.masses[0].position)
        return self.length

    def getL0(self):
        return self.original_length