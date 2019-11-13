import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation

from physics_cube import Mass, Spring

# Fixing random state for reproducibility
np.random.seed(19680801)

GRAVITY = 9.81
damping = 0.9999
DT = 0.0008 # timestep
friction_mu_s = 1
friction_mu_k = 0.8
k_vertices_soft = 2000
k_ground = 200000
omega = 10 
k = 5000

#initialize cube
masses = [None] * 8
springs = []
# self, mass, position, velocity, accel, external
# length, k, masses
masses[0] = Mass(0.1, np.array([0, 0, 0]))
masses[1] = Mass(0.1, np.array([0.1, 0, 0]))
masses[2] = Mass(0.1, np.array([0.1, 0.1, 0]))
masses[3] = Mass(0.1, np.array([0, 0.1, 0]))
masses[4] = Mass(0.1, np.array([0, 0, 0.1]))
masses[5] = Mass(0.1, np.array([0.1, 0, 0.1]))
masses[6] = Mass(0.1, np.array([0.1, 0.1, 0.1]))
masses[7] = Mass(0.1, np.array([0, 0.1, 0.1]))


for i, mass in enumerate(masses):
    j = i
    while j < len(masses):
        length = np.linalg.norm(mass.position - masses[j].position)
        if length > 0:
            spring = Spring(k, length, [mass, masses[j]])
            springs.append(spring)
        j +=1
masses[7].update_pos([0,0,0.15])

def Gen_RandLine(length, dims=2):
    """
    Create a line using a random walk algorithm

    length is the number of points for the line.
    dims is the number of dimensions the line has.
    """
    lineData = np.empty((dims, length))
    lineData[:, 0] = np.random.rand(dims)
    for index in range(1, length):
        # scaling the random numbers by 0.1 so
        # movement is small compared to position.
        # subtraction by 0.5 is to change the range to [-0.5, 0.5]
        # to allow a line to move backwards.
        step = ((np.random.rand(dims) - 0.5) * 0.1)
        lineData[:, index] = lineData[:, index - 1] + step

    return lineData


def update_lines(num, dataLines, lines):
    for line, data in zip(lines, dataLines):
        # NOTE: there is no .set_data() for 3 dim data...
        line.set_data(data[0:2, :num])
        line.set_3d_properties(data[2, :num])
    return lines

def simuloop(k, l, l_0):
    return None

# Attaching 3D axis to the figure
fig = plt.figure()
ax = p3.Axes3D(fig)

# Fifty lines of random 3-D lines
#data = [Gen_RandLine(25, 3) for index in range(50)]
data = [None] * len(masses)
spring_data = []
for i, mass in enumerate(masses):
    data[i] = mass.position
data = np.array(data)
ax.scatter(data[:,0], data[:,1], data[:,2], 'bo')

for i, spring in enumerate(springs):
    spring_data.append(spring.masses[0].position)
    spring_data.append(spring.masses[1].position)
spring_data = np.array(spring_data)
print("spring_Dat", spring_data)
ax.plot(spring_data[:,0], spring_data[:,1], spring_data[:,2], 'rx-')

# NOTE: Can't pass empty arrays into 3d version of plot()
#lines = [ax.plot(dat[0, 0:1], dat[1, 0:1], dat[2, 0:1])[0] for dat in data]

# Setting the axes properties
ax.set_xlim3d([0.0, 0.2])
ax.set_xlabel('X')

ax.set_ylim3d([0.0, 0.2])
ax.set_ylabel('Y')

ax.set_zlim3d([0.0, 0.2])
ax.set_zlabel('Z')

ax.set_title('3D Test')

# Creating the Animation object
line_ani = animation.FuncAnimation(fig, simuloop, 25, fargs=(1, 0.1, 0.1),
                                   interval=50, blit=False)

plt.show()

