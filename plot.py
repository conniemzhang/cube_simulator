import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
from physics_cube import Mass, Spring

# Fixing random state for reproducibility
np.random.seed(19680801)

GRAVITY = np.array([0, 0, -9.81])
TIMESTEP = 0.01 #why is the timestep so small tho. The animation refreshes every 50ms
damping = 0.9999
friction_mu_s = 1
friction_mu_k = 0.8
k_vertexplot_soft = 2000
k_ground = 100000
omega = 10 
k = 5000


def gen_square(edge, origin):
    """
    Create a square with lengths length
    lower corner is the "origin" of square
    """
    mass = 0.1
    masses = [None] * 8
    springs = []
    masses[0] = Mass(mass, np.array(origin))
    masses[1] = Mass(mass, np.array([origin[0] + edge, origin[1], origin[2]]))
    masses[2] = Mass(mass, np.array([origin[0] + edge, origin[1] + edge, origin[2]]))
    masses[3] = Mass(mass, np.array([origin[0], origin[1] + edge, origin[2]]))
    masses[4] = Mass(mass, np.array([origin[0], origin[1], origin[2]+edge]))
    masses[5] = Mass(mass, np.array([origin[0] + edge, origin[1], origin[2]+edge]))
    masses[6] = Mass(mass, np.array([origin[0]+edge, origin[1]+edge, origin[2] + edge]))
    masses[7] = Mass(mass, np.array([origin[0], origin[1]+edge, origin[2]+edge]))

    #make springs
    for i, mass in enumerate(masses):
        j = i
        while j < len(masses):
            length = np.linalg.norm(mass.position - masses[j].position)
            if length > 0:
                spring = Spring(k, length, [mass, masses[j]])
                springs.append(spring)
            j +=1

    return masses, springs


def update_vertexplot(num, masses, vertexplot):
    calculate_forces()
    data = [[mass.position[0], mass.position[1], mass.position[2]] for mass in masses]
    x = [row[0] for row in data]
    y = [row[1] for row in data]
    z = [row[2] for row in data]
    vertexplot[0].set_data([x,y])
    vertexplot[0].set_3d_properties(z)

    sdata = []
    for spring in springs:
        one, two = spring.getMasses()
        sdata.append(one)
        sdata.append(two)
    
    x = [row[0] for row in sdata]
    y = [row[1] for row in sdata]
    z = [row[2] for row in sdata]
    springplot[0].set_data([x,y])
    springplot[0].set_3d_properties(z)

    return vertexplot

def calculate_forces():
    # YOUR CALCULATIONS HERE
    # REMMEBER ITS IN 3D, EVERYTHING IS A VECTOR, TEHCNICALLY
    for mass in masses:
        force = mass.mass * GRAVITY

        if mass.position[2] < 0:
            restorative = [0, 0, - k * mass.position[2]]
            force = force + restorative

        acceleration = force / mass.mass
        velocity = mass.velocity + acceleration * TIMESTEP
        position = mass.position + velocity * TIMESTEP
        mass.update(position, velocity, acceleration, force)

# Attaching 3D axis to the figure
fig = plt.figure()
ax = p3.Axes3D(fig)

# One 3D Cube
masses, springs = gen_square(0.1, [1, 0, 1])

data = np.array([[mass.position[0], mass.position[1], mass.position[2]] for mass in masses])
sdata = []
for i, spring in enumerate(springs):
    sdata.append(spring.masses[0].position)
    sdata.append(spring.masses[1].position)
sdata = np.array(sdata)

vertexplot = ax.plot(data[:,0], data[:,1], data[:,2], 'bo')
springplot = ax.plot(sdata[:,0], sdata[:,1], sdata[:,2])

# Setting the axes properties
ax.set_xlim3d([0.0, 1.0])
ax.set_xlabel('X - meters')

ax.set_ylim3d([0.0, 1.0])
ax.set_ylabel('Y ')

ax.set_zlim3d([0.0, 1.0])
ax.set_zlabel('Z ')

ax.set_title('Physics Cube')

# Creating the Animation object
line_ani = animation.FuncAnimation(fig, update_vertexplot, 25, fargs=(masses, vertexplot),
                                   interval=100, blit=False)

plt.show()