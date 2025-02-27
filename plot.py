import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
import matplotlib.cm as cm
import math as math
from physics_cube import Mass, Spring

# Fixing random state for reproducibility
np.random.seed(19680801)

GRAVITY = np.array([0, 0, -9.81])
TIMESTEP = 0.002
global_time = 0 
friction_mu_s = 1
friction_mu_k = 0.8
k_ground = 10000
omega = 10 
k = 10000


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

def gen_tetrahedron(edge, origin):
    """
    Create a square with lengths length
    lower corner is the "origin" of square
    """
    mass = 0.1
    masses = [None] * 4
    springs = []
    masses[0] = Mass(mass, np.array(origin))
    masses[1] = Mass(mass, np.array([origin[0] + edge, origin[1], origin[2]]))
    #masses[2] = Mass(mass, np.array([origin[0] + edge, origin[1] + edge, origin[2]]))
    masses[2] = Mass(mass, np.array([origin[0], origin[1] + edge, origin[2]]))
    masses[3] = Mass(mass, np.array([origin[0], origin[1], origin[2]+edge]))
    #masses[5] = Mass(mass, np.array([origin[0] + edge, origin[1], origin[2]+edge]))
    #masses[6] = Mass(mass, np.array([origin[0]+edge, origin[1]+edge, origin[2] + edge]))
    #masses[7] = Mass(mass, np.array([origin[0], origin[1]+edge, origin[2]+edge]))

    #make springs
    for i, mass in enumerate(masses):
        j = i
        while j < len(masses):
            length = np.linalg.norm(mass.position - masses[j].position)
            if length > 0:
                spring = Spring(k, length, [mass, masses[j]])
                springs.append(spring)
            j +=1

    return masses, asprings

def breathe(masses):
    for i, mass in enumerate(masses):
        mass = masses[i]
        if i > 4: # top vertices
            mass.position[2] = mass.position[2] + np.sin(global_time * 100)/1000
        if i == 0 or i == 3 or i == 4 or i == 7:
            mass.position[0] = mass.position[0] - np.sin(global_time * 100)/2000
            mass.position[1] = mass.position[1] - np.sin(global_time * 100)/2000
        else:
            mass.position[0] = mass.position[0] + np.sin(global_time * 100)/2000
            mass.position[1] = mass.position[1] + np.sin(global_time * 100)/2000

def locomotion():
    w = 1 
    L = a + b * np.sin(w * global_time + c)

def rotateCube(passmass):
    a = np.random.rand() * math.pi
    trans_x = np.array([[1, 0, 0],
                        [0, np.cos(a), -np.sin(a)],
                        [0, np.sin(a), np.cos(a)]])
    trans_y = np.array([[np.cos(a), 0, np.sin(a)],
                        [0, 1, 0],
                        [-np.sin(a), 0, np.cos(a)]])
    for mass in passmass:
        mass.update_pos(mass.position @ trans_x)
        mass.update_pos(mass.position @ trans_y)

def update_vertexplot(num, masses, vertexplot):
    calculate_forces()
    #breathe(masses)
    data = [[mass.position[0], mass.position[1], mass.position[2]] for mass in masses]
    x = [row[0] for row in data]
    y = [row[1] for row in data]
    z = [row[2] for row in data]
    for i,vertex in enumerate(vertexplot):
        vertex[0].set_data([[x[i]],[y[i]]])
        vertex[0].set_3d_properties([z[i]])

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

    global global_time
    global_time = global_time + TIMESTEP

    return vertexplot

def calculate_forces():
    forces = []
    avg_velocity = 0
    for mass in masses:
        force = mass.mass * GRAVITY

        for spring in springs:
            force_direction = [0,0,0] # needs right direction depending on which mass you're calculating for
            spring_force_vector = [0,0,0] #final 
            if mass == spring.getMassOne() or mass == spring.getMassTwo():
                if mass == spring.getMassOne():
                    force_direction = spring.getMassTwo().position - spring.getMassOne().position
                if mass == spring.getMassTwo():
                    force_direction  = spring.getMassOne().position - spring.getMassTwo().position
                
                force_direction = force_direction / np.linalg.norm(force_direction)
                spring_force = k * (spring.getLength() - spring.getL0())
                spring_force_vector = spring_force * force_direction
                #print("length difference", spring.getLength() - spring.getL0())
                #print("spring force", mass, "val", spring_force)
                #print("spring_force_vector", spring_force_vector, "dir", force_direction)
            force = force + spring_force_vector

        if mass.position[2] < 0:
            restorative = [0, 0, - k_ground * mass.position[2]]
            force = force + restorative
        forces.append(force)

    for i, mass in enumerate(masses):
        force = forces[i]
        acceleration = force / mass.mass
        velocity = mass.velocity + acceleration * TIMESTEP
        position = mass.position + velocity * TIMESTEP
        mass.update(position, velocity, acceleration, 0)

        """ ENERGY LOGGING """
        #f.write(str(0.5 * 0.1 * np.linalg.norm(mass.position[2]) ** 2)+'\n')
        #avg_velocity += velocity
        # write data to file every like 
        #fk.write(str(0.5 * 0.1 * np.linalg.norm(avg_velocity[0]) ** 2)+'\n')


# Attaching 3D axis to the figure
fig = plt.figure()
ax = p3.Axes3D(fig)

# One 3D Cube
#masses, springs = gen_square(0.15, [.5, 0.5, 0.5])
masses, springs = gen_square(0.15, [.5, 0.5, 0.5])
#rotateCube(masses)
#f = open("potential.txt","w+")
#fk = open("kinetic.txt","w+")

# Spring Data for Plotting
data = np.array([[mass.position[0], mass.position[1], mass.position[2]] for mass in masses])
sdata = []
for i, spring in enumerate(springs):
    sdata.append(spring.masses[0].position)
    sdata.append(spring.masses[1].position)
sdata = np.array(sdata)

# Separate the vertices into separate plots
colors = cm.rainbow(np.linspace(0, 1, len(masses)))
vertexplot = [None] * len(masses)
for i, (mass, color) in enumerate(zip(masses, colors)):
    pos = mass.position
    vertexplot[i] = (ax.plot([pos[0]], [pos[1]], [pos[2]], 'o', c = color))
#vertexplot = ax.plot(data[:,0], data[:,1], data[:,2], 'bo')
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
line_ani = animation.FuncAnimation(fig, update_vertexplot, fargs=(masses, vertexplot),
                                   interval=5, blit=False)

plt.show()