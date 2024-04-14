import pygame, os, math, random
import numpy as np
os.environ['SDL_VIDEO_CENTERED'] = '1'

def name_to_colour(name):
    '''Return colour tuple given name.'''
    colours = {"red": (255, 0, 0),
               "green": (0, 255, 0),
               "blue": (0, 0, 255)}
    return colours[name]

# initialise pygame
pygame.init()

# dimensions of the canvas (pygame window)
cw, ch = 700, 700
# dimensions of the viewport (viewing plane projected onto)
vw, vh = 4, 4
# create canvas to draw to
canvas = pygame.display.set_mode((cw, ch))
pygame.display.set_caption("Wireframe")
clock = pygame.time.Clock()
font = pygame.font.SysFont("Arial" , 18 , bold = True)

# distance from camera point to screen
d = 1

def Rx(theta):
    '''Rotation matrix: theta about x-axis'''
    return np.array([[1.0, 0.0, 0.0], [0.0, np.cos(theta), -np.sin(theta)], [0.0, np.sin(theta), np.cos(theta)]])

def Ry(theta):
    '''Rotation matrix: theta about y-axis'''
    return np.array([[np.cos(theta), 0.0, np.sin(theta)], [0.0, 1.0, 0.0], [-np.sin(theta), 0.0, np.cos(theta)]])

def Rz(theta):
    '''Rotation matrix: theta about z-axis'''
    return np.array([[np.cos(theta), -np.sin(theta), 0.0], [np.sin(theta), np.cos(theta), 0.0], [0.0, 0.0, 1.0]])

def rotation_matrix(angles):
    '''
    Rotation anticlockwise about x, then y, then z
    R = Rz(psi) @ Ry(phi) @ Rx(theta)
    '''
    # return Rz(angles[2]) @ Ry(angles[1]) @ Rx(angles[0])
    return Rz(angles[2]), Ry(angles[1]), Rx(angles[0])


def inverse_rotation_matrix(angles):
    '''
    Rotation clockwise about z, then y, then x
    Inverse of rotation_matrix(theta, phi, psi)
    R = Rx(-theta) @ Ry(-phi) @ Rz(-psi)
    '''
    # return Rx(-angles[0]) @ Ry(-angles[1]) @ Rz(-angles[2])
    return Rx(-angles[0]), Ry(-angles[1]), Rz(-angles[2])

class Model:
    def __init__(self, scale, rotation, translation, base, lines):
        '''
        scale: scale factor of verices (float)
        rotation: rotation of vertices (matrix)
        translation: translation of vertices (shape (3) numpy array)
        base: vertices of the base model
        lines: lines of the base model
        '''
        # base vertices of model
        self.base = base
        # vertices of instance
        self.points = base
        # lines of instance
        self.lines = lines
        # centre of model: current translation from origin
        self.centre = np.zeros(3)
        # apply given scale
        self.scale_transform(scale)
        # apply given rotation
        self.rotation_transform(rotation)
        # apply given translation
        self.translation_transform(translation)

    def scale_transform(self, scale):
        '''Scale model about centre by factor: scale.'''
        # centre at origin
        points = self.points - self.centre
        # scale vertices
        points = points * scale
        # re-centre
        self.points = points + self.centre

    def rotation_transform(self, rotation):
        '''Rotate model about centre by: rotation.'''
        # centre at origin
        points = self.points - self.centre
        # rotate vertices
        # points = points @ rotation.T
        points = (((points @ rotation[2].T) @ rotation[1].T) @ rotation[0].T)
        # re-centre
        self.points = points + self.centre

    def translation_transform(self, translation):
        '''Translate model about centre by: translation.'''
        # translate points
        self.points = self.points + translation
        # update centre
        self.centre = self.centre + translation


class Cube(Model):
    def __init__(self, scale=1, rotation=rotation_matrix(np.zeros(3)), translation=np.zeros(3)):
        # vertices of base cube
        base = np.array([
                        [1, 1, 1],
                        [-1, 1, 1],
                        [-1, -1, 1],
                        [1, -1, 1],
                        [1, 1, -1],
                        [-1, 1, -1],
                        [-1, -1, -1],
                        [1, -1, -1]
                        ])
        # lines of cube
        lines = []
        for i in range(8):
            for j in range(8):
                colour = ["red","green","blue"][(i + j) % 3]
                lines.append([i, j, colour])
        # initialize parent: applies transforms
        super().__init__(scale, rotation, translation, base, lines)

class Tetra(Model):
    def __init__(self, scale=1, rotation=rotation_matrix(np.zeros(3)), translation=np.zeros(3)):
        # vetices of base tetrahedron
        base = np.array([
            [1, 0, -1/np.sqrt(2)],
            [-1, 0, -1/np.sqrt(2)],
            [0, 1, 1/np.sqrt(2)],
            [0, -1, 1/np.sqrt(2)]
        ])
        # lines of tetrahedron
        lines = []
        for i in range(4):
            for j in range(4):
                colour = ["red","green","blue"][(i + j) % 3]
                lines.append([i, j, colour])
        super().__init__(scale, rotation, translation, base, lines)


# create list of instances of models in scene
#v_list = [[4, 4, 4], [4, 4, -4], [-4, 4, 4], [-4, 4, -4],
#          [4, -4, 4], [4, -4, -4], [-4, -4, 4], [-4, -4, -4]]
v_list = [[x, y, z] for x in [-4, -2, 2, 4] for y in [-4, -2, 2, 4] for z in [-4, -2, 2, 4]]
instances = [Tetra(translation=np.array(v)) for v in v_list]

def camera_transform(points, position, rotation):
    '''Transform 3D points relative to camera movement.'''
    # inverse translation
    points = points - position
    # inverse rotation
    # points = points @ rotation.T
    points = (((points @ rotation[2].T) @ rotation[1].T) @ rotation[0].T)
    return points
    
def project_point(point):
    """Project 3D point onto viewport."""
    projection = np.array([point[0] * (d / point[2]), point[1] * (d / point[2]), d])
    return projection

def viewport_to_canvas(point):
    """Map 3D point on the viewport to 2D point on the canvas."""
    return (point[0] * (cw / vw), point[1] * (ch / vh))

def canvas_to_pygame(point):
    """Translate 2D canvas coordinate to pygame screen coordinate."""
    return (cw/2 + point[0], ch/2 - point[1])

def input(camera_position, camera_angles):
    '''
    Listen for exit inputs
    Update camera position from inputs
    Update camera rotation from inputs
    '''
    running = True
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                pygame.quit()
                running = False
    # get held keys
    keys = pygame.key.get_pressed()
    # store movement (axis-aligned movement)
    move = np.zeros(3)
    # stepsize
    step = 0.01
    # movement in x-y plane: forward, backward, left, right
    if keys[pygame.K_w]:
        move[2] += step
    if keys[pygame.K_s]:
        move[2] -= step
    if keys[pygame.K_a]:
        move[0] -= step
    if keys[pygame.K_d]:
        move[0] += step
    # movement along y axis: up, down
    if keys[pygame.K_SPACE]:
        move[1] += step
    if keys[pygame.K_LSHIFT]:
        move[1] -= step

    # rotation size
    angle = 0.01
    # rotate about x-axis: up, down
    if keys[pygame.K_UP]:
        # prevent looking up past vertical
        if camera_angles[0] > -math.pi/2:
            camera_angles[0] -= angle
    if keys[pygame.K_DOWN]:
        # prevent looking down past vertical
        if camera_angles[0] < math.pi/2:
            camera_angles[0] += angle
    # rotate about y-axis: left, right
    if keys[pygame.K_LEFT]:
        camera_angles[1] -= angle
    if keys[pygame.K_RIGHT]:
        camera_angles[1] += angle
    # rotate about z-axis: (,),(.)
    if keys[pygame.K_COMMA]:
        camera_angles[2] += angle
    if keys[pygame.K_PERIOD]:
        camera_angles[2] -=  angle

    # rotate move direction to point in direction of camera
    # BUT: not rotation about x-axis
    # ensures no change in y value when moving forward and looking up/down
    camera_angles_adj = [0, camera_angles[1], camera_angles[2]]
    # move = move @ rotation_matrix(camera_angles_adj).T
    cam_rotation = rotation_matrix(camera_angles_adj)
    move = (((move @ cam_rotation[2].T) @ cam_rotation[1].T) @ cam_rotation[0].T)

    # update camera position
    camera_position = camera_position + move

    return running, camera_position, camera_angles

def framerate_counter():
    """Calculate and display frames per second."""
    # get fps
    fps = str(int(clock.get_fps()))
    # create text
    fps_t = font.render(fps , 1, name_to_colour("green"))
    # display on canvas
    canvas.blit(fps_t,(0,0))

def update_instances():
    """Update/modify instances in the scene."""
    R = rotation_matrix([0, 0.001, 0])
    for instance in instances:
        instance.rotation_transform(R)

def render_instances(position, rotation):
    """Draw lines for every instance to canvas."""
    # compute current camera rotation matrix
    rotation_matrix = inverse_rotation_matrix(rotation)
    # loop over all instances in scene
    for instance in instances:
        # camera transform
        points = camera_transform(instance.points, position, rotation_matrix)
        # project all points
        projected = []
        # for point in instance.points:
        for point in points:
            projected.append(project_point(point))
        # loop over lines
        for line in instance.lines:
            # line endpoints in world space (3D)
            a = points[line[0]]
            b = points[line[1]]
            # (1) both endpoints have z >= d
            if a[2] >= d and b[2] >= d:
                # no change to projected endpoints
                A = projected[line[0]]
                B = projected[line[1]]
            # (2) both endpoints have z < d
            elif a[2] < d and b[2] < d:
                # do not draw, go to next line
                continue
            # (3) a has z < d, b has z >= d
            elif a[2] < d and b[2] >= d:
                # replace projection of a by intersection of line with viewport
                A = a + ((d - a[2]) / (b[2] - a[2])) * (b - a)
                B = projected[line[1]]
            # (4) a has z >= d, b has z < d
            elif a[2] >= d and b[2] < d:
                # replace projection of b by intersection of line with viewport
                A = projected[line[0]]
                B = b + ((d - b[2]) / (a[2] - b[2])) * (a - b)
            # Map projected endpoints to canvas, then map to pygame coords
            x = canvas_to_pygame(viewport_to_canvas(A))
            y = canvas_to_pygame(viewport_to_canvas(B))
            colour = name_to_colour(line[2])
            # draw line to canvas
            pygame.draw.line(canvas, colour, x, y)

from profilehooks import profile
@profile
def main():
    # set game state
    running =  True
    # initialize camera position
    camera_position = np.zeros(3)
    # initialize camera rotation
    camera_rotation = np.zeros(3)
    while running:
        # draw background
        pygame.draw.rect(canvas, (0, 0, 0), pygame.Rect(0, 0, cw, ch))
        # update instances
        update_instances()
        # check for inputs
        running, camera_position, camera_rotation = input(camera_position, camera_rotation)
        # draw instances
        render_instances(camera_position, camera_rotation)
        # call clock
        clock.tick()
        # show fps
        framerate_counter()
        # update canvas
        pygame.display.update()

if __name__ == "__main__":
    main()