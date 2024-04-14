import pygame, os, math, random
import numpy as np
import pygame.gfxdraw
os.environ['SDL_VIDEO_CENTERED'] = '1'

def input():
    '''
    Listen for exit inputs
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
    return running

def sphere_SDF(point, centre, radius):
    return np.linalg.norm(point - centre) - radius

def box_SDF(point, b):
    q = abs(point) - b
    return max(np.linalg.norm(q), 0.0) + min(max(q[0], max(q[1], q[2])), 0.0)

def SDF(point, mode, t):
    if mode == "spheres":

        # mod point
        q = np.mod(point, 5) - 0.5
        centre_1 = np.array([2.5, 2.5, 3.0 + np.sin(t)])
        radius_1 = 1.0
        sphere_dist_1 = sphere_SDF(q, centre_1, radius_1)

        return sphere_dist_1
    elif mode == "intersection":

        centre_1 = np.array([-1.0, 1.0, 1.0])
        centre_2 = np.array([0.0, 0.0, 2.0])
        radius_1 = 1
        radius_2 = 2.0
        sphere_dist_1 = sphere_SDF(point, centre_1, radius_1)
        sphere_dist_2 = sphere_SDF(point, centre_2, radius_2)
        
        return max(-sphere_dist_1, sphere_dist_2)
    
    elif mode == "soft":

        centre_1 = np.array([0.0, np.sin(t), 1.0])
        radius_1 = 1
        sphere_dist_1 = sphere_SDF(point, centre_1, radius_1)
        
        ground_dist = point[1] + 1

        return min(ground_dist, sphere_dist_1)

def ray_march(ray_origin, ray_direction, t):
    '''March a ray from origin in direction.'''
    # distance travelled
    total_distance = 0.0
    # parameters
    steps = 25
    min_distance = 0.01
    max_distance = 50
    # march
    for i in range(steps):
        # get position
        current_position = ray_origin + total_distance * ray_direction
        # get closest distance to all objects
        closest_distance = SDF(current_position, "spheres", t)

        if closest_distance < min_distance:
            return (0, 255 - (i / steps)*255, 255 - (i / steps)*255)
        elif total_distance > max_distance:
            return None
        
        total_distance += closest_distance

    return None

from profilehooks import profile
@profile
def main():
    # initialise pygame
    pygame.init()

    # screen dimensions (pygame window)
    screen_width, screen_height = 500, 500
    # screen resolutions (number of grid rows / columns)
    x_res, y_res = 100, 100
    # grid square dimensions
    grid_width, grid_height = screen_width / x_res, screen_height / y_res

    # create window
    window = pygame.display.set_mode((screen_width, screen_height))
    pygame.display.set_caption("Raymarching")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("Arial" , 18 , bold = True)

    # create surface to draw to
    canvas = pygame.Surface((x_res, y_res))

    # set game state
    running = True
    # initialize camera position
    camera_position = np.array([0.0, 0.0, -5.0])
    # time
    t = 0
    while running:
        t += 0.1
        # draw background
        # pygame.draw.rect(canvas, (0, 0, 0), pygame.Rect(0, 0, screen_width, screen_height))
        canvas.fill((0, 0, 0))

        # loop over grid squares
        for j, y_pos in enumerate(np.arange(-screen_height / 2, screen_height / 2 + grid_height, grid_height)):
            for i, x_pos in enumerate(np.arange(-screen_width / 2, screen_width / 2 + grid_width, grid_width)):

                # ray origin: camera
                ro = camera_position
                # ray direction: towards grid square (scaled by screen size to be in [-1, 1]^2)
                rd = np.array([x_pos / (screen_width / 2), y_pos / (screen_height / 2), 1])
                # normalize
                rd = rd / np.linalg.norm(rd)

                # raymarch
                colour = ray_march(ro, rd, t)
                # colour = (i, j, 0)

                if colour:
                    # draw square at position
                    #pygame.draw.rect(canvas, colour,
                    #    pygame.Rect(screen_width/2 + x_pos, screen_height/2 - y_pos, grid_width, grid_height))
                    #pygame.gfxdraw.box(canvas, 
                    #                pygame.Rect(screen_width/2 + x_pos, screen_height/2 - y_pos, grid_width, grid_height),
                    #                colour)
                    pygame.gfxdraw.pixel(canvas, i, y_res - j, colour)
        window.blit(pygame.transform.scale(canvas, window.get_rect().size), (0,0))

        # call clock
        clock.tick()
        # get fps
        fps = str(int(clock.get_fps()))
        # create text
        fps_t = font.render(fps , 1, (0, 255, 0))
        # display on canvas
        window.blit(fps_t,(0,0))

        # update canvas
        pygame.display.update()
        # check for exit
        running = input()

if __name__ == "__main__":
    main()