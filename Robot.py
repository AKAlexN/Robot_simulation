import math
import pygame
import numpy as np


def distance(point1, point2):
    point1 = np.array(point1)
    point2 = np.array(point2)
    return (np.linalg.norm(point1 - point2))


class bot:
    def __init__(self, startpos, width):
        self.MtoP = 3779.52
        self.w = width
        self.x = startpos[0]
        self.y = startpos[1]
        self.heading = 0
        self.vl = 0.01 * self.MtoP
        self.vr = 0.01 * self.MtoP
        self.maxspeed = 0.02 * self.MtoP
        self.minspeed = 0.01 * self.MtoP
        self.min_obsDist = 150
        self.countDown = 5

    def avoid_Obs(self, point_cloud, dt):
        closest_obs = None
        dist = np.inf
        if len(point_cloud) > 1:
            for point in point_cloud:
                if dist > distance([self.x, self.y], point):
                    dist = distance([self.x, self.y], point)
                    closest_obs = (point, dist)
            if closest_obs[1] < self.min_obsDist and self.countDown > 0:
                self.countDown -= dt
                self.move_backward()
            else:
                self.countDown = 5
                self.move_forward()

    def move_backward(self):
        self.vr = - self.minspeed
        self.vl = - self.minspeed / 2

    def move_forward(self):
        self.vr = self.minspeed
        self.vl = self.minspeed / 2

    def kinem(self, dt):
        self.x += ((self.vl + self.vr) / 2) * math.cos(self.heading) * dt
        self.y -= ((self.vl + self.vr) / 2) * math.sin(self.heading) * dt
        self.heading += (self.vr - self.vl) / self.w * dt

        if self.heading > 2 * math.pi or self.heading < - 2 * math.pi:
            self.heading = 0
        self.vr = max(min(self.maxspeed, self.vr), self.minspeed)
        self.vl = max(min(self.maxspeed, self.vl), self.minspeed)


class Graphica:
    def __init__(self, dimention, robot_img, map_img):
        pygame.init()
        self.black = (0, 0, 0)
        self.grey = (70, 70, 70)
        self.Blue = (0, 0, 255)
        self.Green = (0, 255, 0)
        self.Red = (255, 0, 0)
        self.white = (255, 255, 255)

        self.bot = pygame.image.load(robot_img)
        self.map_im = pygame.image.load(map_img)
        self.height, self.width = dimention
        self.MapWindowName = 'SLAM'
        pygame.display.set_caption(self.MapWindowName)
        self.map = pygame.display.set_mode((self.width, self.height))
        self.map.blit(self.map_im, (0, 0))

    def drow_bot(self, x, y, heading):
        rotated = pygame.transform.rotozoom(self.bot, math.degrees(heading), 1)
        rect = rotated.get_rect(center=(x, y))
        self.map.blit(rotated, rect)

    ############################################################################################### sens
    def drow_sensd(self, point_cloud):
        for point in point_cloud:
            pygame.draw.circle(self.map, self.white, point, 3, 0)


class Sensor:
    def __init__(self, sens_range, map):
        self.sens_range = sens_range
        self.mapW, self.mapH = pygame.display.get_surface().get_size()
        self.map = map

    def sens_obs(self, x, y, heading):
        obs = []
        x1, y1 = x, y
        start_angle = heading - self.sens_range[1]
        fin_angle = heading + self.sens_range[1]
        for angle in np.linspace(start_angle, fin_angle, 5, False):
            x2, y2 = (x1 + self.sens_range[0] * math.cos(angle), y1 - self.sens_range[0] * math.sin(angle))
            for i in range(0, 100):
                u = i / 100
                x = int(x2 * u + x1 * (1 - u))
                y = int(y2 * u + y1 * (1 - u))
                if 0 < x < self.mapW and 0 < y < self.mapH:
                    color = self.map.get_at((x, y))
                    self.map.set_at((x, y), (0, 255, 0))
                    if (color[0], color[1], color[2]) == (0, 0, 0):
                        obs.append([x, y])
                        break
        return obs
