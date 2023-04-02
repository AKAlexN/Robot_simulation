import math
import pygame
import numpy as np
from Robot import Graphica, bot, Sensor
map_dim = (748, 1200)

grfx = Graphica(map_dim, 'Robot.PNG', 'MAP.png')
start = (500,600)#500,500
Bot = bot(start, 37.7952)

sensor_range = 150, math.radians(25) #250 40
lidar = Sensor(sensor_range, grfx.map)
dt = 0
last_time = pygame.time.get_ticks()
running =True
while running :
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running=False
    dt = (pygame.time.get_ticks() - last_time) / 1000
    last_time = pygame.time.get_ticks()
    grfx.map.blit(grfx.map_im, (0, 0))

    Bot.kinem(dt)
    grfx.drow_bot(Bot.x, Bot.y, Bot.heading )
    point_cloud = lidar.sens_obs(Bot.x, Bot.y, Bot.heading)
    print(Bot.x, Bot.y)
    Bot.avoid_Obs(point_cloud, dt)
    grfx.drow_sensd(point_cloud)
    pygame.display.update()
