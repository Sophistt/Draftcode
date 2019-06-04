import pygame
import math
import random
from pygame.locals import *
import os


class Player(pygame.sprite.Sprite):
    def __init__(self):
        super(Player, self).__init__()
        self.surf = pygame.Surface((75, 25))
        self.surf.fill((255, 255, 255))
        self.rect = self.surf.get_rect()

    def update(self, pressed_keys):
        if pressed_keys[K_UP]:
            self.rect.move_ip(0, -5)
        if pressed_keys[K_DOWN]:
            self.rect.move_ip(0, 5)
        if pressed_keys[K_LEFT]:
            self.rect.move_ip(-5, 0)
        if pressed_keys[K_RIGHT]:
            self.rect.move_ip(5, 0)


class Vehicle(pygame.sprite.Sprite):
    def __init__(self):
        super(Vehicle, self).__init__()
        self.basic_surf = pygame.Surface((20, 12))
        self.basic_surf.set_colorkey((0, 0, 0))
        self.basic_surf.fill((0, 255, 0))

        # Define position
        self.xpos = self.basic_xpos = 400
        self.ypos = self.basic_ypos = 300

        self.basic_rect = self.basic_surf.get_rect(center=(self.basic_xpos, self.basic_ypos))

        # Define speed
        self.xspeed = 1
        self.yspeed = 0

        self.rot = 0
        self.rot_speed = -200

        self.surf = self.basic_surf
        self.rect = self.basic_rect

        self.radius = 50

    def rotate_according_to_turning_radius(self, radius, rotate_speed):
        if rotate_speed >= 0:
            x_ordinate = self.xpos - radius * math.sin(math.radians())
            y_ordinate = self.ypos - radius * math.cos(math.radians())
        else:
            x_ordinate = self.xpos + radius * math.sin(math.radians())
            y_ordinate = self.ypos + radius * math.cos(math.radians())

        self.rot = (self.rot + rotate_speed) % 360

        if rotate_speed >= 0:
            self.xpos = x_ordinate + radius * math.sin(math.radians(self.rot))
            self.ypos = y_ordinate + radius * math.sin(math.radians(self.rot))
        else:
            self.xpos = x_ordinate - radius * math.sin(math.radians(self.rot))
            self.ypos = y_ordinate - radius * math.sin(math.radians(self.rot))

    def compute(self):
        if self.rot_speed >= 0:
            xord = self.xpos - self.radius * math.sin(math.radians(self.rot))
            yord = self.ypos - self.radius * math.cos(math.radians(self.rot))
            self.rot = (self.rot + self.rot_speed) % 360
            self.xpos = xord + self.radius * math.sin(math.radians(self.rot))
            self.ypos = yord + self.radius * math.cos(math.radians(self.rot))
        if self.rot_speed < 0:
            xord = self.xpos + self.radius * math.sin(math.radians(self.rot))
            yord = self.ypos + self.radius * math.cos(math.radians(self.rot))
            self.rot = (self.rot + self.rot_speed) % 360
            self.xpos = xord - self.radius * math.sin(math.radians(self.rot))
            self.ypos = yord - self.radius * math.cos(math.radians(self.rot))

    def update(self):
        #self.rot = (self.rot + self.rot_speed) % 360
        #self.xpos += self.xspeed
        self.compute()

        self.surf = pygame.transform.rotate(self.basic_surf, self.rot)
        self.rect = self.surf.get_rect()
        self.rect.center = (self.xpos, self.ypos)

        #self.rect.move_ip(self.speed, 0)
        if self.rect.left > 800:
            self.kill()


def main():
    pygame.init()
    screen = pygame.display.set_mode((800, 600))

    clock = pygame.time.Clock()

    player = Player()
    vehicle = Vehicle()

    background = pygame.Surface(screen.get_size())
    background.fill((0, 0, 255))

    running = True
    while running:
        clock.tick(200)
        for event in pygame.event.get():
            if event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    running = False
            elif event.type == QUIT:
                running = False

        screen.blit(background, (0, 0))

        pressed_keys = pygame.key.get_pressed()

        player.update(pressed_keys)
        vehicle.update()

        screen.blit(player.surf, player.rect)
        screen.blit(vehicle.surf, vehicle.rect)
        pygame.display.flip()


if __name__ == "__main__":
    main()
