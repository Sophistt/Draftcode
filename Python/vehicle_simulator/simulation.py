import os
import pygame
import pygame.gfxdraw
from math import tan, radians, degrees, copysign
from pygame.math import Vector2

class Vehicle:
    def __init__(self, x, y, angle=0.0, length=4, max_steering=30, max_acceleration=5.0):
        self.position = Vector2(x, y)
        self.velocity = Vector2(0.0, 0.0)
        self.angle = angle
        self.length = length
        self.max_acceleration = max_acceleration
        self.max_steering = max_steering
        self.max_velocity = 20
        self.brake_deceleration = 10
        self.free_deceleration = 2

        self.acceleration = 0.0
        self.steering = 0.0

    def update(self, dt):
        self.velocity += (self.acceleration * dt, 0)
        self.velocity.x = max(-self.max_velocity, min(self.velocity.x, self.max_velocity))

        if self.steering:
            turning_radius = self.length / tan(radians(self.steering))
            angular_velocity = self.velocity.x / turning_radius
        else:
            angular_velocity = 0

        self.position += self.velocity.rotate(-self.angle) * dt
        self.angle += degrees(angular_velocity) * dt


class Game:
    def __init__(self):
        pygame.init()
        pygame.display.set_caption("Vehicle Simulation")
        width = 1280
        height = 960
        self.screen = pygame.display.set_mode((width, height))
        self.clock = pygame.time.Clock()
        self.ticks = 60
        self.exit = False
    
    def drawRoadLine(self):
        for i in range(13):
            pygame.gfxdraw.line(self.screen, 100 * i, 480, 100 * i + 50, 480, (255, 255, 255))
        pygame.gfxdraw.line(self.screen, 0, 368, 1280, 368, (255, 255, 255))
        pygame.gfxdraw.line(self.screen, 0, 592, 1280, 592, (255, 255, 255))


    def run(self):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        image_path = os.path.join(current_dir, "image/car.png")
        obstacle_path = os.path.join(current_dir, "image/obstacle.png") 
        
        vehicle_image = pygame.image.load(image_path)
        obstacle_image = pygame.image.load(obstacle_path)
        
        ppu = 32

        vehicle = Vehicle(2, 15)
        obstacle = Vehicle(10, 16.75)

        while not self.exit:
            # Set tick    
            dt = self.clock.get_time() / 1000
            self.clock.tick(self.ticks)

            # Event queue
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.exit = True
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        self.exit = True
            
            # Path Plan and purepursuit
            vehicle.acceleration = 3
            vehicle.acceleration = max(-vehicle.max_acceleration, min(vehicle.acceleration, vehicle.max_acceleration))

            obstacle.velocity.x = 5
            obstacle.steering = 30

            # Logic
            vehicle.update(dt)
            obstacle.update(dt)

            # Drawing
            self.screen.fill((0, 0, 0))
            self.drawRoadLine()
            
            rotated = pygame.transform.rotate(vehicle_image, vehicle.angle)
            rect = rotated.get_rect()

            ob_rotated = pygame.transform.rotate(obstacle_image, obstacle.angle)
            ob_rect = ob_rotated.get_rect()

            self.screen.blit(rotated, vehicle.position * ppu - (rect.width / 2, rect.height / 2))
            self.screen.blit(ob_rotated, obstacle.position * ppu - (ob_rect.width / 2, ob_rect.height / 2))
            pygame.display.flip()

        pygame.quit()

if __name__ == "__main__":
    game = Game()
    game.run()

