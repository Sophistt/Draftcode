import pygame
import pygame.gfxdraw
import math
from pygame.locals import *



class Point(pygame.sprite.Sprite):
    def __init__(self):
        super(Point, self).__init__()
        self.surf = pygame.Surface((1, 1))
        self.surf.fill((255, 255, 255))
        self.rect = self.surf.get_rect()

class Circle(pygame.sprite.Sprite):
    def __init__(self):
        super(Circle, self).__init__()
        self.surf = pygame.Surface((30,30), pygame.SRCALPHA)
        self.rect = self.surf.get_rect()
        #pygame.gfxdraw.aacircle(self.surf, 15, 15, 14, (0, 255, 255))
        pygame.gfxdraw.rectangle(self.surf, self.rect, (0, 255, 255))

    def update(self):
        self.rect.move_ip(1, 0)

class Vehicle(pygame.sprite.Sprite):
    def __init__(self,
        xpos,
        ypos):
        super(Vehicle, self).__init__()

        # Vehicle Parameter
        # basic_xpos and basic_ypos are used for transform the rectangle
        self.xpos = self.basic_xpos = xpos
        self.ypos = self.basic_ypos = ypos

        self.width = 10
        self.height = 24
        self.wheelBase = 15

        self.rot = 0

        # Use basic_surf and basic_rect to plot the vehicle
        # Basic_surf and basic_rect are used to tranform the rectangle
        self.basic_surf = pygame.Surface((self.height, self.width), pygame.SRCALPHA)
        self.basic_rect = self.basic_surf.get_rect()
        pygame.gfxdraw.rectangle(self.basic_surf, self.basic_rect, (0, 255, 255))
        self.basic_rect.center = (self.basic_xpos, self.basic_ypos)

        # ------------
        self.surf = self.basic_surf
        self.rect = self.basic_rect

    
    def rotate_according_to_turning_radius(self, radius, rotate_speed):
        # TODO: rotate around the rear axle midpoint
        if rotate_speed >= 0:
            x_ordinate = self.xpos - radius * math.sin(math.radians(self.rot))
            y_ordinate = self.ypos - radius * math.cos(math.radians(self.rot))
            self.rot = (self.rot + rotate_speed) % 360
            self.xpos = x_ordinate + radius * math.sin(math.radians(self.rot))
            self.ypos = y_ordinate + radius * math.cos(math.radians(self.rot))
        else:
            x_ordinate = self.xpos + radius * math.sin(math.radians(self.rot))
            y_ordinate = self.ypos + radius * math.cos(math.radians(self.rot))
            self.rot = (self.rot + rotate_speed) % 360
            self.xpos = x_ordinate - radius * math.sin(math.radians(self.rot))
            self.ypos = y_ordinate - radius * math.cos(math.radians(self.rot))

            self.surf = pygame.transform.rotate(self.basic_surf, self.rot)
            self.rect = self.surf.get_rect()
            self.rect.center = (self.xpos, self.ypos)


    def setCenter(self, xpos, ypos):
        self.rect.center = (xpos, ypos)


    def update(self):
        #self.rect.move_ip(0, 1)
        self.rotate_according_to_turning_radius(50, -5)

        self.surf = pygame.transform.rotate(self.basic_surf, self.rot)
        self.rect = self.surf.get_rect()
        self.rect.center = (self.xpos, self.ypos)



#-------------------------------------
# beizer curve
#-------------------------------------
def bezier(control_points, rank):
    temp_points = []
    planpath = []
   
    for i in range(200):
        u = i / 199.0
        for index in range(len(control_points)):
            temp_points.append(dict(control_points[index]))
        
        j = 1
        while j <= rank:
            k = 0
            while k <= (rank - j):
                temp_points[k]["x"] = (1-u) * temp_points[k]["x"] + u * temp_points[k+1]["x"]
                temp_points[k]["y"] = (1-u) * temp_points[k]["y"] + u * temp_points[k+1]["y"]
                k += 1
            j+=1
        planpath.append(dict(temp_points[0]))

    return planpath


def main():
    pygame.init()
    screen = pygame.display.set_mode((800, 600))

    clock = pygame.time.Clock()
    
    ob_vehicle = Vehicle(400, 300)

    ap_point = Circle()
    ap_point.rect.center = (350, 200)
    
    local_point = Circle()
    local_point.rect.center = (400, 300)
    print(local_point.rect.center)
    
    background = pygame.Surface(screen.get_size())
    background.fill((0, 0, 0))

    running = True
    while running:
        clock.tick(60)
        for event in pygame.event.get():
            if event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    running = False
            elif event.type == QUIT:
                running = False
        
        # Set 4 control points of bezier curve
        ap_dict = {"x":ap_point.rect.center[0], "y":ap_point.rect.center[1]}
        ap1_dict = {"x":ap_point.rect.center[0], "y":ap_point.rect.center[1] + 40}
        local1_dict = {"x":local_point.rect.center[0], "y":local_point.rect.center[1] - 40}
        local_dict = {"x":local_point.rect.center[0], "y":local_point.rect.center[1]}
        cp = [local_dict, local1_dict, ap1_dict, ap_dict]
        
        # Compute bezier curve
        pathplan = bezier(cp, 3)

        screen.blit(background, (0, 0))

        for i in range(len(pathplan)):
            screen.set_at([int(pathplan[i]['x']), int(pathplan[i]['y'])], (255, 255, 255))

        screen.blit(ob_vehicle.surf, ob_vehicle.rect)
        screen.blit(ap_point.surf, ap_point.rect)
        screen.blit(local_point.surf, local_point.rect)
        pygame.display.flip()

        ap_point.update()
        ob_vehicle.rotate_according_to_turning_radius(50, -5)



if __name__ == "__main__":
    main()
