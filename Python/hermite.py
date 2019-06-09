import pygame
import pygame.gfxdraw
import pygame.draw
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
        ypos,
        color):
        super(Vehicle, self).__init__()

        # Vehicle Parameter
        # basic_xpos and basic_ypos are used for transform the rectangle
        self.xpos = self.basic_xpos = xpos
        self.ypos = self.basic_ypos = ypos
        self.rot = 0  # The direction of the vehicle, it equals 0 if the vehicle face right

        self.width = 10  # 2.0m
        self.height = 24  # 4.8m
        self.wheelBase = 15  #3.0m

        # Use basic_surf and basic_rect to plot the vehicle
        # Basic_surf and basic_rect are used to tranform the rectangle
        self.surf = self.basic_surf = pygame.Surface((self.height, self.width), pygame.SRCALPHA)
        self.rect = self.basic_rect = self.basic_surf.get_rect()
        pygame.gfxdraw.rectangle(self.basic_surf, self.basic_rect, color)
        self.basic_rect.center = (self.basic_xpos, self.basic_ypos)

    #-----------------------------------------------------------
    # Update_through_steer_and_speed
    # Desciption: Update the position and direction of the vehicle according to steer and speed
    # Input: speed(m/s), angle(degree)
    #-----------------------------------------------------------
    def update_through_steer_and_speed(self, angle, speed):
        # TODO: rotate around the rear axle midpoint
        if angle == 0:
            self.rect = self.surf.get_rect()
            self.xpos = self.xpos + speed / 6
            self.rect.center = (self.xpos, self.ypos)
        
        if angle > 0:
            radius = self.wheelBase / math.tan(math.radians(angle))
            x_ordinate = self.xpos - radius * math.sin(math.radians(self.rot))
            y_ordinate = self.ypos - radius * math.cos(math.radians(self.rot))

            delta_rot = math.asin(speed / (6 * radius))
            self.rot = (self.rot + math.degrees(delta_rot)) % 360

            self.xpos = x_ordinate + radius * math.sin(math.radians(self.rot))
            self.ypos = y_ordinate + radius * math.cos(math.radians(self.rot))

            
            self.surf = pygame.transform.rotate(self.basic_surf, self.rot)
            self.rect = self.surf.get_rect()
            self.rect.center = (self.xpos, self.ypos)

        if angle < 0:
            radius = abs(self.wheelBase / math.tan(math.radians(angle)))
            x_ordinate = self.xpos + radius * math.sin(math.radians(self.rot))
            y_ordinate = self.ypos + radius * math.cos(math.radians(self.rot))
            self.rot = (self.rot - speed / 6) % 360
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

#----------------------------
#
#----------------------------
def drawRoadLine(surface):
    for i in range(8):
        pygame.gfxdraw.line(surface, 100 * i, 300, 100 * i + 50, 300, (255, 255, 255))
    pygame.gfxdraw.line(surface, 0, 284, 800, 284, (255, 255, 255))
    pygame.gfxdraw.line(surface, 0, 316, 800, 316, (255, 255, 255))

def main():
    pygame.init()
    screen = pygame.display.set_mode((800, 600))

    clock = pygame.time.Clock()
    
    myVehicle = Vehicle(100, 308, (0, 255, 255))
    obVehicle = Vehicle(200, 308, (255, 255, 0))
    
    background = pygame.Surface(screen.get_size())
    background.fill((0, 0, 0))

    running = True
    while running:
        clock.tick(30)
        for event in pygame.event.get():
            if event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    running = False
            elif event.type == QUIT:
                running = False
        
        # Render background and road line
        screen.blit(background, (0, 0))
        drawRoadLine(screen)

        obVehicle.update_through_steer_and_speed(0, 5)
        myVehicle.update_through_steer_and_speed(0, 10)
        
        # Set 4 control points of bezier curve
        local_dict = {"x":myVehicle.rect.center[0], "y":myVehicle.rect.center[1]}
        ap_dict = {"x":myVehicle.rect.center[0] + 75, "y":myVehicle.rect.center[1] - 16}
        local1_dict = {"x":myVehicle.rect.center[0] + 30, "y":myVehicle.rect.center[1]}
        ap1_dict = {"x":myVehicle.rect.center[0] + 35, "y":myVehicle.rect.center[1] - 16}
        cp = [local_dict, local1_dict, ap1_dict, ap_dict]
        
        # Compute bezier curve
        pathplan = bezier(cp, 3)

        for i in range(len(pathplan)):
            screen.set_at([int(pathplan[i]['x']), int(pathplan[i]['y'])], (255, 255, 255))

        screen.blit(obVehicle.surf, obVehicle.rect)
        screen.blit(myVehicle.surf, myVehicle.rect)
        pygame.display.flip()


if __name__ == "__main__":
    main()
