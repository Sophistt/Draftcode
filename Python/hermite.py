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
        pygame.gfxdraw.aacircle(self.surf, 15, 15, 14, (0, 255, 0))


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
    

    ap_point = Circle()
    ap_point.rect.center = (350, 200)
    
    local_point = Circle()
    local_point.rect.center = (400, 300)
    
    # Set 4 control points of bezier curve
    ap_dict = {"x":350, "y":200}
    ap1_dict = {"x":350, "y":240}
    local1_dict = {"x":400, "y":250}
    local_dict = {"x":400, "y":300}
    cp = [local_dict, local1_dict, ap1_dict, ap_dict]
    
    # Compute bezier curve
    pathplan = bezier(cp, 3)
    print(pathplan)

    running = True
    while running:
        clock.tick(60)
        for event in pygame.event.get():
            if event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    running = False
            elif event.type == QUIT:
                running = False
        
        for i in range(len(pathplan)):
            screen.set_at([int(pathplan[i]['x']), int(pathplan[i]['y'])], (255, 255, 255))


        screen.blit(ap_point.surf, ap_point.rect)
        screen.blit(local_point.surf, local_point.rect)
        pygame.display.flip()



if __name__ == "__main__":
    main()
