import cv2
import numpy as np
from sklearn.preprocessing import normalize


class particle:
    def __init__(self, xStart, yStart, velocity, size=3):
        self.xPos = xStart
        self.yPos = yStart
        self.pheromoneType = np.random.randint(0, 3)
        self.velocity = velocity
        self.xVelocity = int(velocity/2)
        self.yVelocity = int(velocity/2)
        self.size = size

    def updatePosition(self, arena):
        self.xPos += self.xVelocity
        self.yPos += self.yVelocity
        self.checkBoundingBox(arena)
        self.normalizeSpeed()

    def returnPos(self):
        return [int(self.xPos), int(self.yPos)]

    def checkBoundingBox(self, arena):
        if self.xPos <= 0:
            self.xPos = 0
            self.xVelocity = self.xVelocity * -1.1
        elif self.xPos >= arena.mapSize:
            self.xPos = arena.mapSize-1
            self.xVelocity = self.xVelocity * -1.1
        if self.yPos <= 0:
            self.yPos = 0
            self.yVelocity = self.yVelocity * -1.1
        elif self.yPos >= arena.mapSize:
            self.yPos = arena.mapSize-1
            self.yVelocity = self.yVelocity * -1.1

    def normalizeSpeed(self):
        vector = np.array([[self.xVelocity, self.yVelocity]])
        normalised = self.velocity * normalize(vector, axis=1, norm='l1')
        self.xVelocity, self.yVelocity = normalised[0]



class arena:
    def __init__(self, size, damping=0.97):
        self.mapSize = size
        self.activeArena = np.zeros((size, size, 3))
        self.dampingFactor = damping

    def showArena(self):
        return self.activeArena

    def addTrail(self, ID, xpos, ypos):
        pass

    def drawAgent(self, agent):
        pass

    def blurTrail(self):
        self.activeArena = self.activeArena * self.dampingFactor

if __name__ == '__main__':

    newArena = arena(600)

    particles = []
    for i in range(4):

        particles.append(particle(
            xStart=np.random.randint(0, newArena.mapSize),
            yStart=np.random.randint(0, newArena.mapSize),
            velocity=4
            )
        )

    while True:

        newArena.blurTrail()
        newArena.activeArena = cv2.GaussianBlur(newArena.activeArena, (3, 3), 0)
        #move agents and draw them
        for agent in particles:
            agent.updatePosition(newArena)
            pheromones = [0,0,0]
            pheromones[agent.pheromoneType] = 255
            newArena.activeArena[int(agent.xPos), int(agent.yPos)] +=pheromones

        #show the arena
        cv2.imshow('main', newArena.showArena())

        k = cv2.waitKey(1)
        if k == ord('q'):
            cv2.destroyAllWindows()
            break
        elif k == ord('a'):
            particles.append(particle(
                xStart=np.random.randint(0, newArena.mapSize),
                yStart=np.random.randint(0, newArena.mapSize),
                velocity=8)
            )