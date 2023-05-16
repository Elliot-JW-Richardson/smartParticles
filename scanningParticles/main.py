import cv2
import numpy as np
from sklearn.preprocessing import normalize, minmax_scale
import random
import time

class particle:
    def __init__(self, xStart, yStart, velocity, pheromoneType, behaviour, size=3):
        self.xPos = xStart
        self.yPos = yStart
        self.pheromoneType = pheromoneType
        self.behaviour = behaviour
        self.velocity = velocity
        self.direction = self.changeDirection(direction=np.array([0.0, 1.1]), inputAngle=random.randrange(0, 360, 5))
        self.size = size
        self.wanderStrength = 0.20
        self.lookDistance = 40
        self.maxLookAngle = 50
        self.sampleVectors = self.generateVectors()
        self.trapped = 1

    def changeDirection(self, direction, inputAngle):
        # define a rotation matrix to adjust the input direction
        theta = np.deg2rad(inputAngle)
        rotationMatrix = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
        # rotate the vector according to the angle

        newDirection = np.dot(rotationMatrix, direction)
        return newDirection.round(4)

    def wander(self, arena, specificDirection=False):
        maxHeight, maxWidth = arena.maxHeight, arena.maxWidth
        movement = np.random.random()
        if specificDirection is not False:
            self.direction = specificDirection
        elif movement <= self.wanderStrength:
            angle = random.choices(np.arange(-self.maxLookAngle, self.maxLookAngle+5, 5))[0]
            try:
                self.direction = self.changeDirection(self.direction, inputAngle=angle)
                self.updateSensors(angle)
            except ValueError:
                print('I am erroring')
                print(angle, self.direction)
        self.yPos, self.xPos = (self.yPos, self.xPos) + (self.trapped * self.velocity * self.direction)
        self.xPos = self.xPos % maxWidth
        self.yPos = self.yPos % maxHeight
        self.checkLocation(arena)

    def updateSensors(self, angle):
        for num, vector in enumerate(self.sensorVectors):
            self.sensorVectors[num] = self.changeDirection(vector, angle)

    def generateVectors(self):
        self.sensorVectors = []
        angles = np.arange(-self.maxLookAngle, self.maxLookAngle+25, 25)
        for angle in angles:
            self.sensorVectors.append(self.changeDirection(self.direction, angle))

    def returnPos(self, numpyType = False, typeInt=False):
        if typeInt:
            x, y = (int(self.xPos), int(self.yPos))
        else:
            x, y = self.xPos, self.yPos
        if numpyType:
            return (y, x)
        return (x, y)

    def checkLocation(self, arena):
        x, y = self.returnPos(numpyType=True, typeInt=True)
        if np.array_equal(arena.activeArena[y, x], np.array([255, 255, 255])):
            self.direction = self.changeDirection(self.direction, inputAngle=180)
            self.updateSensors(180)
            self.trapped += 1
        else:
            self.trapped = 1

    def scanAhead(self, image, direction):
        maxHeight, maxWidth = image.maxHeight, image.maxWidth
        visited = []
        currentPos = self.returnPos(numpyType=True)
        currentPos += 3 * direction
        totalSum = 0,0,0
        for i in range(1, self.maxLookAngle):
            yPos = int(currentPos[0]) % maxHeight
            xPos = int(currentPos[1]) % maxWidth
            if [yPos, xPos] not in visited:
                totalSum += image.activeArena[yPos, xPos]
                visited.append([yPos, xPos])
            currentPos += direction
        return np.round(totalSum, 4)

class arena:
    def __init__(self, height, width, damping=0.94):
        self.maxHeight = height
        self.maxWidth = width
        self.activeArena = np.zeros((self.maxHeight, self.maxWidth, 3))
        self.dampingFactor = damping

    def showArena(self):
        return self.activeArena

    def addTrail(self, ID, xpos, ypos):
        pass

    def drawAgent(self, agent):
        pass

    def blurTrail(self):
        self.activeArena = self.activeArena * self.dampingFactor

    def clearArea(self):
        self.activeArena = np.zeros((self.maxHeight, self.maxWidth, 3))

if __name__ == '__main__':

    newArena = arena(1000, 1000)
    numParticles = 100
    particles = []
    pheromones = [0, 1, 2]
    behaviours = [(1, -1, -1),
                  (-1, 1, -1),
                  (-1, -1, 1)]

    #destinations = [(100, 100), (700, 700), (400, 400), (100, 700), (700, 100)]
    destinations = []

    for i in range(numParticles):
        type = np.random.randint(0, len(pheromones))
        particles.append(particle(
            xStart=np.random.randint(400, 600),
            yStart=np.random.randint(400, 600),
            velocity=np.random.randint(1, 4),
            pheromoneType=pheromones[type],
            behaviour=behaviours[type]
            )
        )

    while True:


        newArena.blurTrail()
        newArena.activeArena = cv2.GaussianBlur(newArena.activeArena, (3, 3), 0)
        #newArena.clearArea()

        for circle in destinations:
            cv2.circle(newArena.activeArena, circle, 25, (255, 255, 255), -1)

        #move agents and draw them

        for agent in particles:
            turningScores = []
            for vector in agent.sensorVectors:
                score = sum(agent.scanAhead(newArena, vector) * agent.behaviour)
                turningScores.append(score)
            if any(score > 50 for score in turningScores):
                minscore = (abs(min(turningScores)))
                turningScores = [x + minscore for x in turningScores]
                newDirection = random.choices(agent.sensorVectors, weights=turningScores, k=1)
                agent.wander(newArena, newDirection[0])
            else:
                agent.wander(newArena)
            pheromones = [0, 0, 0]
            pheromones[agent.pheromoneType] = 255
            newArena.activeArena[int(agent.yPos), int(agent.xPos)] += pheromones

        #show the arena
        cv2.imshow('main', newArena.showArena())

        k = cv2.waitKey(10)
        if k == ord('q'):
            cv2.destroyAllWindows()
            break
        elif k == ord('a'):
            type = np.random.randint(0, len(pheromones))
            particles.append(particle(
                xStart=np.random.randint(400, 600),
                yStart=np.random.randint(400, 600),
                velocity=np.random.randint(1, 4),
                pheromoneType=pheromones[type],
                behaviour=behaviours[type]
            )
            )