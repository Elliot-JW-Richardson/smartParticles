import numpy as np
import cv2
import random
import time
import imageio

class particle:
    def __init__(self, x, y, color, destinations, velocity):
        self.xPos = x
        self.yPos = y
        self.color = color
        self.velocity = velocity
        self.direction = self.changeDirection(np.array([0.0, 1.0]), random.randrange(0, 360, 5))
        self.listenRange = 75
        self.neighbours = []
        self.wanderStrength = 0.15
        self.destinations = destinations
        self.distanceEstimates = [0] * len(destinations)
        self.targetDestination = random.choices(self.destinations)[0]
        self.wanderAngle = 10
        self.trapped = 1

    def changeDirection(self, direction, inputAngle):
        # define a rotation matrix to adjust the input direction
        theta = np.deg2rad(inputAngle)
        rotationMatrix = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
        # rotate the vector according to the angle
        newDirection = np.dot(rotationMatrix, direction)
        return newDirection.round(4)

    def returnPos(self, isInt=False):
        if isInt:
            x, y = int(self.xPos), int(self.yPos)
        else:
            x, y = self.xPos, self.yPos
        return y, x

    def returnColor(self):
        return self.color

    def returnEstimates(self):
        return self.distanceEstimates

    def move(self, particleList, arena):
        maxHeight, maxWidth = arena.shape[:2]
        if np.random.random() < self.wanderStrength:
            angle = random.choices(np.arange(-self.wanderAngle, self.wanderAngle + 5, 5))[0]
            self.direction = self.changeDirection(self.direction, angle)
        self.yPos, self.xPos = (self.yPos, self.xPos) + (self.trapped * self.velocity * self.direction)
        self.bordercheck(maxWidth, maxHeight)
        self.findNeighbours(particleList)
        self.checkDestination()
        self.listen()
        self.wanderStrength += 0.01
        self.wanderStrength = max(0.05, self.wanderStrength)
        self.distanceEstimates = [x+1 for x in self.distanceEstimates]

    def bordercheck(self, maxWidth, maxHeight):
        if self.xPos > maxWidth:
            self.xPos = maxWidth
            self.direction = self.changeDirection(self.direction, 180)
        elif self.xPos < 0:
            self.xPos = 0
            self.direction = self.changeDirection(self.direction, 180)
        if self.xPos > maxHeight:
            self.xPos = maxHeight
            self.direction = self.changeDirection(self.direction, 180)
        elif self.yPos < 0:
            self.yPos = 0
            self.direction = self.changeDirection(self.direction, 180)

    def checkDestination(self):
        isTrapped = False
        for num, spot in enumerate(destinations):
            destinationName = spot[4]
            destinationSize = np.square(spot[3])
            destinationY, destinationX = spot[:2]
            destinationColor = spot[2]
            dist = np.square(destinationY - self.yPos) + np.square(destinationX - self.xPos)
            if dist <= destinationSize:
                self.direction = self.changeDirection(self.direction, 180)
                self.distanceEstimates[num] = 0
                isTrapped = True
                if self.targetDestination[4] == destinationName:
                    self.color = tuple(elem * 0.5 for elem in destinationColor)
                    self.targetDestination = random.choice([i for i in self.destinations if i not in [self.targetDestination]])
        if isTrapped:
            self.trapped += 1
        else:
            self.trapped = 1

    def listen(self):
        for neighbour in self.neighbours:
            listenPos = self.destinations.index(self.targetDestination)
            if neighbour.returnEstimates()[listenPos] < self.distanceEstimates[listenPos]:
                self.distanceEstimates[listenPos] = neighbour.returnEstimates()[listenPos]
                targetY, targetX = neighbour.returnPos()
                vector = np.array([targetY-self.yPos, targetX-self.xPos])
                self.direction = vector / np.linalg.norm(vector)
                self.wanderStrength = 0

    def findNeighbours(self, neighbourList):
        self.neighbours = []
        neighbourPositions = np.array([particle.returnPos() for particle in neighbourList])
        neighbourY, neighbourX = neighbourPositions.T
        distSquared = (neighbourY - self.yPos) ** 2 + (neighbourX - self.xPos) ** 2
        listenRangeSquared = self.listenRange ** 2
        self.neighbours = [particle for particle, dist in zip(neighbourList, distSquared) if (dist < listenRangeSquared)]


class destination:
    def __init__(self, x, y, color, velocity):
        self.xPos = x
        self.yPos = y
        self.color = color
        self.velocity = velocity
        self.direction = self.changeDirection(np.array([0.0, 1.0]), random.randrange(0, 360, 5))
        self.wanderStrength = 0.25
        self.distanceEstimates = [0] * len(destinations)
        self.wanderAngle = 10

    def changeDirection(self, direction, inputAngle):
        # define a rotation matrix to adjust the input direction
        theta = np.deg2rad(inputAngle)
        rotationMatrix = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
        # rotate the vector according to the angle
        newDirection = np.dot(rotationMatrix, direction)
        return newDirection.round(4)

    def returnPos(self, isInt=False):
        if isInt:
            x, y = int(self.xPos), int(self.yPos)
        else:
            x, y = self.xPos, self.yPos
        return y, x

    def returnColor(self):
        return self.color

    def move(self, arena):
        if np.random.random() < self.wanderStrength:
            angle = random.choices(np.arange(-self.wanderAngle, self.wanderAngle + 5, 5))[0]
            self.direction = self.changeDirection(self.direction, angle)
        self.yPos, self.xPos = (self.yPos, self.xPos) + (self.velocity * self.direction)
        self.bordercheck()
        self.wanderStrength += 0.01
        self.wanderStrength = max(0.25, self.wanderStrength)

    def bordercheck(self, arena):
        maxHeight, maxWidth = arena.shape[:2]
        if self.xPos > maxWidth:
            self.xPos = maxWidth
            self.direction = self.changeDirection(self.direction, 180)
        elif self.xPos < 0:
            self.xPos = 0
            self.direction = self.changeDirection(self.direction, 180)
        if self.xPos > maxHeight:
            self.xPos = maxHeight
            self.direction = self.changeDirection(self.direction, 180)
        elif self.yPos < 0:
            self.yPos = 0
            self.direction = self.changeDirection(self.direction, 180)


# Define the output video file name
output_file = 'output_video.mp4'

# Create a VideoWriter object using imageio
video_writer = imageio.get_writer(output_file, fps=30)

numParticles = 200
height = 512
width = 512

#colors listed in b,g,r according to cv2
targets = [[100, 100, (1, 0, 0), 20, 'A'],
           [400, 400, (0, 0, 1), 20, 'B']]

destinations = []
particles = []
image = np.zeros([height, width, 4])

for i in targets:
    destinations.append(i)

for i in range(numParticles):
    x = np.random.randint(0, width)
    y = np.random.randint(0, height)
    color = (0, 0.5, 0.5)
    velocity = np.random.randint(1, 3)
    particles.append((particle(x, y, color, destinations, velocity)))

print(len(particles))


while True:
    image = np.zeros([height, width, 4])

    for spot in destinations:
        ypos, xpos = spot[:2]
        color = spot[2]
        size = spot[3]
        cv2.circle(image, (xpos, ypos), size, color, -1)

    for particle in particles:
        particle.move(particles, image)
        xpos, ypos = particle.returnPos(True)
        color = particle.returnColor()
        size = 2
        cv2.circle(image, (xpos, ypos), size, color, -1)

    cv2.imshow('thing', image)
    video_writer.append_data((255 * image[...,[2,1,0]]).astype(np.uint8))
    k= cv2.waitKey(1)

    if k == ord('q'):
        cv2.destroyAllWindows()
        video_writer.close()
        break