#!/usr/bin/env python

from collections import deque
from queue import PriorityQueue
import heapq
import numpy as np
import math
import random as rd
import pygame
import time


class Node:
    def __init__(self, state=None, cost=0.0, costToCome=0.0, parent=None):
        self.state = state
        self.parent = parent
        self.cost = cost
        self.costToCome = costToCome
        
        
# Workspace
def isValidWorkspace(pt, r, radiusClearance):
    x, y = pt

    # For Circle 1 with center at (2, 8) and radius 1 unit 
    ptInCircle1 = (x - math.floor(2 / r)) ** 2 + (y - math.floor(2 / r)) ** 2 - ((1 + radiusClearance) / r) ** 2 <= 0

    # For Circle 2 with center at (2, 2) and radius 1 unit
    ptInCircle2 = (x - math.floor(2 / r)) ** 2 + (y - math.floor(8 / r)) ** 2 - ((1 + radiusClearance) / r) ** 2 <= 0

    # For Square 1 with center at (1, 5) and dimension 1.5x1.5
    X = np.float32([0.25, 1.75, 1.75, 0.25]) / r
    Y = np.float32([4.25, 4.25, 5.75, 5.75]) / r
    ptInSquare1 = Y[0] - radiusClearance / r <= y <= Y[2] + radiusClearance / r and \
                  0 >= (Y[2] - Y[1]) * (x - X[1]) - radiusClearance / r and \
                  0 >= (Y[0] - Y[3]) * (x - X[3]) - radiusClearance / r

    # For Rectangle 1 with center at (5, 5) and dimension 2.5x1.5
    X = np.float32([3.75, 6.25, 6.25, 3.75]) / r
    Y = np.float32([4.25, 4.25, 5.75, 5.75]) / r
    ptInRectangle1 = Y[0] - radiusClearance / r <= y <= Y[2] + radiusClearance / r and \
                  0 >= (Y[2] - Y[1]) * (x - X[1]) - radiusClearance / r and \
                  0 >= (Y[0] - Y[3]) * (x - X[3]) - radiusClearance / r

    # For Rectangle 2 with center at (8, 3) and dimension 1.5x2.0
    X = np.float32([7.25, 8.75, 8.75, 7.25]) / r
    Y = np.float32([6.0, 6.0, 8.0, 8.0]) / r
    ptInRectangle2 = Y[0] - radiusClearance / r <= y <= Y[2] + radiusClearance / r and \
                  0 >= (Y[2] - Y[1]) * (x - X[1]) - radiusClearance / r and \
                  0 >= (Y[0] - Y[3]) * (x - X[3]) - radiusClearance / r

    if ptInCircle1 or ptInCircle2 or ptInSquare1 or ptInRectangle1 or ptInRectangle2:
        return False
    
    else:
        return True


# checks whether next action is near an obstacle or ill defined
def isSafe(newState, r, radiusClearance):
    col = float(10 / r)
    row = float(10 / r)

    if newState[0] < 0.0 or newState[0] > col or newState[1] < 0.0 or newState[1] > row:
        return False
    
    else:
        return isValidWorkspace(newState, r, radiusClearance)


def steer(xNearest, xRand):
    stepsize = 0.1
    dist = distance(xNearest,xRand) 
    if (dist<stepsize):
        return xRand
    else:
        t = stepsize/dist
        v = xRand - xNearest
        r = t*v + xNearest
        return r


def isObstacleFree(pt1, pt2, radiusClearance):
    stepsize = 0.1
    t = np.arange(stepsize, 1.0 + stepsize, stepsize)
    v = pt2 - pt1
    for i in range(len(t)):
        r = t[i] * v + pt1
        if not isSafe(r, 1, radiusClearance):
            return False
    return True


def printPath(node):
    solution = []
    current = node
    while current:
        solution.append(current.state)
        current = current.parent
    return solution

def samplePoint():
    x = rd.uniform(0.0, 10.0)
    y = rd.uniform(0.0, 10.0)
    return [x, y]

def distance(startPosition, goalPosition):
    sx, sy = startPosition
    gx, gy = goalPosition
    return math.sqrt((gx - sx) ** 2 + (gy - sy) ** 2)


def nearest(nodesExplored, newState):
    minDist = np.inf
    for key, node in nodesExplored.items():
        dist = distance(node.state, newState)
        if dist < minDist:
            minDist = dist
            minKey = key
    return minKey, minDist


def rewiring(bestNeighbour, neighbours, radiusClearance):
    for node in neighbours:
        if not np.array_equal(node.state, bestNeighbour.state) and node.parent != node:
            if isObstacleFree(node.state, bestNeighbour.state, radiusClearance) and \
                    node.costToCome > bestNeighbour.costToCome + distance(node.state, bestNeighbour.state):
                node.parent = bestNeighbour
                node.cost = distance(node.state, bestNeighbour.state)
                node.costToCome = bestNeighbour.costToCome + node.cost


def findNeighbors(NodesExplored, radiusClearance, newNode, radius=2.0):
    neighbours = []
    newX, newY = newNode.state
    xBest = newNode

    for key, node in NodesExplored.items():
        posX, posY = node.state
        if (node.state != newNode.state).all() and isObstacleFree(node.state, newNode.state, radiusClearance):
            if (newX - posX) ** 2 + (newY - posY) ** 2.0 - radius ** 2 <= 0:
                neighbours.append(node)
                # Finding the best neighbour
                tempCost = node.costToCome + distance(node.state, newNode.state)
                if tempCost < newNode.costToCome:
                    xBest = node

    return xBest, neighbours


# generates optimal path for robot
def generatePath(q, startEndCoor, nodesExplored, radiusClearance, numIterations=3000):
    # get start and goal locations
    sx, sy = startEndCoor[0]
    gx, gy = startEndCoor[1]

    # Initializing root node
    key = str(sx) + str(sy)
    root = Node(np.float32([sx, sy]), 0.0, 0.0, None)
    nodesExplored[key] = root

    # for i in range(numIterations):

    while True:
        # sample random point
        newPosX, newPosY = samplePoint()
        xRand = np.array([newPosX, newPosY])

        # Get Nearest Node
        xNearestKey, xNearestDist = nearest(nodesExplored, xRand)
        xNearest = nodesExplored[xNearestKey].state

        # steer in direction of path
        xNew = steer(xNearest, xRand)

        # check if edge is not in obstacle
        if (xNew == xNearest).all() or not isObstacleFree(xNearest, xNew, radiusClearance):
            continue

        xNewCost = distance(xNew, xNearest)
        xNewcostToCome = nodesExplored[xNearestKey].costToCome + xNewCost
        newNode = Node(xNew, xNewCost, xNewcostToCome, nodesExplored[xNearestKey])
        s = str(newNode.state[0]) + str(newNode.state[1])
        nodesExplored[s] = newNode

        bestNeighbour, neighbours = findNeighbors(nodesExplored, radiusClearance, newNode)

        if (newNode.state == bestNeighbour.state).all():
            continue
        newNode.cost = distance(xNew, bestNeighbour.state)
        newNode.costToCome = bestNeighbour.costToCome + xNewCost
        newNode.parent = bestNeighbour

        rewiring(bestNeighbour, neighbours, radiusClearance)

        if distance(newNode.state, [gx, gy]) <= 0.5:
            sol = printPath(newNode)
            return [True, sol]
        
    return [False, None]


def triangleCoordinates(start, end, triangleSize=5):

    rotation = (math.atan2(start[1] - end[1], end[0] - start[0])) + math.pi/2
    rad = math.pi/180

    coordinateList = np.array([[end[0],end[1]],
                              [end[0] + triangleSize * math.sin(rotation - 165*rad), end[1] + triangleSize * math.cos(rotation - 165*rad)],
                              [end[0] + triangleSize * math.sin(rotation + 165*rad), end[1] + triangleSize * math.cos(rotation + 165*rad)]])

    return coordinateList


if __name__ == "__main__":

    is1 = -4  
    is2 = -4  
    ig1 = 4  
    ig2 = 4  

    # Inputs From World Coordinates to Pygame Coordinates
    s1 = 5 + (is1)
    s2 = 5 - (is2)
    g1 = 5 + (ig1)
    g2 = 5 - (ig2)

    threshDistance = 0.1
    clearance = 0.4
    robotRadius = 0  

    pygame.init()

    res = 1.0  # resolution of grid
    scale = 80  # scale of grid

    white = (255, 255, 255)
    black = (0, 0, 0)
    red = (255, 0, 0)
    green = (0, 255, 0)
    blue = (0, 0, 255)
    yellow = (255, 255, 0)

    size_x = 10
    size_y = 10
    gameDisplay = pygame.display.set_mode((size_x * scale, size_y * scale))

    # Start and goal coordinates
    startCoor = np.float32((np.float32([s1, s2])) / res)
    goalCoor = np.float32((np.float32([g1, g2])) / res)

    startEndCoor = [startCoor, goalCoor]

    # Display Obstacles
    pygame.draw.circle(gameDisplay, red, (2 * scale, 2 * scale), 1 * scale)
    pygame.draw.circle(gameDisplay, red, (2 * scale, 8 * scale), 1 * scale)
    pygame.draw.rect(gameDisplay, red, [int(scale * 0.25), int(scale * 4.25), int(scale * 1.5), int(scale * 1.5)])
    pygame.draw.rect(gameDisplay, red, [int(scale * 3.75), int(scale * 4.25), int(scale * 2.5), int(scale * 1.5)])
    pygame.draw.rect(gameDisplay, red, [int(scale * 7.25), int(scale * 6.0), int(scale * 1.5), int(scale * 2.0)])

    # Draw Explored Nodes and solution path
    nodesExplored = {}
    q = []

    if not isSafe(startCoor, res, clearance + robotRadius) or not isSafe(goalCoor, res, clearance + robotRadius):
        pygame.draw.rect(gameDisplay, blue, (startCoor[0] * res * scale, startCoor[1] * res * scale,
                                             res * 2, res * 2))

        pygame.draw.circle(gameDisplay, blue, (int(goalCoor[0] * res * scale), int(goalCoor[1] * res * scale)),
                           math.floor(0.3 * res * scale))

        pygame.draw.rect(gameDisplay, white, (goalCoor[0] * res * scale, goalCoor[1] * res * scale,
                                              res * 2, res * 2))
        basicfont = pygame.font.SysFont(None, 48)
        text = basicfont.render('Start or goal position must be in a valid workspace', True, (255, 0, 0),
                                (255, 255, 255))
        textrect = text.get_rect()
        textrect.centerx = gameDisplay.get_rect().centerx
        textrect.centery = gameDisplay.get_rect().centery

        gameDisplay.blit(text, textrect)
        pygame.display.update()
        pygame.time.delay(2000)

    else:
        startTime = time.time()  # Start time of simulation
        print('Exploring nodes...')
        success, solution = generatePath(q, startEndCoor, nodesExplored, clearance + robotRadius)
        endTime = time.time()

        # Drawing
        if success:
            print('Optimal path found')
            print("Total time taken for exploring nodes " + str(endTime - startTime) + " seconds.")

            draw = True
            while draw:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        pygame.quit()
                        quit()

                # draw nodesExplored
                for s in nodesExplored:
                    if nodesExplored[s].parent:
                        pt = nodesExplored[s].state[0:2]
                        ptParent = nodesExplored[s].parent.state[0:2]
                        x, y = pt * scale * res
                        x2, y2 = ptParent * scale * res

                        # draw explored nodes
                        pygame.draw.line(gameDisplay, white, (x2, y2), (x, y), 1)
                        # pygame.draw.circle(gameDisplay,green,(int(x),int(y)),4)
                        triangle = triangleCoordinates([x2, y2], [x, y], 5)
                        pygame.draw.polygon(gameDisplay, green, [tuple(triangle[0]), tuple(triangle[1]), tuple(triangle[2])])

                    # draw start and goal locations
                    pygame.draw.rect(gameDisplay, blue, (startCoor[0] * res * scale, startCoor[1] * res * scale, res * 2, res * 2))
                    pygame.draw.circle(gameDisplay, blue, (int(goalCoor[0] * res * scale), int(goalCoor[1] * res * scale)), math.floor(0.3 * res * scale))
                    pygame.draw.rect(gameDisplay, white, (goalCoor[0] * res * scale, goalCoor[1] * res * scale, res * 2, res * 2))
                    pygame.display.update()

                # draw solution path
                for i in range(len(solution) - 2, -1, -1):
                    pt = solution[i][0:2]
                    pt1 = solution[i + 1][0:2]
                    xt, yt = pt[0] * scale * res, pt[1] * scale * res
                    x, y = pt1[0] * scale * res, pt1[1] * scale * res
                    pygame.draw.line(gameDisplay, yellow, (xt, yt), (x, y), 3)
                    pygame.draw.circle(gameDisplay, red, (int(x), int(y)), 4)
                    pygame.display.update()
                pygame.time.delay(4000)
                draw = False
                
                # writing solution path for simulation in gazebo
                control_data = np.array(solution)
                control_data[:, 0] = -5 + control_data[:, 0]
                control_data[:, 1] = 5 - control_data[:, 1]
                
                for i in range(2):
                    control_data[:, i] = control_data[:, i][::-1]
                np.savetxt("rrt_star_controls.txt", control_data, delimiter=',')

        else:
            print('Path not possible')
            basicfont = pygame.font.SysFont(None, 48)
            text = basicfont.render('Path can\'t be generated', True, (255, 0, 0), (255, 255, 255))
            textrect = text.get_rect()
            textrect.centerx = gameDisplay.get_rect().centerx
            textrect.centery = gameDisplay.get_rect().centery
            gameDisplay.blit(text, textrect)
            pygame.display.update()
            pygame.time.delay(2000)

    pygame.quit()

