import sys
from pprint import pprint
from collections import deque
import cProfile
from heapq import heappush, heappop 
import numpy as np

GOAL_STATE = [1, 2, 3, 4, 5, 6, 0, 0, 0] # goal state in array form and 0 represents the blank tile
UP_FORBIDDEN = {0, 1, 2} # set of indices that blank tile cannot move up
RIGHT_FORBIDDEN = {2, 5, 8} # set of indices that blank tile cannot move right
DOWN_FORBIDDEN = {6, 7, 8} # set of indices that blank tile cannot move down
LEFT_FORBIDDEN = {0, 3, 6} # set of indices that blank tile cannot move left
stabilizer = 0 # used to break ties 
nen = 0 # number of expanded nodes
hashDict = {1: (0, 0), 2: (0, 1), 3: (0, 2), 4: (1, 0), 5: (1, 1), 6: (1, 2)} # gives the correct position of tiles as i,j coordinates in the goal state


class Node:
    def __init__(self, state, path, pathCost, parent, stability):
        self.state = state # current state of the puzzle in array form
        self.path = path # "U" for up, "R" for right, "D" for down, "L" for left
        self.pathCost = pathCost # cost of the path
        self.parent = parent # parent node
        self.stability = stability # used to break ties (information of the order of nodes put to the fringe)
    
    """
    This function is used to sort nodes in the fringe(heap).
    First sorted by path cost and heuristic(evaluation function), then by actions, then by order they are put to fringe. 
    """
    def __lt__(self, other):
        if self.evaluationFunction() < other.evaluationFunction():
            return True
        elif self.evaluationFunction() == other.evaluationFunction() and (self.path == "U" or (self.path == "R" and (other.path == "D" or other.path == "L")) or (self.path == "D" and other.path == "L")):
            return True 
        elif self.evaluationFunction() == other.evaluationFunction() and self.path == other.path and self.stability < other.stability:
            return True
    """
    This function calculates the manhattan distance of the current state to the goal state.
    """
    def manhattan(self):
        global hashDict
        temp = self.state[:] 
        temp = np.mat(temp)
        matrixForm = temp.reshape(3,3)
        sum = 0
        for i in range(0, 3):
            for j in range(0, 3):
                if matrixForm[i, j] == 0:
                    continue
                tup = hashDict[matrixForm[i, j]]
                sum += abs(tup[0] - i) + abs(tup[1] - j) 
        return sum
    
    """
    This function calculates the evaluation function of the current state for A*.
    """
    def evaluationFunction(self):
        return self.pathCost + self.manhattan() 


"""
This function is used to perform the graph search algorithm.
"""
def GraphSearch(problem):
    visited = set() # set of visited nodes
    fringe = [] # heap of nodes
    heappush(fringe, problem) # put the initial node to the fringe
    
    while fringe: # while fringe is not empty
        node = heappop(fringe) # pop the node with the lowest evaluation function
        if node.state == GOAL_STATE: # if the current state is the goal state, return the node
            return node
        
        if tuple(node.state) not in visited: # if the current state is not visited, expand the node and add successors to the fringe
            visited.add(tuple(node.state))
            for item in expand(node): 
                heappush(fringe, item)

    return "Fail to find a path!"


"""
This function is used to expand the current node and return successors. 
"""
def expand(node): 
    global stabilizer # used to break ties
    global nen # number of expanded nodes
    successors = [] # list of successors
    ns = node.state # current state
    nc = node.pathCost # current path cost
    idxList = [i for i in range(9) if ns[i] == 0] # list of indices of blank tiles 
    for idx in idxList: # for each blank tile, try to move it up, right, down and left
        if idx not in UP_FORBIDDEN: # if the blank tile can move up, create a new node and add it to the successors list
            ups = ns[:] # copy the current state
            ups[idx], ups[idx - 3] = ups[idx - 3], ups[idx] # swap the blank tile with the tile above it
            nodeUp = Node(ups, "U", nc + 1, node, stabilizer) # create a new node with the new state
            successors.append(nodeUp) # add the new node to the successors list
            nen += 1
            stabilizer += 1
        if idx not in RIGHT_FORBIDDEN: # if the blank tile can move right, create a new node and add it to the successors list
            rights = ns[:]
            rights[idx], rights[idx + 1] = rights[idx + 1], rights[idx]
            nodeRight = Node(rights, "R", nc + 1, node, stabilizer)
            successors.append(nodeRight)
            nen += 1
            stabilizer += 1
        if idx not in DOWN_FORBIDDEN: # if the blank tile can move down, create a new node and add it to the successors list
            downs = ns[:]
            downs[idx], downs[idx + 3] = downs[idx + 3], downs[idx]
            nodeDown = Node(downs, "D", nc + 1, node, stabilizer)
            successors.append(nodeDown)
            nen += 1
            stabilizer += 1
        if idx not in LEFT_FORBIDDEN: # if the blank tile can move left, create a new node and add it to the successors list
            lefts = ns[:]
            lefts[idx], lefts[idx - 1] = lefts[idx - 1], lefts[idx]
            nodeLeft = Node(lefts, "L", nc + 1, node, stabilizer) 
            successors.append(nodeLeft)  
            nen += 1
            stabilizer += 1
    return successors # return the successors list

# __main__
with open(sys.argv[1], "r") as f: # read the input file
    contents = f.read()

puzzle = [int(val) for val in contents.split()] # initial state of the puzzle in array form
initialNode = Node(puzzle, "", 0, None, 0)  # initial node

"""
This part is used to write the output to the output file.
"""
file = open(sys.argv[2], 'w') # open the output file
result = GraphSearch(initialNode)
pathCost = result.pathCost
file.write(f"Number of expanded nodes: {str(nen)}")
file.write('\n')
file.write(f"Path-cost: {str(pathCost)}")
file.write('\n')
path = deque()
while result.parent != None:
    path.appendleft(result.state) 
    result = result.parent
for item in path:
    temp = item[:] 
    temp = np.mat(temp)
    matrixForm = temp.reshape(3,3)
    file.write(str(matrixForm))  
    file.write('\n')
    file.write('\n')
file.close()