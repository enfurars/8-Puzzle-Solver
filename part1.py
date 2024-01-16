import sys
from pprint import pprint
from collections import deque
import cProfile
from heapq import heappush, heappop 
import numpy as np

GOAL_STATE = [1, 2, 3, 4, 5, 6, 7, 8, 0] # goal state in array form and 0 represents the blank tile
UP_FORBIDDEN = {0, 1, 2} # set of indices that blank tile cannot move up
RIGHT_FORBIDDEN = {2, 5, 8} # set of indices that blank tile cannot move right
DOWN_FORBIDDEN = {6, 7, 8} # set of indices that blank tile cannot move down
LEFT_FORBIDDEN = {0, 3, 6} # set of indices that blank tile cannot move left 
stabilizer = 0 # used to break ties
nen = 0 # number of expanded nodes
hashDict = {1: (0, 0), 2: (0, 1), 3: (0, 2), 4: (1, 0), 5: (1, 1), 6: (1, 2), 7: (2, 0), 8: (2, 1)} # gives the correct position of tiles as i,j coordinates in the goal state


class Node:
    def __init__(self, state, path, pathCost, idxBlank, parent, stability, searchType):
        self.state = state # current state of the puzzle in array form
        self.path = path # "U" for up, "R" for right, "D" for down, "L" for left
        self.pathCost = pathCost # cost of the path
        self.idxBlank = idxBlank # index of the blank tile
        self.parent = parent # parent node
        self.stability = stability # used to break ties (information of the order of nodes put to the fringe)
        self.searchType = searchType # search type (BFS, DFS, UCS, Greedy, A*) 
    
    """
    This function is used to sort nodes in the fringe(heap).
    First sorted by path cost, then by actions, then by order they are put to fringe. Includes modification for Greedy and A* for the path cost part.
    """
    def __lt__(self, other):
        match self.searchType:
            case "UCS":
                if self.pathCost < other.pathCost: 
                    return True
                elif self.pathCost == other.pathCost and (self.path == "U" or (self.path == "R" and (other.path == "D" or other.path == "L")) or (self.path == "D" and other.path == "L")): 
                    return True 
                elif self.pathCost == other.pathCost and self.path == other.path and self.stability < other.stability: 
                    return True
            case "Greedy":
                if self.manhattan() < other.manhattan(): 
                    return True
                elif self.manhattan() == other.manhattan() and (self.path == "U" or (self.path == "R" and (other.path == "D" or other.path == "L")) or (self.path == "D" and other.path == "L")):  
                    return True
                elif self.manhattan() == other.manhattan() and self.path == other.path and self.stability < other.stability:
                    return True 
            case "A*":
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
def GraphSearch(problem, srcType):
    problem.searchType = srcType # set the search type of the initial node
    visited = set() # set of visited nodes
    if srcType == "BFS" or srcType == "DFS": # fringe is a queue for BFS and DFS
        fringe = deque([])
        fringe.append(problem)
    else: # fringe is a heap for UCS, Greedy and A*
        fringe = [] 
        heappush(fringe, problem) 
    
    while fringe:
        match srcType:
            case "DFS":
                node = fringe.pop() # pop the last element of the queue for DFS
            case "BFS":
                node = fringe.popleft() # pop the first element of the queue for BFS
            case "UCS":
                node = heappop(fringe) # pop the first element of the heap for UCS
            case "Greedy":
                node = heappop(fringe) # pop the first element of the heap for Greedy
            case "A*":
                node = heappop(fringe) # pop the first element of the heap for A*
        if node.state == GOAL_STATE: # if the current state is the goal state, return the current node
            return node
        
        if tuple(node.state) not in visited: # if the current state is not visited, add it to the visited set and expand it
            visited.add(tuple(node.state))
            match srcType:
                case "DFS":
                    successors = expand(node, srcType) 
                    successors.reverse() # reverse the successors for DFS
                    fringe.extend(successors) # add the successors to the queue
                case "BFS":
                    fringe.extend(expand(node, srcType)) # add the successors to the queue
                case "UCS":
                    for item in expand(node, srcType): 
                        heappush(fringe, item)  # add the successors to the heap
                case "Greedy": 
                    for item in expand(node, srcType): 
                        heappush(fringe, item)  # add the successors to the heap
                case "A*":
                    for item in expand(node, srcType):  
                        heappush(fringe, item)  # add the successors to the heap

    return "Fail to find a path!" 

"""
This function is used to expand the current node.
"""
def expand(node, srcType): 
    global nen # number of expanded nodes
    global stabilizer # used to break ties
    successors = [] # list of successors
    ns = node.state # current state
    nc = node.pathCost # current path cost
    idx = node.idxBlank # index of the blank tile
    if idx not in UP_FORBIDDEN: # if the blank tile can move up, create a new node with the new state and add it to the successors list
        ups = ns[:] # copy the current state
        ups[idx], ups[idx - 3] = ups[idx - 3], ups[idx] # swap the blank tile with the tile above it
        nodeUp = Node(ups, "U", nc + 1, idx - 3, node, stabilizer, srcType) # create a new node with the new state
        successors.append(nodeUp) # add the new node to the successors list
        nen += 1 # increase the number of expanded nodes
        stabilizer += 1 # increase the stabilizer 
    if idx not in RIGHT_FORBIDDEN: # if the blank tile can move right, create a new node with the new state and add it to the successors list
        rights = ns[:]
        rights[idx], rights[idx + 1] = rights[idx + 1], rights[idx]
        nodeRight = Node(rights, "R", nc + 1, idx + 1, node, stabilizer, srcType)
        successors.append(nodeRight)
        nen += 1
        stabilizer += 1
    if idx not in DOWN_FORBIDDEN: # if the blank tile can move down, create a new node with the new state and add it to the successors list
        downs = ns[:]
        downs[idx], downs[idx + 3] = downs[idx + 3], downs[idx]
        nodeDown = Node(downs, "D", nc + 1, idx + 3, node, stabilizer, srcType)
        successors.append(nodeDown)
        nen += 1
        stabilizer += 1
    if idx not in LEFT_FORBIDDEN: # if the blank tile can move left, create a new node with the new state and add it to the successors list
        lefts = ns[:]
        lefts[idx], lefts[idx - 1] = lefts[idx - 1], lefts[idx]
        nodeLeft = Node(lefts, "L", nc + 1, idx - 1, node, stabilizer, srcType) 
        successors.append(nodeLeft)  
        nen += 1
        stabilizer += 1
    return successors # return the successors lists

# __main__ 
with open(sys.argv[1]) as f: # read the input file
    contents = f.read()

puzzle = [int(val) for val in contents.split()] # initial state of the puzzle in array form
idxBlank = puzzle.index(0) # index of the blank tile
initialNode = Node(puzzle, "", 0, idxBlank, None, 0, "") # initial node

file = open(sys.argv[2], 'w') # open the output file

"""
This part is used to write the output to the output file.
"""
writeList = ["BFS","DFS","UCS","Greedy","A*"] 
for item in writeList:
    stabilizer = 0 # set stabilizer to 0 for each search type
    nen = 0 # set number of expanded nodes to 0 for each search type 
    result = GraphSearch(initialNode, item)
    state = result.state[:]
    pathCost = result.pathCost
    path = deque()
    while result.parent != None:
        path.appendleft(result.path)
        result = result.parent
    file.write(f"Number of expanded nodes: {str(nen)}") 
    file.write('\n')
    file.write(f"Path-cost: {str(pathCost)}")
    file.write('\n')
    file.write(f'Path: {" ".join(path).strip()}')
    file.write('\n')

    # print(numExpanded)
    # pprint(vars(result))
file.close() 

# if __name__ == "__main__":
# cProfile.run("main()")




