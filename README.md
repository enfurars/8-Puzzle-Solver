# 8-Puzzle-Solver
BFS, DFS, UCS, Greedy Search and A* Search are used in this project and detailed description of the project can be found [here](https://github.com/enfurars/8-Puzzle-Solver/blob/main/Project-Description.pdf). 
I part 2 of the project, I have used Manhattan distance as heuristic function with minor changes. I have calculated Manhattan distances of only numbered tiles and take “0” as Manhattan distance for all 3 blank tiles. This is admissible because it never overestimates the true value of the problem. It is also consistent because evaluation function is non-decreasing. 
