# RBC_Phase3

Documentation(What I assume I understand from the Internet and ChatGPT)

ALgorithm
1) Dijkstra's Algorithm
    Pro's
    - Finds the shortest distance possible between each nodes.
    - Evaluate overall distance from each possible nodes and then chooses the shortest one.

    Con's
    - Diagonal and orthognal path are different in length IRL. It is possible to code the program to assume that the distance for orthonal and diagonal movement to be the same but IRL it is not. 

2) A* Algorithm
    Pro's 
    - Most efficient in finding the shortest path.
    - Use a heuristic function to estimate the distance.
    - Seems to be able to store the value of each nodes visited and reflect on it to optimize the path if it is more efficent or the distance is shorter. (Dont quite understand how it works but seems cool overall, like some sort of try and error i assume.)

    Con's 
    - Expensive when the map is too big and there are too many path to process, store the values and optimize the shortest path there is.
    - Eats up a lot of the memory.
    - Heuristic is some sort of estimation, so I assume there will be some uncertainties when it comes to pathfinding as it is not a certain valid value. (More of a feeling even though calculations is involved but it remains an estimation) So, there might be errors appearing.

Coding (ChatGPT)
1) Get to the end from a starting point
2) Take into account urgency and weight of the delivery
3) Added in diagonal movements
4) Displayed the path in grid form(Visual)
5) Fix an error where it can start from an obstacle or end at an obstacle.

Testing


Fancy terms found along the way:
1) Heuristic function
    - Estimate of the distance between a node and a goal node
    - Reduces the number of nodes explored, higher efficiency

2) Euclidean distance
    - Used in grids where diagonal movements are allowed
3) Manhattan Distance
    - Used in grids where only horizontal and vertical movements are allowed
  

