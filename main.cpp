#include <iostream>
#include <vector>
#include <queue>
#include <stack>
#include <cmath>
#include <chrono>
#include <algorithm>

struct Task {
    int taskId;
    int urgency;
    double weight;
    std::pair<int, int> start;
    std::vector<std::pair<int, int>> goals;

    // Priority comparison: higher urgency comes first, if equal, higher weight comes first
    bool operator<(const Task& other) const {
        if (urgency == other.urgency) {
            return weight < other.weight; // Prioritize by weight if urgency is equal
        }
        return urgency < other.urgency; // Otherwise prioritize by urgency
    }
};

struct Node {
    int x, y; // Coordinates of the node
    double cost; // Cost to reach this node
    double heuristic; // Estimated cost to the goal
    Node* parent; // Pointer to the parent node for path reconstruction

    // Calculate total cost as the sum of actual cost and heuristic
    double totalCost() const {
        return cost + heuristic;
    }

    // Comparison operator for priority queue (min-heap)
    bool operator>(const Node& other) const {
        return totalCost() > other.totalCost();
    }
};

const int GRID_SIZE = 30;
const int OBSTACLE = 1;
const int EMPTY = 0;

// Define the grid with obstacles and empty spaces
int grid[GRID_SIZE][GRID_SIZE] = {
    {0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0},
    {0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0},
    {0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0},
    {0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0},    
    {0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0},    
    {0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},    
    {0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0},    
    {0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0},    
    {0, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0},    
    {0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0},    
    {0, 0, 0, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0},    
    {0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0},    
    {0, 0, 0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0},    
    {0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0},    
    {0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0},    
    {0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0},    
    {0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0},    
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0},    
    {0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},    
    {0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0},    
    {0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0},    
    {0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0},    
    {0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0},    
    {0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0},    
    {0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0},    
    {0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},    
    {0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0},    
    {0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0},    
    {0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0},    
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0}    

};

// Directions for movement (up, down, left, right, and diagonals)
std::vector<std::pair<int, int>> directions = {
    {-1, 0}, {1, 0}, {0, -1}, {0, 1}, {-1, -1}, {-1, 1}, {1, -1}, {1, 1}
};

// Calculate Euclidean distance as the heuristic
double calculateHeuristic(const std::pair<int, int>& a, const std::pair<int, int>& b) {
    return std::sqrt(std::pow(a.first - b.first, 2) + std::pow(a.second - b.second, 2));
}

// Check if a cell is within bounds and not an obstacle
bool isValid(int x, int y) {
    return x >= 0 && x < GRID_SIZE && y >= 0 && y < GRID_SIZE && grid[x][y] != OBSTACLE;
}

// Perform A* path planning for multiple goals
std::pair<std::vector<std::pair<int, int>>, std::pair<int, int>> aStarPathPlanningMultiGoal(
    const std::pair<int, int>& start,
    const std::vector<std::pair<int, int>>& goals,
    double weight) {

    // Priority queue to store nodes to explore
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openList;
    bool visited[GRID_SIZE][GRID_SIZE] = {false};

    // Add the starting node to the open list
    openList.push({start.first, start.second, 0, 0, nullptr});

    std::vector<std::pair<int, int>> bestPath;
    std::pair<int, int> bestGoal = {-1, -1}; // Track the best goal reached
    double shortestDistance = std::numeric_limits<double>::infinity();

    while (!openList.empty()) {
        Node current = openList.top();
        openList.pop();

        // Skip already visited nodes
        if (visited[current.x][current.y]) {
            continue;
        }
        visited[current.x][current.y] = true;

        // Check if the current node is a goal
        for (const auto& goal : goals) {
            if (current.x == goal.first && current.y == goal.second) {
                // Reconstruct the path by tracing back from the current node
                std::vector<std::pair<int, int>> path;
                Node* pathNode = &current;
                while (pathNode != nullptr) {
                    path.push_back({pathNode->x, pathNode->y});
                    pathNode = pathNode->parent;
                }
                std::reverse(path.begin(), path.end());

                // Update the best path if this one is shorter
                if (current.cost < shortestDistance) {
                    shortestDistance = current.cost;
                    bestPath = path;
                    bestGoal = goal;
                }
                continue;
            }
        }

        // Explore neighbors of the current node
        for (const auto& dir : directions) {
            int newX = current.x + dir.first;
            int newY = current.y + dir.second;

            if (isValid(newX, newY) && !visited[newX][newY]) {
                // Calculate cost and heuristic for the neighbor
                double newCost = current.cost + weight;
                double heuristic = std::numeric_limits<double>::infinity();
                for (const auto& goal : goals) {
                    heuristic = std::min(heuristic, calculateHeuristic({newX, newY}, goal));
                }

                // Add the neighbor to the open list
                openList.push({newX, newY, newCost, heuristic, new Node(current)});
            }
        }
    }

    return {bestPath, bestGoal};
}

// Display the grid with the path, start, and goal marked
void displayGridWithPath(const std::vector<std::pair<int, int>>& path, 
                         const std::pair<int, int>& start, 
                         const std::pair<int, int>& goal) {
    char displayGrid[GRID_SIZE][GRID_SIZE];

    // Initialize display grid
    for (int i = 0; i < GRID_SIZE; ++i) {
        for (int j = 0; j < GRID_SIZE; ++j) {
            displayGrid[i][j] = (grid[i][j] == OBSTACLE) ? 'X' : '.';
        }
    }

    // Mark the path
    for (const auto& p : path) {
        displayGrid[p.first][p.second] = '*';
    }

    // Mark the start and goal
    displayGrid[start.first][start.second] = 'S';
    displayGrid[goal.first][goal.second] = 'E';

    // Display the grid
    for (int i = 0; i < GRID_SIZE; ++i) {
        for (int j = 0; j < GRID_SIZE; ++j) {
            std::cout << displayGrid[i][j] << ' ';
        }
        std::cout << '\n';
    }
}

// Process a queue of tasks, prioritizing higher urgency and considering weight
void processTasksWithMultipleEndpoints(std::priority_queue<Task> taskQueue) {
    while (!taskQueue.empty()) {
        Task currentTask = taskQueue.top();
        taskQueue.pop();

        std::cout << "Processing Task ID: " << currentTask.taskId 
                  << " (Urgency: " << currentTask.urgency 
                  << ", Weight: " << currentTask.weight << ")\n";

        auto taskStartTime = std::chrono::high_resolution_clock::now();

        // Plan path for the current task
        auto [path, selectedGoal] = aStarPathPlanningMultiGoal(currentTask.start, currentTask.goals, currentTask.weight);

        if (path.empty()) {
            std::cout << "No path found for Task ID: " << currentTask.taskId << "\n";
        } else {
            std::cout << "Path found for Task ID: " << currentTask.taskId 
                      << " to goal (" << selectedGoal.first << ", " << selectedGoal.second << ")\n";

            displayGridWithPath(path, currentTask.start, selectedGoal);
        }

        auto taskEndTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> taskElapsed = taskEndTime - taskStartTime;
        std::cout << "Task completed in: " << taskElapsed.count() << " seconds\n\n";
    }
}

int main() {
    // Create a priority queue of tasks (higher urgency first, then higher weight)
    std::priority_queue<Task> taskQueue;
    taskQueue.push({1, 7, 1.5, {0, 0}, {{1, 1}, {2, 2}, {3, 3}}});
    taskQueue.push({2, 10, 1.2, {5, 5}, {{7, 7}, {3, 3}, {7, 3}}});
    taskQueue.push({3, 7, 2.0, {10, 10}, {{15, 30}, {5, 29}, {20, 10}}});

    // Process the tasks
    processTasksWithMultipleEndpoints(taskQueue);

    return 0;
}
