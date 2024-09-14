#include <iostream>
#include <string>
#include <vector>
#include <queue>
#include "API.h"
using namespace std;


// Function prototypes
void log(const string& text);
bool isValid(int x, int y);

// Constants for directions
enum Direction { UP, DOWN, LEFT, RIGHT };
const int SIZE = 16;

struct Cell{
	// variables for north,east,south,west walls
    bool visited = false; // Visited flag
	bool walls[4] = {false, false, false, false}; // Walls for each direction
    int angleUpdate = 90; // Angle update for the cell
    bool isDead = false; // Dead-end flag
    
};

// Structure for the maze
struct Maze{
	Cell cells[16][16];
};


// Structure for grid positions
struct Position {
    int rowIndex;
    int columnIndex;
    int cellValue;
};

class SolvingMaze {

Maze maze;
std::queue<Position> myQueue;


public:
 SolvingMaze() {
        initializeMaze();
    }

// Function to initialize the maze
void initializeMaze() {
    for (int i = 0; i < 16; i++) {
        for (int j = 0; j < 16; j++) {
            maze.cells[i][j].visited = false;
            maze.cells[i][j].angleUpdate = 90;
            maze.cells[i][j].isDead = false;
            for (int k = 0; k < 4; k++) {
                maze.cells[i][j].walls[k] = false;
            }
        }
    }
}


// Function to move the robot to a new cell in a given direction
void moveToNewPos(int& currentAngle, int direction) {
    switch (direction) {
        case -1:
            log("Invalid direction");
            break;
        case UP:
            API::moveForward();
            break;
        case DOWN:
            currentAngle -= 180;
            API::turnRight();
            API::turnRight();
            API::moveForward();
            break;
        case LEFT:
            currentAngle += 90;
            API::turnLeft();
            API::moveForward();
            break;
        case RIGHT:
            currentAngle -= 90;
            API::turnRight();
            API::moveForward();
            break;
        default:
            break;
    }
    currentAngle = currentAngle % 360;
    if (currentAngle < 0) {
        currentAngle += 360;
    }
}


// Function to move the robot to a new cell in a shortened direction
void moveToShortNewPos(int& currentAngle, int direction) {
    int newDirection = direction;
    switch (currentAngle) {
        case 90:
            break;
        case 270:
            if (direction % 2 == 0) {
                newDirection += 1;
            } else {
                newDirection -= 1;
            }
            break;
        case 0:
            if (direction == 0 || direction == 1) {
                newDirection += 2;
            } else if (direction == 2) {
                newDirection = 1;
            } else {
                newDirection = 0;
            }
            break;
        case 180:
            if (direction == 2 || direction == 3) {
                newDirection -= 2;
            } else if (direction == 0) {
                newDirection = 3;
            } else {
                newDirection = 2;
            }
            break;
    }
    moveToNewPos(currentAngle, newDirection);
}


// Function to check and fill a cell
void decideFill(std::vector<std::vector<int>>& arr, int rowIndex, int columnIndex, int cellValue) {
    if (rowIndex < 0 || columnIndex < 0 || rowIndex >= arr.size() || columnIndex >= arr[0].size() || arr[rowIndex][columnIndex] != -1) {
        return;
    }
    cellValue += 1;
    Position point = {rowIndex, columnIndex, cellValue};
    myQueue.push(point);
    arr[rowIndex][columnIndex] = cellValue;
}

const int rowChange[4] = {1, -1, 0, 0};
const int colChange[4] = {0, 0, -1, 1};

// Function to initialize the flood-fill from a specific cell
void startFloodFillFromCell(std::vector<std::vector<int>>& arr, int rowIndex, int columnIndex) {
    int count = 0;
    Position point = {rowIndex, columnIndex, count};
    myQueue.push(point);
    arr[rowIndex][columnIndex] = 0;

    // Add the neighbors of the starting cell to the queue
    std::vector<Position> neighbors = {
        {rowIndex + 1, columnIndex, count},
        {rowIndex, columnIndex + 1, count},
        {rowIndex + 1, columnIndex + 1, count}
    };
    for (const Position& neighbor : neighbors) {
        myQueue.push(neighbor);
        arr[neighbor.rowIndex][neighbor.columnIndex] = 0;
    }

    // Perform the flood-fill
    while (!myQueue.empty()) {
        Position frontCoord = myQueue.front();
        myQueue.pop();

        // Check each neighbor of the current cell
        for (int i = 0; i < 4; ++i) {
            int newRow = frontCoord.rowIndex + colChange[i];
            int newCol = frontCoord.columnIndex + rowChange[i];
            bool check = maze.cells[frontCoord.rowIndex][frontCoord.columnIndex].walls[i];

            // If the neighbor is valid and not separated by a wall, add it to the queue
            if (isValid(newRow, newCol) && !check) {
                decideFill(arr, newRow, newCol, frontCoord.cellValue);
            }
        }

        // Break if the queue size exceeds a certain limit
        if (myQueue.size() > 120) {
            log("Queue is full");
            break;
        }
    }
}

// Function to initialize the flood-fill from a specific cell with a back option
void startFloodFillWithBack(std::vector<std::vector<int>>& arr, int rowIndex, int columnIndex, int back = 0) {
    int count = 0;

    // Initialize the flood-fill array
    for (int i = 0; i < 16; i++) {
        for (int j = 0; j < 16; j++) {
            arr[i][j] = -1;
            if (back == 2 && !maze.cells[i][j].visited) {
                arr[i][j] = 255;
                maze.cells[i][j].isDead = true;
            }
        }
    }

    // Add the starting cell and its neighbors to the queue
    if (back != 1) {
        std::vector<Position> points = {
            {rowIndex + 1, columnIndex, count},
            {rowIndex, columnIndex + 1, count},
            {rowIndex + 1, columnIndex + 1, count}
        };
        for (const Position& point : points) {
            myQueue.push(point);
            arr[point.rowIndex][point.columnIndex] = 0;
        }
    }

    Position point = {rowIndex, columnIndex, count};
    myQueue.push(point);
    arr[rowIndex][columnIndex] = 0;

    // Perform the flood-fill
    while (!myQueue.empty()) {
        Position frontCoord = myQueue.front();
        myQueue.pop();

        // Check each neighbor of the current cell
        for (int i = 0; i < 4; ++i) {
            int newRow = frontCoord.rowIndex + colChange[i];
            int newCol = frontCoord.columnIndex + rowChange[i];
            bool check = maze.cells[frontCoord.rowIndex][frontCoord.columnIndex].walls[i];

            // If the neighbor is valid and not separated by a wall, add it to the queue
            if (isValid(newRow, newCol) && !check) {
                decideFill(arr, newRow, newCol, frontCoord.cellValue);
            }
        }

        // Break if the queue size exceeds a certain limit
        if (myQueue.size() > 120) {
            log("Queue is full");
            break;
        }
    }
}
///

// Function to update the walls and colors of the cells in the maze
void updateMaze(std::vector<std::vector<int>>& arr) {
    char dir;
    bool isClear = false;
    for (int i = 0; i < 16; i++) {
        for (int j = 0; j < 16; j++) {
            std::string cellValue = std::to_string(arr[i][j]);
            for (int k = 0; k < 4; k++) {
                isClear = maze.cells[i][j].walls[k];
                if(k==0)dir='n';
                else if(k==1)dir='s';
                else if(k==2)dir='w';
                else dir='e'; // North, East, South, West
                if (isClear) {
                    API::setWall(i, j, dir);
                } else {
                    API::clearWall(i, j, dir);
                }
            }
            if (maze.cells[i][j].visited) {
                API::setColor(i, j, 'g'); // Blue
            } else {
                API::clearColor(i, j);
            }
            if (maze.cells[i][j].isDead) {
                API::setText(i, j, "Dead");
                API::setColor(i, j, 'r'); // Red
            } else {
                API::setText(i, j, cellValue);
            }
        }
    }
}

// Function to check if there's a wall in a given direction from the current cell
bool isWallInDirection(Cell cell, int& dir) {
    switch (cell.angleUpdate) {
        case 90:
            break;
        case 270:
            if (dir % 2 == 0) {
                dir += 1;
            } else {
                dir -= 1;
            }
            break;
        case 0:
            if (dir == 0 || dir == 1) {
                dir += 2;
            } else if (dir == 2) {
                dir = 1;
            } else {
                dir = 0;
            }
            break;
        case 180:
            if (dir == 2 || dir == 3) {
                dir -= 2;
            } else if (dir == 0) {
                dir = 3;
            } else {
                dir = 2;
            }
            break;
    }
    return cell.walls[dir];
}

// Function to adjust the direction of the walls in a cell based on the cell's angleUpdate
Cell adjustCellDirection(Cell cell) {
    Cell newCell = cell;
    for (int i = 0; i < 4; i++) {
        int index = i;
        switch (cell.angleUpdate) {
            case 90:
                break;
            case 270:
                if (i % 2 == 0) {
                    index += 1;
                } else {
                    index -= 1;
                }
                break;
            case 0:
                if (i == 0 || i == 1) {
                    index += 2;
                } else if (i == 2) {
                    index = 1;
                } else {
                    index = 0;
                }
                break;
            case 180:
                if (i == 2 || i == 3) {
                    index -= 2;
                } else if (i == 0) {
                    index = 3;
                } else {
                    index = 2;
                }
                break;
        }
        newCell.walls[i] = cell.walls[index];
    }
    return newCell;
}


// Function to get the neighbor with the minimum value
Position getMinNeighbour(Cell cellWall, Position cur, std::vector<std::vector<int>>& arr, bool change = false) {
    int minNeighbour = 255;
    Position nextStep;
    nextStep.cellValue = -1;
    int ind;

    for (int dir = 0; dir < 4; ++dir) {
        int newRow = cur.rowIndex + colChange[dir];
        int newCol = cur.columnIndex + rowChange[dir];
        ind = dir;
        bool check = cellWall.walls[dir];

        if (change) {
            check = isWallInDirection(cellWall, ind);
        }

        if (isValid(newRow, newCol) && !check) {
            if (arr[newRow][newCol] <= minNeighbour) {
                minNeighbour = arr[newRow][newCol];
                nextStep.rowIndex = newRow;
                nextStep.columnIndex = newCol;
                nextStep.cellValue = ind;
            }
        }
    }

    return nextStep;
}


void flood(std::queue<Position>& queue_flood, std::vector<std::vector<int>>& arr) {
    Position cur_queue;
    Position next_step;
    while (!queue_flood.empty()) {
        cur_queue = queue_flood.front();
        queue_flood.pop();

        int min_neightbor = 255;
        bool check_;
        next_step = getMinNeighbour(maze.cells[cur_queue.rowIndex][cur_queue.columnIndex], cur_queue, arr);

        min_neightbor = arr[next_step.rowIndex][next_step.columnIndex];
        if (arr[cur_queue.rowIndex][cur_queue.columnIndex] - 1 != min_neightbor) {
            for (int i = 0; i < 4; i++) {
                Position cur_add;
                cur_add.rowIndex = cur_queue.rowIndex + colChange[i];
                cur_add.columnIndex = cur_queue.columnIndex + rowChange[i];
                check_ = maze.cells[cur_queue.rowIndex][cur_queue.columnIndex].walls[i];
                if (isValid(cur_add.rowIndex, cur_add.columnIndex) && arr[cur_add.rowIndex][cur_add.columnIndex] != 0 && !check_) {
                    queue_flood.push(cur_add);
                }
            }
            if (arr[cur_queue.rowIndex][cur_queue.columnIndex] != 0)
                arr[cur_queue.rowIndex][cur_queue.columnIndex] = min_neightbor + 1;
        }
        int queue_size = queue_flood.size();
        if (queue_size >= 35) {
            log("full queue");
            for (int i = 0; i < queue_size; i++) {
                queue_flood.pop();
            }
            return;
        }
    }
}


// Function to update the walls of a cell based on the current angle
Cell updateCellWalls(int currentAngle, int rowIndex, int columnIndex) {
    Cell newCell;
    newCell.angleUpdate = currentAngle;
    newCell.walls[UP] = API::wallFront();
    newCell.walls[DOWN] = 0;
    newCell.walls[LEFT] = API::wallLeft();
    newCell.walls[RIGHT] = API::wallRight();
    newCell.isDead = false;
    newCell.visited = true;
    maze.cells[rowIndex][columnIndex] = adjustCellDirection(newCell);

    // Check if the cell is a dead-end
    if (newCell.walls[UP] && newCell.walls[LEFT] && newCell.walls[RIGHT] && rowIndex != 0 && columnIndex != 0) {
        API::setColor(rowIndex, columnIndex, 'Y');
        log("NO WAY");
        maze.cells[rowIndex][columnIndex].isDead = true;
    }

    // Update the walls of the neighboring cells
    for (int i = 0; i < 4; i++) {
        API::setColor(rowIndex, columnIndex, 'Y');
        int newRow = rowIndex + colChange[i];
        int newCol = columnIndex + rowChange[i];
        if (isValid(newRow, newCol)) {
            if (i == UP) {
                maze.cells[newRow][newCol].walls[DOWN] = maze.cells[rowIndex][columnIndex].walls[UP];
            } else if (i == LEFT) {
                maze.cells[newRow][newCol].walls[RIGHT] = maze.cells[rowIndex][columnIndex].walls[LEFT];
            } else if (i == RIGHT) {
                maze.cells[newRow][newCol].walls[LEFT] = maze.cells[rowIndex][columnIndex].walls[RIGHT];
            }
        }
    }

    return newCell;
}


Position findPathWithFloodFill(Position start, Position dest, std::vector<std::vector<int>> &arr, int &angle_now) {
    std::queue<Position> path_queue;  
    path_queue.push(start);
    Position cur = start;
    Cell new_cell;
    std::queue<Position> queue_flood;  
    queue_flood.push(start);
    int path_distance_value_find = 0;
    int save_row, save_col;
    Position next_step;
    while (1) {
        if (!path_queue.empty()) {
            cur = path_queue.front();
            new_cell = updateCellWalls(angle_now, cur.rowIndex, cur.columnIndex);
            if (arr[cur.rowIndex][cur.columnIndex] == arr[dest.rowIndex][dest.columnIndex]) break;
            flood(queue_flood, arr);  
            path_queue.pop();
            next_step = getMinNeighbour(new_cell, cur, arr, 1);
            path_queue.push(next_step);
            queue_flood.push(next_step);
            moveToNewPos(angle_now, next_step.cellValue);
            path_distance_value_find++;
        } else {
            log("empty Queue- break");
            break;
        }
    }
    while (!path_queue.empty()) path_queue.pop();
    std::cerr << "Cost:" << path_distance_value_find << std::endl;
    Position p_return = {next_step.rowIndex, next_step.columnIndex, 0};
    return p_return;
}


// Function to find and follow the shortest path
void followShortestPath(std::vector<std::vector<int>>& arr, int angle_now, Position start, Position dest) {
    std::queue<int> nextDirPath;
    Cell newCell;
    int save_row,save_col;
    Position cur = start;
    int angle = angle_now;

    // Iterate over each cell in the path
    for (int i = 0; i < arr[start.rowIndex][start.columnIndex]; i++) {
        int nextDir = -1;
        int newRow;
        int newCol;

        // Check each neighbor of the current cell
        for (int dir = 0; dir < 4; ++dir) {
            newRow = cur.rowIndex + colChange[dir];
            newCol = cur.columnIndex + rowChange[dir];
            bool check = maze.cells[cur.rowIndex][cur.columnIndex].walls[dir];

            // If the neighbor is valid and has a lower value, update nextDir and move to that neighbor
            if (isValid(newRow, newCol) && !check) {
                if (arr[newRow][newCol] < arr[cur.rowIndex][cur.columnIndex]) {
                    nextDir = dir;
                    save_row=newRow;
                    save_col=newCol;
                }
            }
        }

        // If a valid neighbor was found, add the direction to the queue and update the cell in the API
        if (nextDir != -1) {
            cur.rowIndex = save_row;
            cur.columnIndex = save_col;
            nextDirPath.push(nextDir);
            API::setColor(cur.rowIndex, cur.columnIndex, 'g');
            API::setText(cur.rowIndex, cur.columnIndex, std::to_string(arr[cur.rowIndex][cur.columnIndex]));
        }
    }
}
};


void log(const std::string& text) {
    std::cerr << text << std::endl;
}

bool isValid(int x, int y) {
    return (x >= 0 && x < 16 && y >= 0 && y < 16);
}

int main(int argc, char* argv[]) {
    // Create a maze solver
    SolvingMaze solver;

    // Initialize a 2D array
    std::vector<std::vector<int>> grid(SIZE, std::vector<int>(SIZE, -1));

    // Define the start and end positions
    Position startPos = {0, 0, grid[0][0]};
    Position endPos = {7, 7, grid[7][7]};

    // Initialize the flood-fill algorithm and the maze
    solver.startFloodFillFromCell(grid, endPos.rowIndex, endPos.columnIndex);
    solver.initializeMaze();

    // Update the maze
    solver.updateMaze(grid);

    // Define the initial angle
    int initialAngle = 90;

    // Perform the flood-fill algorithm from the start to the end position
    Position newPos = solver.findPathWithFloodFill(startPos, endPos, grid, initialAngle);

    // Re-initialize the flood-fill algorithm from the start position
    solver.startFloodFillWithBack(grid, startPos.rowIndex, startPos.columnIndex, 1);

    // Log the completion of the first flood-fill
    std::cerr << "First flood-fill completed" << std::endl;

    // Perform the flood-fill algorithm from the new position to the start position
    newPos = solver.findPathWithFloodFill(newPos, startPos, grid, initialAngle);

    // Re-initialize the flood-fill algorithm from the end position
    solver.startFloodFillWithBack(grid, endPos.rowIndex, endPos.columnIndex, 2);

    // Find and follow the shortest path from the new position to the end position
    solver.followShortestPath(grid, initialAngle, newPos, endPos);

    return 0;
}