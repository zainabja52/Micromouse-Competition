#include <iostream>
#include <vector>
#include "API.h"
using namespace std;

const int SIZE = 16;

struct Position {
    int rowIndex;
    int columnIndex;
};

class LeftHandRuleSolver {
private:
    int currentAngle;
    Position currentPosition;
    Position startPos;
    Position endPos;

public:
    LeftHandRuleSolver() : currentAngle(90), currentPosition({0, 0}), startPos({0, 0}), endPos({7, 7}) {
        // Initialize the robot's starting position, angle, and end position
        API::setText(0, 0, "Start");
        API::setColor(0, 0, 'Y');
    }

    void moveForward() {
        // Move the robot forward and update its position
        API::moveForward();
        updatePosition();
    }

    void turnLeft() {
        // Turn the robot left and update its angle
        API::turnLeft();
        currentAngle = (currentAngle + 90) % 360;
    }

    void turnRight() {
        // Turn the robot right and update its angle
        API::turnRight();
        currentAngle = (currentAngle + 270) % 360;
    }

    void updatePosition() {
        // Update the robot's current position based on the movement
        if (currentAngle == 0) {
            currentPosition.columnIndex += 1; // Move to the right
        } else if (currentAngle == 90) {
            currentPosition.rowIndex += 1;    // Move downward
        } else if (currentAngle == 180) {
            currentPosition.columnIndex -= 1; // Move to the left
        } else if (currentAngle == 270) {
            currentPosition.rowIndex -= 1;    // Move upward
        }
    }

    bool isGoalReached() {
        return currentPosition.rowIndex == endPos.rowIndex && currentPosition.columnIndex == endPos.columnIndex;
    }

    void exploreMaze() {
        while (!isGoalReached()) {
            if (!API::wallLeft()) {
                // If no wall to the left, turn left and move forward
                turnLeft();
                moveForward();
            } else if (!API::wallFront()) {
                // If no wall in front, move forward
                moveForward();
            } else {
                // If walls on the left and front, turn right
                turnRight();
            }
        }

        // Robot reached the exit
        API::setText(currentPosition.rowIndex, currentPosition.columnIndex, "Exit");
        API::setColor(currentPosition.rowIndex, currentPosition.columnIndex, 'G');
    }
};

int main() {
    // Create an instance of the LeftHandRuleSolver class
    LeftHandRuleSolver solver;

    // Explore the maze using the Left-Hand Rule algorithm
    solver.exploreMaze();

    return 0;
}