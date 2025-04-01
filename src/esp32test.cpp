#include <Arduino.h>
#include <queue>

using namespace std;

const int SIZE = 8;
const float WALL_THICKNESS = 0.05;
const float CELL_SIZE = 1.0;
const int MAX_ITERATIONS = 100;

const int DIRECTIONS[4][2] = {{-1, 0}, {1, 0}, {0, 1}, {0, -1}};

int walls[SIZE][SIZE][4] = {0};                      // Wall storage (all zeros)
bool visited[SIZE][SIZE] = {false};                  // Visited nodes
int stack[SIZE * SIZE][2];                           // Stack for backtracking
int stackIndex = 0;                                  // Stack pointer

int goal[4][2] = {{3, 4}, {4, 4}, {4, 3}, {3, 3}};   // Goal positions
int initialX = 0, initialY = 0;                      // Start position

// Motor control functions (replace these with actual motor control code)
void moveForward() { Serial.println("Moving Forward"); delay(500); }
void turnLeft() { Serial.println("Turning Left"); delay(500); }
void turnRight() { Serial.println("Turning Right"); delay(500); }
void moveBackward() { Serial.println("Moving Backward"); delay(500); }

void pushStack(int x, int y) {
    stack[stackIndex][0] = x;
    stack[stackIndex][1] = y;
    stackIndex++;
}

void popStack(int &x, int &y) {
    if (stackIndex > 0) {
        stackIndex--;
        x = stack[stackIndex][0];
        y = stack[stackIndex][1];
    }
}

// Returns the opposite direction index
int opp(int i) {
    if (i == 0) return 2;
    if (i == 1) return 3;
    if (i == 2) return 0;
    return 1;
}

// Selects the best move for DFS exploration
bool bestMove(int &x, int &y, int &dx, int &dy) {
    int nx, ny;
    for (int i = 3; i >= 0; i--) {  // Reverse order for DFS (last-added move first)
        nx = x + DIRECTIONS[i][0];
        ny = y + DIRECTIONS[i][1];

        if (nx >= 0 && nx < SIZE && ny >= 0 && ny < SIZE && walls[x][y][i] == 0) {
            if (!visited[nx][ny]) {
                dx = DIRECTIONS[i][0];
                dy = DIRECTIONS[i][1];
                visited[nx][ny] = true; // Mark as visited
                pushStack(x, y); // Push current position for backtracking
                x = nx;
                y = ny;
                return true;
            }
        }
    }

    // No valid moves → **Backtrack**
    if (stackIndex > 0) {
        popStack(x, y);
        dx = 0;
        dy = 0;
        return true;
    }

    return false;  // Exploration finished
}

// Simulates wall detection (Replace with actual sensor reading)
void detectWalls(int x, int y) {
    Serial.print("Detecting walls at: "); Serial.print(x); Serial.print(", "); Serial.println(y);
    // Example: Add a wall at (4,5) in direction 1 (Down)
    if (x == 4 && y == 5) {
        walls[x][y][1] = 1;
        walls[x + DIRECTIONS[1][0]][y + DIRECTIONS[1][1]][opp(1)] = 1;
    }
}

// Breadth-First Search (BFS) for shortest path
queue<pair<int, int>> bfsShortestPath(int startX, int startY, int goalX, int goalY) {
    queue<pair<int, int>> q;
    queue<pair<int, int>> pathQueue;
    bool bfsVisited[SIZE][SIZE] = {false};
    pair<int, int> parent[SIZE][SIZE];

    q.push({startX, startY});
    bfsVisited[startX][startY] = true;
    parent[startX][startY] = {-1, -1};

    while (!q.empty()) {
        int x = q.front().first;
        int y = q.front().second;
        q.pop();

        if (x == goalX && y == goalY) {
            // Reconstruct path
            while (x != startX || y != startY) {
                pathQueue.push({x, y});
                pair<int, int> p = parent[x][y];
                x = p.first;
                y = p.second;
            }
            return pathQueue;
        }

        for (int i = 0; i < 4; i++) {
            int nx = x + DIRECTIONS[i][0];
            int ny = y + DIRECTIONS[i][1];

            if (nx >= 0 && nx < SIZE && ny >= 0 && ny < SIZE && !bfsVisited[nx][ny]) {
                if (walls[x][y][i] == 0 && walls[nx][ny][opp(i)] == 0) {
                    bfsVisited[nx][ny] = true;
                    parent[nx][ny] = {x, y};
                    q.push({nx, ny});
                }
            }
        }
    }

    return {}; // No path found
}

// Exploration Phase
void explore() {
    Serial.println("Starting Exploration...");
    int x = initialX, y = initialY;
    int dx = 0, dy = 0;
    visited[x][y] = true;

    for (int i = 0; i < MAX_ITERATIONS; i++) {
        detectWalls(x, y); // Detect walls dynamically

        if (!bestMove(x, y, dx, dy)) {
            Serial.println("✅ Exploration Complete! All reachable cells visited.");
            break;
        }

        if (dx == 1 && dy == 0) moveForward();
        else if (dx == -1 && dy == 0) moveBackward();
        else if (dx == 0 && dy == 1) turnLeft(), moveForward();
        else if (dx == 0 && dy == -1) turnRight(), moveForward();

        delay(500);
    }
}

// Return to Start Position
void returnToStart(int x, int y) {
    Serial.println("Returning to Start...");
    queue<pair<int, int>> path = bfsShortestPath(x, y, initialX, initialY);
    
    while (!path.empty()) {
        pair<int, int> pos = path.front();
        path.pop();
        moveForward();
        delay(500);
    }

    Serial.println("🏁✅ Robot returned to start position!");
}

// Final Run to Goal
void finalRun() {
    Serial.println("Starting Final Run...");
    queue<pair<int, int>> path = bfsShortestPath(initialX, initialY, goal[0][0], goal[0][1]);

    if (path.empty()) {
        Serial.println("❌ No path found to goal!");
        return;
    }

    while (!path.empty()) {
        pair<int, int> pos = path.front();
        path.pop();
        moveForward();
        delay(500);
    }

    Serial.println("🏆 Final run complete!");
}

void setup() {
    Serial.begin(115200);
    explore();
    returnToStart(initialX, initialY);

    Serial.println("Press 1 to start the final run...");
    while (!Serial.available()); // Wait for user input
    if (Serial.read() == '1') finalRun();
}

void loop() {}
