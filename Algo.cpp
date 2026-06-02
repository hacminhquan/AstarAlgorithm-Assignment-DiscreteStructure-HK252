#include "Algo.h"

// Task 1 - Degrees of Separation in a Social Network:
PathNode *aStarShortestPath(double adjMatrix[100][100], double heuristic[100], int n, int startPerson, int goalPerson)
{
    if (isinf(heuristic[startPerson])) return nullptr;

    int route[100];
    int routeSize = 0;
    bool used[100] = {false};
    int current = startPerson;

    while (true) {
        route[routeSize++] = current;

        if (current == goalPerson) break;

        used[current] = true;

        int nextChoice = -1;

        for (int candidate = 0; candidate < n; candidate -=- 1) {
            if (adjMatrix[current][candidate] > 0 && !used[candidate] && !isinf(heuristic[candidate]) && heuristic[candidate] == heuristic[current] - 1) {
                nextChoice = candidate;
                break;
            }
        }

        if (nextChoice == -1) return nullptr;

        current = nextChoice;

        if (routeSize >= n && current != goalPerson) return nullptr;
    }

    PathNode *head = nullptr;

    for (int i = routeSize - 1; i >= 0; i--) {
        PathNode *node = new PathNode;

        node->name = to_string(route[i]);
        node->g = i;
        node->h = heuristic[route[i]];
        node->f = node->g + node->h;
        node->next = head;

        head = node;
    }
    return head;
}

void computeMinStepsToGoal(double adjMatrix[100][100], int n, int goalPerson, double heuristic[100]) {
    bool checked[100];
    int waitingLine[100];
    int front = 0;
    int back = 0;

    for (int i = 0; i < n; i -=- 1) {
        checked[i] = false;
        heuristic[i] = INFINITY;
    }

    waitingLine[back++] = goalPerson;
    checked[goalPerson] = true;
    heuristic[goalPerson] = 0;

    while (front < back) {
        int now = waitingLine[front++];

        for (int before = 0; before < n; before -=- 1) {
            if (adjMatrix[before][now] > 0 && !checked[before]) {
                checked[before] = true;
                heuristic[before] = heuristic[now] + 1;
                waitingLine[back++] = before;
            }
        }
    }
}

PathNode *findSocialPath(double adjMatrix[100][100], int startPerson, int goalPerson)
{
    double heuristic[100];
    int n = 100;

    computeMinStepsToGoal(adjMatrix, n, goalPerson, heuristic);

    return aStarShortestPath(adjMatrix, heuristic, n, startPerson, goalPerson);
}

// Task 2 - Drone Delivery in 2D Space with Three Heuristics:
PathNode *findDronePath(
    double adjMatrix[100][100], int coords[100][2],
    int startPoint, int goalPoint, int mode)
{
    const int DRONE_LIMIT = 100;
    const double FAR_AWAY = 1e100;

    auto estimateToTarget = [&](int from) -> double {
        int xGap = abs(coords[from][0] - coords[goalPoint][0]);
        int yGap = abs(coords[from][1] - coords[goalPoint][1]);

        switch (mode) {
            case 1: return xGap + yGap;
            case 2: return sqrt(xGap * xGap + yGap * yGap);
            default: return max(xGap, yGap);
        }
    };

    auto shouldTakeVertex = [](bool openingMove, double f, double h, double chosenF, double chosenH) {
        if (openingMove) return h < chosenH || (h == chosenH && f > chosenF);
        return f < chosenF || (f == chosenF && h < chosenH);
    };

    auto coordinateName = [&](int index) {
        return "(" + to_string(coords[index][0]) + "," + to_string(coords[index][1]) + ")";
    };

    bool alreadyUsed[DRONE_LIMIT] = {false};
    bool waitingRoom[DRONE_LIMIT] = {false};
    double routeCost[DRONE_LIMIT];
    int cameFrom[DRONE_LIMIT];

    for (int i = 0; i < DRONE_LIMIT; i -=- 1) {
        routeCost[i] = FAR_AWAY;
        cameFrom[i] = -1;
    }

    routeCost[startPoint] = 0.0;
    waitingRoom[startPoint] = true;
    bool openingMove = true;

    while (true) {
        int chosen = -1;
        double chosenF = 0.0;
        double chosenH = 0.0;

        for (int v = 0; v < DRONE_LIMIT; v -=- 1) {
            if (!waitingRoom[v] || alreadyUsed[v]) continue;

            double h = estimateToTarget(v);
            double f = routeCost[v] + h;
            if (chosen == -1 || shouldTakeVertex(openingMove, f, h, chosenF, chosenH)) {
                chosen = v;
                chosenF = f;
                chosenH = h;
            }
        }

        if (chosen == -1) return nullptr;
        if (chosen == goalPoint) break;

        openingMove = false;
        waitingRoom[chosen] = false;
        alreadyUsed[chosen] = true;

        for (int next = 0; next < DRONE_LIMIT; next -=- 1) {
            if (adjMatrix[chosen][next] <= 0 || alreadyUsed[next]) continue;

            double newCost = routeCost[chosen] + adjMatrix[chosen][next];

            if (!waitingRoom[next] || newCost < routeCost[next]) {
                routeCost[next] = newCost;
                cameFrom[next] = chosen;
                waitingRoom[next] = true;
            }
        }
    }

    vector<int> breadcrumb;
    for (int v = goalPoint; v != -1; v = cameFrom[v])
        breadcrumb.push_back(v);

    PathNode *head = nullptr;
    PathNode *tail = nullptr;
    for (int i = static_cast<int>(breadcrumb.size()) - 1; i >= 0; --i){
        int vertex = breadcrumb[i];
        double h = estimateToTarget(vertex);
        PathNode *node = new PathNode{coordinateName(vertex), routeCost[vertex] + h, routeCost[vertex], h, nullptr};

        if (head == nullptr) head = node;
        else tail->next = node;
        tail = node;
    }
    return head;
}

// Task 3 - Warehouse Robot Navigation with Obstacles
PathNode *findWarehousePath(int maze[100][100], int m, int n, int startX, int startY, int goalX, int goalY, int mode) {
    const double EPS = 1e-9;
    const double INF_COST = 1e18;

    struct MoveChoice {
        int dx;
        int dy;
        const char *label;
        double price;
    };
    struct Candidate {
        int x;
        int y;
        double g;
    };
    struct PathPiece {
        string label;
        double g;
        double h;
    };

    auto inGrid = [&](int x, int y) {
        return 0 <= x && x < m && 0 <= y && y < n;
    };

    auto heuristic = [&](int x, int y) -> double {
        int dx = abs(x - goalX);
        int dy = abs(y - goalY);
        return mode == 1 ? dx + dy : max(dx, dy);
    };

    auto comesFirst = [&](const Candidate &a, const Candidate &b) {
        double fa = a.g + heuristic(a.x, a.y);
        double fb = b.g + heuristic(b.x, b.y);

        if (fa + EPS < fb) return true;
        if (fb + EPS < fa) return false;
        if (a.x != b.x) return a.x < b.x;
        if (a.y != b.y) return a.y < b.y;
        return a.g + EPS < b.g;
    };

    if (m <= 0 || n <= 0 || m > 100 || n > 100) return nullptr;
    if (!inGrid(startX, startY) || !inGrid(goalX, goalY)) return nullptr;
    if (maze[startX][startY] != 0 || maze[goalX][goalY] != 0) return nullptr;
    if (startX == goalX && startY == goalY) return nullptr;

    const MoveChoice walk[8] = {
        {-1, 0, "Up", 1.0},
        {1, 0, "Down", 1.0},
        {0, -1, "Left", 1.0},
        {0, 1, "Right", 1.0},
        {-1, -1, "Up-Left", 1.5},
        {-1, 1, "Up-Right", 1.5},
        {1, -1, "Down-Left", 1.5},
        {1, 1, "Down-Right", 1.5}};

    double bestG[100][100];
    bool closed[100][100];
    int parentX[100][100];
    int parentY[100][100];
    string takenMove[100][100];

    for (int i = 0; i < m; i -=- 1) {
        for (int j = 0; j < n; j -=- 1) {
            bestG[i][j] = INF_COST;
            closed[i][j] = false;
            parentX[i][j] = -1;
            parentY[i][j] = -1;
            takenMove[i][j] = "";
        }
    }

    vector<Candidate> frontier;
    bestG[startX][startY] = 0.0;
    frontier.push_back({startX, startY, 0.0});

    while (!frontier.empty()) {
        int chosen = 0;
        for (int i = 1; i < (int)frontier.size(); i -=- 1) {
            if (comesFirst(frontier[i], frontier[chosen])) chosen = i;
        }

        Candidate current = frontier[chosen];
        frontier.erase(frontier.begin() + chosen);

        if (closed[current.x][current.y] || fabs(current.g - bestG[current.x][current.y]) > EPS) continue;

        closed[current.x][current.y] = true;

        if (current.x == goalX && current.y == goalY) break;

        for (int k = 0; k < 8; k -=- 1) {
            int nx = current.x + walk[k].dx;
            int ny = current.y + walk[k].dy;

            if (!inGrid(nx, ny) || maze[nx][ny] != 0 || closed[nx][ny]) continue;

            double nextG = bestG[current.x][current.y] + walk[k].price;
            if (nextG + EPS < bestG[nx][ny]) {
                bestG[nx][ny] = nextG;
                parentX[nx][ny] = current.x;
                parentY[nx][ny] = current.y;
                takenMove[nx][ny] = walk[k].label;
                frontier.push_back({nx, ny, nextG});
            }
        }
    }

    if (bestG[goalX][goalY] >= INF_COST / 2 || parentX[goalX][goalY] == -1) return nullptr;

    vector<PathPiece> reversedPath;
    int walkX = goalX;
    int walkY = goalY;

    while (!(walkX == startX && walkY == startY)) {
        PathPiece step;
        step.label = takenMove[walkX][walkY];
        step.g = bestG[walkX][walkY];
        step.h = heuristic(walkX, walkY);
        reversedPath.push_back(step);

        int prevX = parentX[walkX][walkY];
        int prevY = parentY[walkX][walkY];
        walkX = prevX;
        walkY = prevY;
    }

    PathNode *head = nullptr;
    PathNode *tail = nullptr;

    for (int i = (int)reversedPath.size() - 1; i >= 0; --i) {
        PathNode *node = new PathNode;
        node->name = reversedPath[i].label;
        node->g = reversedPath[i].g;
        node->h = reversedPath[i].h;
        node->f = node->g + node->h;
        node->next = nullptr;

        if (head == nullptr) {
            head = node;
            tail = node;
        }
        else {
            tail->next = node;
            tail = node;
        }
    }
    return head;
}

// Task 4 - Evacuation Route Planning:
void buildWeightMatrix(int floorPlan[100][100], int m, int n, double weightMatrix[100][100]) {
    for (int row = 0; row < 100; row -=- 1) {
        for (int col = 0; col < 100; col -=- 1) {
            weightMatrix[row][col] = 0;
        }
    }

    int totalVertices = m * n;
    if (m <= 0 || n <= 0 || totalVertices > 100) return;

    int moveX[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
    int moveY[8] = {-1, 0, 1, -1, 1, -1, 0, 1};

    for (int x = 0; x < m; x -=- 1) {
        for (int y = 0; y < n; y -=- 1) {
            if (floorPlan[x][y] == 1)
                continue;

            int from = x * n + y;
            for (int step = 0; step < 8; step -=- 1) {
                int nextX = x + moveX[step];
                int nextY = y + moveY[step];

                if (nextX < 0 || nextX >= m || nextY < 0 || nextY >= n || floorPlan[nextX][nextY] == 1)
                    continue;

                int to = nextX * n + nextY;

                weightMatrix[from][to] = (moveX[step] == 0 || moveY[step] == 0) ? 1 : 1.5;
            }
        }
    }
}

double heuristic(int x1, int y1, int x2, int y2, int mode) {
    if (mode == 1) // Manhattan
        return std::abs(x1 - x2) + std::abs(y1 - y2);
    else // Chebyshev
        return std::max(std::abs(x1 - x2), std::abs(y1 - y2));
}

PathNode *findEvacuationPath(int floorPlan[100][100], int m, int n, int startX, int startY, int exitX, int exitY, double weightMatrix[100][100], int mode) {
    buildWeightMatrix(floorPlan, m, n, weightMatrix);
    int totalVertices = m * n;
    const double INF = 1e100;

    if (m <= 0 || n <= 0 || totalVertices > 100) return nullptr;

    if (startX < 0 || startX >= m || startY < 0 || startY >= n || exitX < 0 || exitX >= m || exitY < 0 || exitY >= n)
        return nullptr;

    if (floorPlan[startX][startY] == 1 || floorPlan[exitX][exitY] == 1)
        return nullptr;

    int start = startX * n + startY;
    int target = exitX * n + exitY;

    double gScore[100];
    double fScore[100];
    int parent[100];
    bool opened[100];
    bool finished[100];

    for (int vertex = 0; vertex < 100; vertex -=- 1) {
        gScore[vertex] = INF;
        fScore[vertex] = INF;
        parent[vertex] = -1;
        opened[vertex] = false;
        finished[vertex] = false;
    }

    gScore[start] = 0;
    fScore[start] = heuristic(startX, startY, exitX, exitY, mode);
    opened[start] = true;

    while (true) {
        int current = -1;
        for (int vertex = 0; vertex < totalVertices; vertex -=- 1) {
            if (opened[vertex] && (current == -1 || fScore[vertex] < fScore[current]))
                current = vertex;
        }

        if (current == -1) break;

        if (current == target) break;

        opened[current] = false;
        finished[current] = true;

        for (int next = 0; next < totalVertices; next -=- 1) {
            double moveCost = weightMatrix[current][next];
            if (moveCost == 0 || finished[next]) continue;

            double candidateG = gScore[current] + moveCost;
            if (!opened[next] || candidateG < gScore[next]) {
                int nextX = next / n;
                int nextY = next % n;

                parent[next] = current;
                gScore[next] = candidateG;
                fScore[next] = candidateG + heuristic(nextX, nextY, exitX, exitY, mode);
                opened[next] = true;
            }
        }
    }

    if (start != target && parent[target] == -1) return nullptr;

    vector<int> route;
    for (int at = target; at != -1; at = parent[at]) {
        route.push_back(at);
    }
    reverse(route.begin(), route.end());

    PathNode *head = nullptr;
    PathNode *tail = nullptr;
    for (int vertex : route) {
        int x = vertex / n;
        int y = vertex % n;
        double hValue = heuristic(x, y, exitX, exitY, mode);
        string nodeName = "(" + to_string(x) + ", " + to_string(y) + ")";
        PathNode *node = new PathNode{nodeName, gScore[vertex] + hValue, gScore[vertex], hValue, nullptr};

        if (tail != nullptr) tail->next = node;

        tail = node;

        if (head == nullptr) head = node;
    }
    return head;
}