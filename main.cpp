#include "Algo.h"
#include <iomanip>

void freePath(PathNode *head)
{
    while (head != nullptr)
    {
        PathNode *temp = head;
        head = head->next;
        delete temp;
    }
}

void showPathCase(const string &title, PathNode *path)
{
    cout << "\n--- " << title << " ---\n";
    if (path == nullptr)
    {
        cout << "No path found.\n";
        return;
    }

    printPath(path);
    freePath(path);
}

void connectTwoWay(double adjMatrix[100][100], int a, int b, double cost)
{
    adjMatrix[a][b] = cost;
    adjMatrix[b][a] = cost;
}

void clearGrid(int grid[100][100])
{
    for (int i = 0; i < 100; i++)
        for (int j = 0; j < 100; j++)
            grid[i][j] = 0;
}

void loadLayout(const string layout[], int rows, int cols, int grid[100][100])
{
    clearGrid(grid);
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
            grid[i][j] = (layout[i][j] == '#') ? 1 : 0;
}

void printGrid(int grid[100][100], int rows, int cols,
               int startX, int startY, int goalX, int goalY, char goalMarker)
{
    cout << "Map (# = blocked, . = open):\n";
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            if (i == startX && j == startY)
                cout << "S ";
            else if (i == goalX && j == goalY)
                cout << goalMarker << ' ';
            else
                cout << (grid[i][j] == 1 ? "# " : ". ");
        }
        cout << '\n';
    }
}

string droneModeName(int mode)
{
    if (mode == 1)
        return "Mode 1 Manhattan";
    if (mode == 2)
        return "Mode 2 Euclidean";
    return "Mode 3 Chebyshev";
}

string gridModeName(int mode)
{
    if (mode == 1)
        return "Mode 1 Manhattan";
    return "Mode 2 Chebyshev";
}

void printDroneCatalog(int coords[100][2], int count)
{
    cout << "Drone waypoint catalog:\n";
    for (int i = 0; i < count; i++)
    {
        cout << setw(2) << i << " -> ("
             << setw(2) << coords[i][0] << ","
             << setw(2) << coords[i][1] << ")\n";
    }
}

// BUILD: g++ -std=c++17 -Wall -Wextra -pedantic -o main main.cpp Algo.cpp PathNode.cpp
// RUN: ./main
int main()
{
    cout << fixed << setprecision(2);

    //------------------------------------------
    // TASK 1
    //------------------------------------------
    cout << "========== TASK 1 ==========\n";

    double social[100][100] = {};

    // Graph:
    //
    // 0--1--3
    // | /
    // 2--4
    //

    social[0][1] = 1;
    social[1][0] = 1;

    social[0][2] = 1;
    social[2][0] = 1;

    social[1][2] = 1;
    social[2][1] = 1;

    social[1][3] = 1;
    social[3][1] = 1;

    social[2][4] = 1;
    social[4][2] = 1;

    showPathCase("Friend bridge from person 0 to person 4",
                 findSocialPath(social, 0, 4));

    //------------------------------------------
    // TASK 2
    //------------------------------------------
    cout << "\n========== TASK 2 ==========\n";

    double skyGraph[100][100] = {};
    int skyCoords[100][2] = {};

    skyCoords[0][0] = 0;
    skyCoords[0][1] = 0;
    skyCoords[1][0] = 2;
    skyCoords[1][1] = 6;
    skyCoords[2][0] = 4;
    skyCoords[2][1] = 1;
    skyCoords[3][0] = 6;
    skyCoords[3][1] = 7;
    skyCoords[4][0] = 7;
    skyCoords[4][1] = -2;
    skyCoords[5][0] = 9;
    skyCoords[5][1] = 4;
    skyCoords[6][0] = 11;
    skyCoords[6][1] = 9;
    skyCoords[7][0] = 12;
    skyCoords[7][1] = 1;
    skyCoords[8][0] = 15;
    skyCoords[8][1] = 6;
    skyCoords[9][0] = 16;
    skyCoords[9][1] = 12;
    skyCoords[10][0] = 18;
    skyCoords[10][1] = 3;
    skyCoords[11][0] = 20;
    skyCoords[11][1] = 8;

    connectTwoWay(skyGraph, 0, 1, 4.8);
    connectTwoWay(skyGraph, 0, 2, 3.1);
    connectTwoWay(skyGraph, 1, 3, 3.0);
    connectTwoWay(skyGraph, 1, 5, 6.5);
    connectTwoWay(skyGraph, 2, 4, 2.2);
    connectTwoWay(skyGraph, 2, 5, 4.2);
    connectTwoWay(skyGraph, 3, 6, 2.6);
    connectTwoWay(skyGraph, 3, 8, 6.0);
    connectTwoWay(skyGraph, 4, 5, 3.3);
    connectTwoWay(skyGraph, 4, 7, 2.4);
    connectTwoWay(skyGraph, 5, 8, 2.3);
    connectTwoWay(skyGraph, 6, 9, 2.0);
    connectTwoWay(skyGraph, 7, 8, 4.4);
    connectTwoWay(skyGraph, 7, 10, 2.5);
    connectTwoWay(skyGraph, 8, 11, 2.2);
    connectTwoWay(skyGraph, 9, 11, 1.8);
    connectTwoWay(skyGraph, 10, 11, 4.0);
    connectTwoWay(skyGraph, 0, 11, 30.0);

    cout << "Aurora relay: same drone map, three heuristic personalities.\n";
    printDroneCatalog(skyCoords, 12);
    for (int mode = 1; mode <= 3; mode++)
    {
        showPathCase("Aurora relay - " + droneModeName(mode),
                     findDronePath(skyGraph, skyCoords, 0, 11, mode));
    }

    double blackoutGraph[100][100] = {};
    int blackoutCoords[100][2] = {};
    blackoutCoords[0][0] = -3;
    blackoutCoords[0][1] = 0;
    blackoutCoords[1][0] = -1;
    blackoutCoords[1][1] = 2;
    blackoutCoords[2][0] = 1;
    blackoutCoords[2][1] = 1;
    blackoutCoords[3][0] = 4;
    blackoutCoords[3][1] = -3;
    blackoutCoords[4][0] = 7;
    blackoutCoords[4][1] = -1;
    blackoutCoords[5][0] = 9;
    blackoutCoords[5][1] = 9;
    connectTwoWay(blackoutGraph, 0, 1, 1.4);
    connectTwoWay(blackoutGraph, 1, 2, 1.4);

    cout << "\nBlackout island: target 5 exists in the coordinate system but not in the air network.\n";
    printDroneCatalog(blackoutCoords, 6);
    showPathCase("Blackout island - unreachable target",
                 findDronePath(blackoutGraph, blackoutCoords, 0, 5, 2));

    //------------------------------------------
    // TASK 3
    //------------------------------------------
    cout << "\n========== TASK 3 ==========\n";

    int warehouse[100][100] = {};
    const string diagonalKeyhole[9] = {
        ".#.......",
        "#.#......",
        ".#.#.....",
        "..#.#....",
        "...#.#...",
        "....#.#..",
        ".....#.#.",
        "......#.#",
        ".......#."};

    loadLayout(diagonalKeyhole, 9, 9, warehouse);
    cout << "Diagonal keyhole warehouse: every normal doorway is blocked, but diagonal moves can thread the beads.\n";
    printGrid(warehouse, 9, 9, 0, 0, 8, 8, 'G');
    for (int mode = 1; mode <= 2; mode++)
    {
        showPathCase("Diagonal keyhole - " + gridModeName(mode),
                     findWarehousePath(warehouse, 9, 9, 0, 0, 8, 8, mode));
    }

    int freezerVault[100][100] = {};
    const string sealedFreezer[5] = {
        "#######",
        "#.#...#",
        "###.#.#",
        "#...#.#",
        "#######"};

    loadLayout(sealedFreezer, 5, 7, freezerVault);
    cout << "\nSealed freezer vault: start is open but completely boxed in.\n";
    printGrid(freezerVault, 5, 7, 1, 1, 2, 5, 'G');
    showPathCase("Sealed freezer vault - no legal robot route",
                 findWarehousePath(freezerVault, 5, 7, 1, 1, 2, 5, 2));

    //------------------------------------------
    // TASK 4
    //------------------------------------------
    cout << "\n========== TASK 4 ==========\n";

    int floorPlan[100][100] = {};
    double weightMatrix[100][100] = {};
    const string fireDoorStairs[10] = {
        "#########.",
        "########..",
        "#######.##",
        "######.###",
        "#####.####",
        "####.#####",
        "###.######",
        "##.#######",
        "#.########",
        ".#########"};

    loadLayout(fireDoorStairs, 10, 10, floorPlan);
    cout << "Fire-door staircase: a narrow diagonal escape shaft across a 10x10 floor.\n";
    printGrid(floorPlan, 10, 10, 9, 0, 0, 9, 'E');
    for (int mode = 1; mode <= 2; mode++)
    {
        showPathCase("Fire-door staircase - " + gridModeName(mode),
                     findEvacuationPath(floorPlan, 10, 10, 9, 0, 0, 9, weightMatrix, mode));
    }

    int sealedExit[100][100] = {};
    const string smokeRing[7] = {
        "#######",
        "#.....#",
        "#.###.#",
        "#.#.#.#",
        "#.###.#",
        "#.....#",
        "#######"};

    loadLayout(smokeRing, 7, 7, sealedExit);
    cout << "\nSmoke-ring floor: the exit is visible in the center but sealed off by walls.\n";
    printGrid(sealedExit, 7, 7, 1, 1, 3, 3, 'E');
    showPathCase("Smoke-ring floor - trapped exit",
                 findEvacuationPath(sealedExit, 7, 7, 1, 1, 3, 3, weightMatrix, 2));

    int alreadySafe[100][100] = {};
    const string safeRoom[3] = {
        "...",
        ".#.",
        "..."};

    loadLayout(safeRoom, 3, 3, alreadySafe);
    cout << "\nAlready-at-exit sanity check: evacuation returns the start node itself.\n";
    printGrid(alreadySafe, 3, 3, 0, 0, 0, 0, 'E');
    showPathCase("Already-at-exit - zero movement evacuation",
                 findEvacuationPath(alreadySafe, 3, 3, 0, 0, 0, 0, weightMatrix, 1));

    return 0;
}
