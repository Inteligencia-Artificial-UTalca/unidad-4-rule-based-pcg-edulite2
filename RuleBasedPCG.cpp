#include <iostream>
#include <vector>
#include <random>   // For random number generation
#include <chrono>   // For seeding the random number generator

// Define Map as a vector of vectors of integers.
// You can change 'int' to whatever type best represents your cells (e.g., char, bool).
using Map = std::vector<std::vector<int>>;

template <typename T>
T clamp(T v, T lo, T hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

/**
 * @brief Prints the map (matrix) to the console.
 * @param map The map to print.
 */
void printMap(const Map& map) {
    std::cout << "--- Current Map ---" << std::endl;
    for (const auto& row : map) {
        for (int cell : row) {
            // Adapt this to represent your cells meaningfully (e.g., ' ' for empty, '#' for occupied).
            std::cout << cell << " ";
        }
        std::cout << std::endl;
    }
    std::cout << "-------------------" << std::endl;
}

/**
 * @brief Function to implement the Cellular Automata logic.
 * It should take a map and return the updated map after one iteration.
 * @param currentMap The map in its current state.
 * @param W Width of the map.
 * @param H Height of the map.
 * @param R Radius of the neighbor window (e.g., 1 for 3x3, 2 for 5x5).
 * @param U Threshold to decide if the current cell becomes 1 or 0.
 * @return The map after applying the cellular automata rules.
 */
Map cellularAutomata(const Map& currentMap, int W, int H, int R, double U) {
    Map newMap = currentMap;
    for (int x = 0; x < H; ++x) {
        for (int y = 0; y < W; ++y) {
            int count = 0;
            int total = 0;
            for (int dx = -R; dx <= R; ++dx) {
                for (int dy = -R; dy <= R; ++dy) {
                    int nx = x + dx;
                    int ny = y + dy;
                    if (nx < 0 || nx >= H || ny < 0 || ny >= W) {
                        // Considera fuera del mapa como 1 (pared)
                        count += 1;
                    } else {
                        count += currentMap[nx][ny];
                    }
                    total++;
                }
            }
            double ratio = static_cast<double>(count) / total;
            newMap[x][y] = (ratio > U) ? 1 : 0;
        }
    }
    return newMap;
}

/**
 * @brief Function to implement the Drunk Agent logic.
 * It should take a map and parameters controlling the agent's behavior,
 * then return the updated map after the agent performs its actions.
 *
 * @param currentMap The map in its current state.
 * @param W Width of the map.
 * @param H Height of the map.
 * @param J The number of times the agent "walks" (initiates a path).
 * @param I The number of steps the agent takes per "walk".
 * @param roomSizeX Max width of rooms the agent can generate.
 * @param roomSizeY Max height of rooms the agent can generate.
 * @param probGenerateRoom Probability (0.0 to 1.0) of generating a room at each step.
 * @param probIncreaseRoom If no room is generated, this value increases probGenerateRoom.
 * @param probChangeDirection Probability (0.0 to 1.0) of changing direction at each step.
 * @param probIncreaseChange If direction is not changed, this value increases probChangeDirection.
 * @param agentX Current X position of the agent (updated by reference).
 * @param agentY Current Y position of the agent (updated by reference).
 * @return The map after the agent's movements and actions.
 */
Map drunkAgent(const Map& currentMap, int W, int H, int J, int I, int roomSizeX, int roomSizeY,
               double probGenerateRoom, double probIncreaseRoom,
               double probChangeDirection, double probIncreaseChange,
               int& agentX, int& agentY) {
    Map newMap = currentMap; // The new map is a copy of the current one

    // Random engine
    static std::mt19937 rng(static_cast<unsigned int>(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::uniform_int_distribution<int> dirDist(0, 3); // 0:arriba, 1:abajo, 2:izq, 3:der
    std::uniform_real_distribution<double> probDist(0.0, 1.0);

    // Direcciones: {dx, dy}
    const int dx[4] = {-1, 1, 0, 0};
    const int dy[4] = {0, 0, -1, 1};

    // Si el agente está fuera del mapa, lo reubicamos aleatoriamente
    if (agentX < 0 || agentX >= H || agentY < 0 || agentY >= W) {
        std::uniform_int_distribution<int> xDist(0, H - 1);
        std::uniform_int_distribution<int> yDist(0, W - 1);
        agentX = xDist(rng);
        agentY = yDist(rng);
    }

    int dir = dirDist(rng); // Dirección inicial aleatoria

    double pRoom = probGenerateRoom;
    double pDir = probChangeDirection;

    for (int j = 0; j < J; ++j) {
        for (int i = 0; i < I; ++i) {
            // Marca el paso del agente (pasillo)
            if (agentX >= 0 && agentX < H && agentY >= 0 && agentY < W)
                newMap[agentX][agentY] = 1;

            // Generar habitación
            if (probDist(rng) < pRoom) {
                // Generar habitación centrada en el agente
                int halfRoomX = roomSizeX / 2;
                int halfRoomY = roomSizeY / 2;
                for (int rx = -halfRoomX; rx <= halfRoomX; ++rx) {
                    for (int ry = -halfRoomY; ry <= halfRoomY; ++ry) {
                        int nx = clamp(agentX + rx, 0, H - 1);
                        int ny = clamp(agentY + ry, 0, W - 1);
                        newMap[nx][ny] = 1;
                    }
                }
                pRoom = probGenerateRoom; // Reinicia probabilidad
            } else {
                pRoom = std::min(1.0, pRoom + probIncreaseRoom);
            }

            // Cambiar dirección
            if (probDist(rng) < pDir) {
                dir = dirDist(rng);
                pDir = probChangeDirection; // Reinicia probabilidad
            } else {
                pDir = std::min(1.0, pDir + probIncreaseChange);
            }

            // Mover agente
            int nextX = agentX + dx[dir];
            int nextY = agentY + dy[dir];

            // Si sale del mapa, cambia dirección y detén este paso
            if (nextX < 0 || nextX >= H || nextY < 0 || nextY >= W) {
                dir = dirDist(rng);
                continue;
            }

            agentX = nextX;
            agentY = nextY;
        }
    }
    return newMap;
}

int main() {
    std::cout << "--- SIMULACIÓN DRUNK AGENT Y CELLULAR AUTOMATA ---" << std::endl;

    // --- Initial Map Configuration ---
    int mapRows = 20;
    int mapCols = 40;
    Map mapDrunk(mapRows, std::vector<int>(mapCols, 0)); // Mapa para Drunk Agent
    Map mapCA(mapRows, std::vector<int>(mapCols, 0));    // Mapa para Cellular Automata

    // Inicializa ambos mapas con el mismo estado si lo deseas
    std::mt19937 rng(static_cast<unsigned int>(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::uniform_int_distribution<int> binDist(0, 1);
    for (int i = 0; i < mapRows; ++i)
        for (int j = 0; j < mapCols; ++j)
            mapCA[i][j] = binDist(rng); // Solo el de automata inicia con ruido
    // El de drunk agent inicia en ceros

    // Drunk Agent's initial position aleatoria
    std::uniform_int_distribution<int> xDist(0, mapRows - 1);
    std::uniform_int_distribution<int> yDist(0, mapCols - 1);
    int drunkAgentX = xDist(rng);
    int drunkAgentY = yDist(rng);

    std::cout << "\nEstado inicial mapa Drunk Agent:" << std::endl;
    printMap(mapDrunk);
    std::cout << "\nEstado inicial mapa Cellular Automata:" << std::endl;
    printMap(mapCA);

    // --- Simulation Parameters ---
    int numIterations = 5; // Number of simulation steps

    // Cellular Automata Parameters
    int ca_W = mapCols;
    int ca_H = mapRows;
    int ca_R = 1;      // Radius of neighbor window
    double ca_U = 0.5; // Threshold

    // Drunk Agent Parameters
    int da_W = mapCols;
    int da_H = mapRows;
    int da_J = 5;      // Number of "walks"
    int da_I = 10;     // Steps per walk
    int da_roomSizeX = 5;
    int da_roomSizeY = 3;
    double da_probGenerateRoom = 0.1;
    double da_probIncreaseRoom = 0.05;
    double da_probChangeDirection = 0.2;
    double da_probIncreaseChange = 0.03;

    // --- Main Simulation Loop ---
    for (int iteration = 0; iteration < numIterations; ++iteration) {
        std::cout << "\n--- Iteración " << iteration + 1 << " ---" << std::endl;

        // Cellular Automata SOLO sobre su mapa
        mapCA = cellularAutomata(mapCA, ca_W, ca_H, ca_R, ca_U);

        // Drunk Agent SOLO sobre su mapa
        mapDrunk = drunkAgent(mapDrunk, da_W, da_H, da_J, da_I, da_roomSizeX, da_roomSizeY,
                              da_probGenerateRoom, da_probIncreaseRoom,
                              da_probChangeDirection, da_probIncreaseChange,
                              drunkAgentX, drunkAgentY);

        std::cout << "\nMapa Drunk Agent:" << std::endl;
        printMap(mapDrunk);

        std::cout << "\nMapa Cellular Automata:" << std::endl;
        printMap(mapCA);
    }

    std::cout << "\n--- Simulación terminada ---" << std::endl;
    return 0;
}