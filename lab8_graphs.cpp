#include <iostream>
#include <vector>
#include <map>
#include <fstream>
#include <sstream>
#include <cmath>
#include <limits>
#include <chrono>
#include <algorithm>
#include <stack>
#include <set>
#include <queue>
#include <cassert>

using namespace std;
using namespace chrono;

using Graph = map<int, vector<pair<int, double>>>; // O(V + E)
using Coordinates = map<int, pair<double, double>>; // O(V)

// Функция для парсинга графа из файла
void parseGraph(const string& filename, Graph& graph, Coordinates& coordinates) {
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Error opening file: " << filename << endl;
        return;
    }

    map<pair<double, double>, int> coordinateToId; // V*(8*2 + 4) байта
    int nodeId = 0; // 4 байта

    auto getId = [&](double lon, double lat) -> int { // 4 байта
        pair<double, double> coord = {lon, lat}; // V*8*2 байта
        auto it = coordinateToId.find(coord);
        if (it == coordinateToId.end()) {
            coordinateToId[coord] = nodeId;
            coordinates[nodeId] = coord;
            return nodeId++;
        }
        return it->second;
    };

    string line; // O(1)
    while (getline(file, line)) {
        size_t colon = line.find(":");
        if (colon == string::npos) continue;

        string parentNode = line.substr(0, colon); // O(m), m - длина строки родительского узла
        string children = line.substr(colon + 1); // O(m), m - длина строки родительского узла

        double lon1, lat1; // 8 * 2 байт
        sscanf(parentNode.c_str(), "%lf,%lf", &lon1, &lat1);
        int parentId = getId(lon1, lat1); // 4 байта

        stringstream ss(children);
        string child; // O(1)
        while (getline(ss, child, ';')) {
            double lon2, lat2, weight; // 8 * 3 байта
            if (sscanf(child.c_str(), "%lf,%lf,%lf", &lon2, &lat2, &weight) == 3) {
                int childId = getId(lon2, lat2); // 4 байта
                graph[parentId].emplace_back(childId, weight);
                graph[childId].emplace_back(parentId, weight);
            }
        }
    }
}

// Функция для нахождения ближайшего узла
int findNearestNode(const Coordinates& coordinates, double lon, double lat) {
    int nearestNode = -1; // 4 байта
    double minDistance = numeric_limits<double>::infinity(); // 8 байт

    // Проходим по всем координатам
    for (const auto& [node, coord] : coordinates) {
        // Вычисляем расстояние до текущей координаты
        double distance = sqrt(pow(coord.first - lon, 2) + pow(coord.second - lat, 2)); // 8 байт
        if (distance < minDistance) {
            minDistance = distance;
            nearestNode = node;
        }
    }
    return nearestNode;
}

// Функция для печати пути и его итогового веса
double printPath(const map<int, int>& parent, int start, int goal, const Graph& graph) {
    vector<int> path; // 4 * len(path) байт
    double weight = 0.0; // 8 байт

    // Восстанавливаем путь от goal до start
    for (int cur = goal; cur != start; cur = parent.at(cur)) { // 4 байта
        path.push_back(cur);
        // Находим вес текущего ребра
        for (const auto& [neighbor, w] : graph.at(parent.at(cur))) {
            if (neighbor == cur) weight += w;
        }
    }
    path.push_back(start);

    // Переворачиваем путь, чтобы получить его от start до goal
    reverse(path.begin(), path.end());

    // Печатаем итоговый путь
    // cout << "Final path: ";
    // for (const int node : path) {
    //     cout << node << " ";
    // }
    // cout << endl;

    cout << "Final weight: " << weight << endl;
    return weight;
}

// Функция для BFS
double BFS(const Graph& graph, int start, int goal) {
    auto start_time = high_resolution_clock::now();

    queue<int> q; // максимум - V * 4 байта
    set<int> visited; // максимум - V * 4 байта
    map<int, int> parent; // максимум - V * 4*2 байта
    q.push(start);

    while (!q.empty()) {
        int current = q.front(); // 4 байта
        q.pop();

        if (visited.count(current)) continue;
        visited.insert(current);

        if (current == goal) {
            auto end_time = high_resolution_clock::now();
            double totalWeight = printPath(parent, start, goal, graph); // 8 байт
            cout << "BFS time: " << duration_cast<milliseconds>(end_time - start_time).count() << " ms\n";
            return totalWeight;
        }

        for (const auto& [neighbor, _] : graph.at(current)) {
            if (!visited.count(neighbor)) {
                q.push(neighbor);
                parent[neighbor] = current;
            }
        }
    }
    cout << "BFS: Path not found.\n"; // O(1)
    return -1; // Возвращаем -1, если путь не найден
}

// Функция для DFS
double DFS(const Graph& graph, int start, int goal) {
    auto start_time = high_resolution_clock::now();

    stack<int> s; // V*4 байта
    set<int> visited; // максимум- V*4 байта
    map<int, int> parent; // максимум - V * 4*2 байта
    s.push(start);

    while (!s.empty()) {
        int current = s.top(); // 4 байта
        s.pop();

        if (visited.count(current)) continue;
        visited.insert(current);

        if (current == goal) {
            auto end_time = high_resolution_clock::now();
            double totalWeight = printPath(parent, start, goal, graph); // 8 байт
            cout << "DFS time: " << duration_cast<milliseconds>(end_time - start_time).count() << " ms\n";
            return totalWeight;
        }

        for (const auto& [neighbor, _] : graph.at(current)) {
            if (!visited.count(neighbor)) {
                s.push(neighbor);
                parent[neighbor] = current;
            }
        }
    }
    cout << "DFS: Path not found.\n";
    return -1;
}

// Функция для алгоритма Дейкстры
double Dijkstra(const Graph& graph, int startNode, int targetNode) {
    auto startTime = high_resolution_clock::now();

    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<>> minHeap; // O(V)
    map<int, double> shortestDistances; // O(V) в худшем случае
    map<int, int> predecessors; // O(V) в худшем случае

    // Инициализация расстояний до всех узлов бесконечностью
    for (const auto& node : graph) {
        shortestDistances[node.first] = numeric_limits<double>::infinity();
    }
    shortestDistances[startNode] = 0;

    minHeap.emplace(0, startNode);

    while (!minHeap.empty()) {
        auto [currentDistance, currentNode] = minHeap.top();
        minHeap.pop();

        if (currentNode == targetNode) {
            auto endTime = high_resolution_clock::now();
            double totalWeight = printPath(predecessors, startNode, targetNode, graph); // 8 байт
            cout << "Dijkstra time: "
                 << duration_cast<milliseconds>(endTime - startTime).count() << " ms\n";
            return totalWeight;
        }

        // Обход соседей текущего узла
        for (const auto& [neighbor, edgeWeight] : graph.at(currentNode)) {
            double potentialDistance = currentDistance + edgeWeight; // 8 байт

            if (potentialDistance < shortestDistances[neighbor]) {
                shortestDistances[neighbor] = potentialDistance;
                predecessors[neighbor] = currentNode;
                minHeap.emplace(potentialDistance, neighbor);
            }
        }
    }

    cout << "Dijkstra's algorithm: Path not found.\n";
    return -1; // Возвращаем -1, если путь не найден
}

// Функция для алгоритма A*
double AStar(const Graph& graph, int startNode, int goalNode, const Coordinates& coords) {
    auto startTime = high_resolution_clock::now();

    // Эвристическая функция для вычисления расстояния до цели
    auto heuristicFunc = [&](int node) {
        auto [longitude1, latitude1] = coords.at(node);
        auto [longitude2, latitude2] = coords.at(goalNode);
        return sqrt(pow(longitude1 - longitude2, 2) + pow(latitude1 - latitude2, 2));
    };

    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<>> openSet; // O(V)
    map<int, double> gCosts; // O(V) в худшем случае
    map<int, double> fCosts; // O(V) в худшем случае
    map<int, int> cameFrom; // O(V) в худшем случае

    // Инициализация стоимости g и f для всех узлов
    for (const auto& node : graph) {
        gCosts[node.first] = numeric_limits<double>::infinity();
        fCosts[node.first] = numeric_limits<double>::infinity();
    }
    gCosts[startNode] = 0;
    fCosts[startNode] = heuristicFunc(startNode);

    openSet.emplace(fCosts[startNode], startNode);

    while (!openSet.empty()) {
        auto [currentFCost, currentNode] = openSet.top();
        openSet.pop();

        if (currentNode == goalNode) {
            auto endTime = high_resolution_clock::now();
            double totalWeight = printPath(cameFrom, startNode, goalNode, graph); // 8 байт
            cout << "A* execution time: " << duration_cast<milliseconds>(endTime - startTime).count() << " ms\n";
            return totalWeight;
        }

        for (const auto& [neighbor, edgeWeight] : graph.at(currentNode)) {
            double tentativeGCost = gCosts[currentNode] + edgeWeight; // 8 байт

            if (tentativeGCost < gCosts[neighbor]) {
                gCosts[neighbor] = tentativeGCost;
                fCosts[neighbor] = tentativeGCost + heuristicFunc(neighbor);
                cameFrom[neighbor] = currentNode;
                openSet.emplace(fCosts[neighbor], neighbor);
            }
        }
    }
    cout << "A*: Path not found.\n";
    return -1; // Возвращаем -1, если путь не найден
}


int main() {
    string filename = "C:/Users/chuni/CLionProjects/polygon6/spb_graph.txt";

    Graph graph;
    Coordinates coordinates;
    parseGraph(filename, graph, coordinates);

    double start_lon = 30.331774, start_lat = 59.847451; // Ленсовета 23
    double goal_lon= 30.337795, goal_lat = 59.926835; // корпус на Ломоносова 9


    int start = findNearestNode(coordinates, start_lon, start_lat);
    int goal = findNearestNode(coordinates, goal_lon, goal_lat);

    if (start == -1 || goal == -1) {
        cerr << "Start or goal node not found by coordinates." << endl;
        return 1;
    }

    cout << "\nRunning BFS:\n";
    BFS(graph, start, goal);

    cout << "\nRunning DFS:\n";
    DFS(graph, start, goal);

    cout << "\nRunning Dijkstra:\n";
    Dijkstra(graph, start, goal);

    cout << "\nRunning A*:\n";
    AStar(graph, start, goal, coordinates);

    return 0;
}