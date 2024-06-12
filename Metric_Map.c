/*
 * File:   Metric_Map.c
 * Author: hugofva
 *
 * Created on 10 de junio de 2024, 20:50
 */


#include "Metric_Map.h"
#include <math.h>
#include <stdio.h>

typedef struct {
    int x;
    int y;
} Point;

// Mapa métrico global
int metric_map[MAP_SIZE][MAP_SIZE];
Point current_position;  // Posición actual del robot {x, y}
Point metric_path[MAP_SIZE * MAP_SIZE];  // Ruta métrica
int metric_path_length = 0;  // Longitud de la ruta métrica

// Inicialización del mapa métrico
void init_metric_map(void) {
    // Inicializar el mapa con valores apropiados
    for (int i = 0; i < MAP_SIZE; i++) {
        for (int j = 0; j < MAP_SIZE; j++) {
            metric_map[i][j] = -1;  // Por defecto, -1 indica que se desconoce
        }
    }

    // Inicializar nodos y áreas intransitables aquí
    // ...
}

// Actualizar la posición actual del robot
void update_current_position(Point position) {
    current_position = position;
}

// Heurística de distancia euclídea
int heuristic_metric(Point a, Point b) {
    return (int) sqrt(pow(b.x - a.x, 2) + pow(b.y - a.y, 2));
}

// Encontrar el nodo con el menor costo f en la lista abierta
int find_lowest_f_metric(Point open_list[], int f_cost[MAP_SIZE][MAP_SIZE], int open_count) {
    int min_index = 0;
    int min_value = INF;

    for (int i = 0; i < open_count; i++) {
        int x = open_list[i].x;
        int y = open_list[i].y;
        if (f_cost[x][y] < min_value) {
            min_value = f_cost[x][y];
            min_index = i;
        }
    }
    return min_index;
}

// Almacenar la ruta métrica
void store_metric_path(Point path[], int length) {
    metric_path_length = length;
    for (int i = 0; i < length; i++) {
        metric_path[i] = path[i];
    }
}

/*void print_path_metric(Point start, Point goal) {
    int path[MAX_NODES * 10];
    int path_length = 0;
    Point current = goal;

    while (current.x != start.x || current.y != start.y) {
        path[path_length++] = current.x * MAP_SIZE + current.y;
        current = store_metric_path[path_length];
    }
    path[path_length++] = start.x * MAP_SIZE + start.y;

    uart_write_string("Camino encontrado (métrico): ");
    for (int i = path_length - 1; i >= 0; i--) {
        char buffer[10];
        sprintf(buffer, "(%d, %d) ", path[i] / MAP_SIZE, path[i] % MAP_SIZE);
        uart_write_string(buffer);
    }
    uart_write_string("\n");

    store_metric_path(path, path_length);
}*/

// A* para el mapa métrico
void a_star_metric(Point start, Point goal, int current_node, int next_node) {
    Point open_list[MAP_SIZE * MAP_SIZE];  // Lista abierta
    int closed_set[MAP_SIZE][MAP_SIZE] = {0};  // Conjunto cerrado
    int g_cost[MAP_SIZE][MAP_SIZE];  // Costo g
    int f_cost[MAP_SIZE][MAP_SIZE];  // Costo f
    int open_count = 0;  // Contador de la lista abierta
    Point came_from[MAP_SIZE][MAP_SIZE];  // Para rastrear la ruta

    // Inicializar costos
    for (int i = 0; i < MAP_SIZE; i++) {
        for (int j = 0; j < MAP_SIZE; j++) {
            g_cost[i][j] = INF;
            f_cost[i][j] = INF;
            came_from[i][j].x = -1;
            came_from[i][j].y = -1;
        }
    }

    // Añadir nodo inicial a la lista abierta
    open_list[open_count] = start;
    open_count++;
    g_cost[start.x][start.y] = 0;
    f_cost[start.x][start.y] = heuristic_metric(start, goal);

    while (open_count > 0) {
        // Encontrar el nodo con el menor costo f en la lista abierta
        int current_index = find_lowest_f_metric(open_list, f_cost, open_count);
        Point current = open_list[current_index];

        // Si llegamos al nodo objetivo, reconstruimos el camino
        if (current.x == goal.x && current.y == goal.y) {
            Point path[MAP_SIZE * MAP_SIZE];
            int path_length = 0;
            while (!(current.x == start.x && current.y == start.y)) {
                path[path_length++] = current;
                Point temp = came_from[current.x][current.y];
                current = temp;
            }
            path[path_length++] = start;

            // Invertir el camino para que esté en el orden correcto
            for (int i = 0; i < path_length / 2; i++) {
                Point temp = path[i];
                path[i] = path[path_length - i - 1];
                path[path_length - i - 1] = temp;
            }

            // Almacenar la ruta métrica
            store_metric_path(path, path_length);
            return;
        }

        // Eliminar el nodo actual de la lista abierta y añadirlo al conjunto cerrado
        for (int i = current_index; i < open_count - 1; i++) {
            open_list[i] = open_list[i + 1];
        }
        open_count--;
        closed_set[current.x][current.y] = 1;

        // Evaluar nodos vecinos
        Point directions[4] = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};
        for (int i = 0; i < 4; i++) {
            Point neighbor = {current.x + directions[i].x, current.y + directions[i].y};

            // Asegurarse de que el vecino está dentro de los límites del mapa
            if (neighbor.x < 0 || neighbor.x >= MAP_SIZE || neighbor.y < 0 || neighbor.y >= MAP_SIZE) {
                continue;
            }

            // Ignorar los vecinos que no son transitables o que ya están en el conjunto cerrado
            if (metric_map[neighbor.x][neighbor.y] == 0 || closed_set[neighbor.x][neighbor.y]) {
                continue;
            }

            int tentative_g_cost = g_cost[current.x][current.y] + 1;  // Cada paso tiene un costo de 1

            if (tentative_g_cost < g_cost[neighbor.x][neighbor.y]) {
                came_from[neighbor.x][neighbor.y] = current;
                g_cost[neighbor.x][neighbor.y] = tentative_g_cost;
                f_cost[neighbor.x][neighbor.y] = g_cost[neighbor.x][neighbor.y] + heuristic_metric(neighbor, goal);

                // Añadir el vecino a la lista abierta si no está ya en ella
                int in_open_list = 0;
                for (int j = 0; j < open_count; j++) {
                    if (open_list[j].x == neighbor.x && open_list[j].y == neighbor.y) {
                        in_open_list = 1;
                        break;
                    }
                }
                if (!in_open_list) {
                    open_list[open_count] = neighbor;
                    open_count++;
                }
            }
        }
    }
    // Si llegamos aquí, no se encontró un camino
    uart_write_string("No se encontró una ruta en el mapa métrico\n");
}
