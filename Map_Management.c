/*
 * File:   Map_Management.c
 * Author: hugofva
 *
 * Created on 10 de junio de 2024, 19:04
 */

#include "Map_Management.h"
#include "Metric_Map.h"
#include "Topo_Map.h"
#include <stdio.h>
#include <string.h>

// Inicializar el mutex
Tmutex map_mutex;

// Inicializar los mapas
void init_maps(void) {
    init_metric_map();
    init_topological_map();
    mutex_init(&map_mutex, 1);  // Inicializar el mutex con la prioridad adecuada
}
void add_node(int id, char *name) {
    mutex_lock(&map_mutex);
    nodes[id].id = id;
    strcpy(nodes[id].name, name);
    nodes[id].num_connections = 0;
    //char buffer[100];
    //sprintf(buffer, "Nodo aniadido: %d - %s\n", id, name);
    //uart_write_string(buffer);
    mutex_unlock(&map_mutex);
}

void add_connection(int from, int to) {
    mutex_lock(&map_mutex);
    if (from >= MAX_NODES || to >= MAX_NODES) {
        uart_write_string("Error: indice de nodo fuera de rango\n");
        mutex_unlock(&map_mutex);
        return;
    }
    nodes[from].connections[nodes[from].num_connections++] = to;
    nodes[to].connections[nodes[to].num_connections++] = from; // Conexión bidireccional
    //char buffer[100];
    //sprintf(buffer, "Conexion aniadida: %d <-> %d\n", from, to);
    //uart_write_string(buffer);
}

// Funciones para manejar el mapa métrico
void update_metric_map(int x, int y, int value) {
    mutex_lock(&map_mutex);
    if (x >= 0 && x < MAP_SIZE && y >= 0 && y < MAP_SIZE) {
        metric_map[x][y] = value;
        char buffer[100];
        sprintf(buffer, "Mapa métrico actualizado en [%d, %d] con valor %d\n", x, y, value);
        uart_write_string(buffer);
    } else {
        uart_write_string("Error: Coordenadas fuera de los límites del mapa métrico\n");
    }
    mutex_unlock(&map_mutex);
}

// Actualización de la posición actual del robot
void update_robot_position(int x, int y) {
    mutex_lock(&map_mutex);
    Point new_position = {x, y};
    update_current_position(new_position);
    char buffer[100];
    sprintf(buffer, "Posición del robot actualizada a [%d, %d]\n", x, y);
    uart_write_string(buffer);
    mutex_unlock(&map_mutex);
}
