/*
 * File:   Topo_Map.c
 * Author: hugofva
 *
 * Created on 10 de junio de 2024, 20:22
 */

#include "Topo_Map.h"
#include <string.h>
#include <stdio.h>

typedef struct {
    int id;
    char name[50];
    int num_connections;
    int connections[MAX_NODES];
} Node;

// Variables globales internas
static int predecessor[MAX_NODES]; // Para rastrear los nodos predecesores
int final_path[MAX_NODES];
int final_path_length = 0;

// Variables globales compartidas
int final_path[MAX_NODES];
int final_path_length = 0;
Node nodes[MAX_NODES];

void init_topological_map(void)
{

}

void uart_write_string(const char* str) {
    while (*str != '\0') {
        while (U1STAbits.UTXBF); // Wait while TX buffer is full
        U1TXREG = *str++;
    }
}

int heuristic(int node) {
    int count = 0;
    while (node != -1) {
        node = predecessor[node];
        count++;
    }
    return count - 1; // Restar uno para no contar el nodo inicial
}

int find_lowest_f(int open_list[], int f_cost[], int open_count) {
    int min_index = open_list[0];
    int min_value = f_cost[min_index];
    for (int i = 1; i < open_count; i++) {
        int node_index = open_list[i];
        if (f_cost[node_index] < min_value) {
            min_value = f_cost[node_index];
            min_index = node_index;
        }
    }
    return min_index;
}

void store_path(int path[], int length) {
    final_path_length = length;
    for (int i = 0; i < length; i++) {
        final_path[i] = path[i];
    }
}

void save_path(int start, int goal) {
    int path[MAX_NODES];
    int path_length = 0;
    int current = goal;

    while (current != start) {
        path[path_length++] = current;
        current = predecessor[current];
    }
    path[path_length++] = start;

    uart_write_string("Camino encontrado: ");
    for (int i = path_length - 1; i >= 0; i--) {
        char buffer[10];
        sprintf(buffer, "%d ", path[i]);
        uart_write_string(buffer);
    }
    uart_write_string("\n");

    store_path(path, path_length);
}

void a_star_topological(int start, int goal) 
{
    int open_list[MAX_NODES];
    int closed_set[MAX_NODES] = {0};
    int g_cost[MAX_NODES];
    int f_cost[MAX_NODES];
    int open_count = 0;

    for (int i = 0; i < MAX_NODES; i++) 
    {
        g_cost[i] = INF;
        f_cost[i] = INF;
        predecessor[i] = -1;
    }

    open_list[open_count++] = start;
    g_cost[start] = 0;
    f_cost[start] = heuristic(start);

    while (open_count > 0) 
    {
        int current = find_lowest_f(open_list, f_cost, open_count);

        // Eliminar nodo actual de la lista abierta
        for (int i = 0; i < open_count; i++) 
        {
            if (open_list[i] == current) 
            {
                open_list[i] = open_list[--open_count];
                break;
            }
        }
        closed_set[current] = 1;

        if (current == goal) 
        {
            uart_write_string("Ruta encontrada\n");
            save_path(start, goal);
            return;
        }

        for (int i = 0; i < nodes[current].num_connections; i++) 
        {
            int neighbor = nodes[current].connections[i];

            if (closed_set[neighbor]) continue;

            int tentative_g_cost = g_cost[current] + 1;

            if (tentative_g_cost < g_cost[neighbor]) 
            {
                g_cost[neighbor] = tentative_g_cost;
                f_cost[neighbor] = g_cost[neighbor] + heuristic(neighbor);
                predecessor[neighbor] = current;

                int in_open = 0;
                for (int j = 0; j < open_count; j++) 
                {
                    if (open_list[j] == neighbor) 
                    {
                        in_open = 1;
                        break;
                    }
                }
                if (!in_open) 
                {
                    open_list[open_count++] = neighbor;
                }
            }
        }
    }

    uart_write_string("No se encontro una ruta\n");
}