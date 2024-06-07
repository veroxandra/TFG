/*
 * File:   Topo_Map.c
 * Author: hugofva
 *
 * Created on June 7, 2024, 7:54 PM
 */


#include "xc.h"

#define MAX_NODES 20
#define INF 9999

typedef struct {
    int id;
    char name[50];
    int num_connections;
    int connections[MAX_NODES];
} Node;

Node nodes[MAX_NODES];
int predecessor[MAX_NODES]; // Para rastrear los nodos predecesores

void uart_write_string(const char* str) {
    while (*str != '\0') {
        while (U1STAbits.UTXBF); // Wait while TX buffer is full
        U1TXREG = *str++;
    }
}

void add_node(int id, char *name) {
    nodes[id].id = id;
    strcpy(nodes[id].name, name);
    nodes[id].num_connections = 0;
    char buffer[100];
    sprintf(buffer, "Nodo aniadido: %d - %s\n", id, name);
    uart_write_string(buffer);
}

void add_connection(int from, int to) {
    if (from >= MAX_NODES || to >= MAX_NODES) {
        uart_write_string("Error: indice de nodo fuera de rango\n");
        return;
    }
    nodes[from].connections[nodes[from].num_connections++] = to;
    nodes[to].connections[nodes[to].num_connections++] = from; // Conexión bidireccional
    char buffer[100];
    sprintf(buffer, "Conexion aniadida: %d <-> %d\n", from, to);
    uart_write_string(buffer);
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

void print_path(int start, int goal) {
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
}

void a_star(int start, int goal) {
    int open_list[MAX_NODES];
    int closed_set[MAX_NODES] = {0};
    int g_cost[MAX_NODES];
    int f_cost[MAX_NODES];
    int open_count = 0;

    for (int i = 0; i < MAX_NODES; i++) {
        g_cost[i] = INF;
        f_cost[i] = INF;
        predecessor[i] = -1;
    }

    open_list[open_count++] = start;
    g_cost[start] = 0;
    f_cost[start] = heuristic(start);

    while (open_count > 0) {
        int current = find_lowest_f(open_list, f_cost, open_count);

        // Eliminar nodo actual de la lista abierta
        for (int i = 0; i < open_count; i++) {
            if (open_list[i] == current) {
                open_list[i] = open_list[--open_count];
                break;
            }
        }
        closed_set[current] = 1;

        if (current == goal) {
            uart_write_string("Ruta encontrada\n");
            print_path(start, goal);
            return;
        }

        for (int i = 0; i < nodes[current].num_connections; i++) {
            int neighbor = nodes[current].connections[i];

            if (closed_set[neighbor]) continue;

            int tentative_g_cost = g_cost[current] + 1;

            if (tentative_g_cost < g_cost[neighbor]) {
                g_cost[neighbor] = tentative_g_cost;
                f_cost[neighbor] = g_cost[neighbor] + heuristic(neighbor);
                predecessor[neighbor] = current;

                int in_open = 0;
                for (int j = 0; j < open_count; j++) {
                    if (open_list[j] == neighbor) {
                        in_open = 1;
                        break;
                    }
                }
                if (!in_open) {
                    open_list[open_count++] = neighbor;
                }
            }
        }
    }

    uart_write_string("No se encontro una ruta\n");
}

//Tests a poner en otro lao
void test_scenario_7() {
    uart_write_string("Escenario 7: Ciclo\n");
    add_node(0, "Nodo 0");
    add_node(1, "Nodo 1");
    add_node(2, "Nodo 2");
    add_node(3, "Nodo 3");

    add_connection(0, 1);
    add_connection(1, 2);
    add_connection(2, 0); // Crear un ciclo
    add_connection(2, 3);

    a_star(0, 3);
}

void test_scenario_8() {
    uart_write_string("Escenario 8: Nodos Desconectados\n");
    add_node(0, "Nodo 0");
    add_node(1, "Nodo 1");
    add_node(2, "Nodo 2");
    add_node(3, "Nodo 3");
    add_node(4, "Nodo 4");

    add_connection(0, 1);
    add_connection(1, 2);
    // Nodo 3 y Nodo 4 están desconectados

    a_star(0, 4);
}

void test_scenario_9() {
    uart_write_string("Escenario 9: Caminos Complejos\n");
    for (int i = 0; i < 10; i++) {
        char name[10];
        sprintf(name, "Nodo %d", i);
        add_node(i, name);
    }

    add_connection(0, 1);
    add_connection(0, 2);
    add_connection(1, 3);
    add_connection(1, 4);
    add_connection(2, 5);
    add_connection(2, 6);
    add_connection(3, 7);
    add_connection(4, 7);
    add_connection(5, 8);
    add_connection(6, 8);
    add_connection(7, 9);
    add_connection(8, 9);

    a_star(0, 9);
}

void test_scenario_10() {
    uart_write_string("Escenario 10: Caminos Alternativos\n");
    for (int i = 0; i < 10; i++) {
        char name[10];
        sprintf(name, "Nodo %d", i);
        add_node(i, name);
    }

    add_connection(0, 1);
    add_connection(0, 2);
    add_connection(1, 3);
    add_connection(1, 4);
    add_connection(2, 5);
    add_connection(2, 6);
    add_connection(3, 7);
    add_connection(4, 7);
    add_connection(5, 8);
    add_connection(6, 8);
    add_connection(7, 9);
    add_connection(8, 9);

    // Agregar conexiones alternativas
    add_connection(0, 3);
    add_connection(1, 5);
    add_connection(4, 6);
    add_connection(3, 8);

    a_star(0, 9);
}
