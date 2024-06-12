/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/*
 * File:   Metric_Map.c
 * Author: hugofva
 *
 * Created on 10 de junio de 2024, 20:50
 */

#ifndef METRIC_MAP_H
#define METRIC_MAP_H

#include "Topo_Map.h"

#define MAP_SIZE 50  // Tamaño del mapa
#define CELL_SIZE 0.4 //Tamaño celdas

extern int metric_map[MAP_SIZE][MAP_SIZE];  // Mapa métrico
extern Point metric_path[MAP_SIZE * MAP_SIZE];  // Ruta métrica
extern int metric_path_length;  // Longitud de la ruta métrica

extern int final_path[MAX_NODES];  // Ruta topológica global
extern int final_path_length;  // Longitud de la ruta topológica global

void init_metric_map(void);
void a_star_metric(int start_x, int start_y, int goal_x, int goal_y, int current_node, int next_node);
void update_current_position(int x, int y);

#endif // METRIC_MAP_H