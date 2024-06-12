/*
 * File:   perception_Connection.c
 * Author: hugofva
 *
 * Created on 10 de junio de 2024, 19:10
 */


#include "Perception_Connection.h"
#include "Map_Management.h"
#include "AuK.h"
#include <stdio.h>

// Semáforo para sincronización
Tsemaphore sensor_semaphore;
int sensor_distance;

void perception_connection_task(void) {
    // Inicializar el semáforo
    init_semaphore(&sensor_semaphore, 0);

    while (1) {
        // Solicitar una lectura de sensor en un ángulo determinado
        request_sensor_reading(90);

        // Esperar a que se complete la lectura del sensor
        wait(&sensor_semaphore);

        // Procesar los datos del sensor
        process_sensor_data(sensor_distance);

        // Delay para la siguiente lectura
        delay_until(clock() + 100); // Ajustar el delay según sea necesario
    }
}

void request_sensor_reading(int angle) {
    // Implementación de la solicitud de lectura del sensor
    // Enviar el ángulo al sistema de percepción
}

void process_sensor_data(int distance) {
    // Implementación del procesamiento de datos del sensor
    // Actualizar el mapa métrico con la información recibida
    int x = ...; // Calcular la posición x basada en la distancia y el ángulo
    int y = ...; // Calcular la posición y basada en la distancia y el ángulo
    update_metric_map(x, y, 1);
}

// Función para simular la recepción de datos del sensor
void receive_sensor_data(int distance) {
    sensor_distance = distance;
    signal(&sensor_semaphore);
}
