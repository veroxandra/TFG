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

// Sem�foro para sincronizaci�n
Tsemaphore sensor_semaphore;
int sensor_distance;

void perception_connection_task(void) {
    // Inicializar el sem�foro
    init_semaphore(&sensor_semaphore, 0);

    while (1) {
        // Solicitar una lectura de sensor en un �ngulo determinado
        request_sensor_reading(90);

        // Esperar a que se complete la lectura del sensor
        wait(&sensor_semaphore);

        // Procesar los datos del sensor
        process_sensor_data(sensor_distance);

        // Delay para la siguiente lectura
        delay_until(clock() + 100); // Ajustar el delay seg�n sea necesario
    }
}

void request_sensor_reading(int angle) {
    // Implementaci�n de la solicitud de lectura del sensor
    // Enviar el �ngulo al sistema de percepci�n
}

void process_sensor_data(int distance) {
    // Implementaci�n del procesamiento de datos del sensor
    // Actualizar el mapa m�trico con la informaci�n recibida
    int x = ...; // Calcular la posici�n x basada en la distancia y el �ngulo
    int y = ...; // Calcular la posici�n y basada en la distancia y el �ngulo
    update_metric_map(x, y, 1);
}

// Funci�n para simular la recepci�n de datos del sensor
void receive_sensor_data(int distance) {
    sensor_distance = distance;
    signal(&sensor_semaphore);
}
