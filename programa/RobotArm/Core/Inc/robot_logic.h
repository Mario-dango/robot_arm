/*
 * robot_logic.h
 *
 *  Created on: Jan 4, 2026
 *      Author: mprob
 */
#ifndef ROBOT_LOGIC_H
#define ROBOT_LOGIC_H

#include "main.h"

// Prototipos de las funciones de lógica
void Robot_Consignas(void);          // Ayuda CAPA 1: lista completa de comandos
void Robot_ComandoEjemplo(char c);   // Ayuda CAPA 2: ejemplo de un comando puntual
uint8_t Robot_ModoCalibracion(void);
uint8_t Robot_ModoAprendizaje(void);
uint8_t Robot_ModoEjecucion(void);
uint8_t Robot_ModoTest(void);        // Modo test/diagnóstico (prefijo :*)
void Robot_UpdateTelemetry(void);

// Reporta por USB el origen del último evento de interrupción (botón E-STOP,
// comando :-S o fin de carrera X/Y/Z). Se llama desde el bucle principal.
void Robot_ReportInterrupts(void);



// Función maestra que decide qué hacer según el comando recibido
void Robot_ProcesarComando(char *cmd);

#endif /* ROBOT_LOGIC_H */
