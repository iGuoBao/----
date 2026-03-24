#ifndef __SERVOS_H
#define __SERVOS_H

#define NONE  0
#define UP    1
#define DOWN  2

#define OPEN  1
#define CLOSE 2
void Servos_Init(void);
void Servos_Lift(int lift);
void Servos_Retract(int retract);
void Servos_up(int position);
void Servos_down(int position);
void Servos_close(int position);
void Servos_open(int position);
#endif