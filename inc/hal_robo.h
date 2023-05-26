#ifndef HAL_ROBO_H
#define HAL_ROBO_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <util/delay.h>

/*
**  HAL_ROBO.C - HAL utk RoboTar with Atmel MEGA328P @16MHz
**
**  Copyright (C) Rosbi Mamat 2010. All right reserved.
**
**  Toolchains: WinAVR
**  Created: 06/09/2010
**  Modify: 05/05/2014 for RoboTar
**  Modify: 31/03/2023 for new sensors
**          Supported sensors:
**            proximity sensor (A0), Light sensor (A4) & 3 track sensors (A1, A2, A3)
*/

#define LINE_SENSOR_THRESH 300

void robo_Setup(void);
void robo_motorSpeed(int lspeed, int rspeed);
int  robo_proxSensor(void);
int  robo_lightSensor(void);
int  robo_lineSensor(void);
int  robo_bumpSensorR(void);
int  robo_bumpSensorL(void);
void robo_Honk(void);
void robo_checkBattery(void);
char robo_goPressed(void);
void robo_wait4goPress(void);

void OS_ticks_init( void );

void cputchar(unsigned char c);
unsigned char cgetchar( void );
void cputs(char *s);
int cprintf(const char *format, ... );

#define robo_LED_off()      PORTB &= ~0x01
#define robo_LED_on()       PORTB |= 0x01
#define robo_LED_toggle()   PORTB ^= (1 << 0)

#define RIGHT_MOTOR   0
#define LEFT_MOTOR    1
#define MOTOR_FORWARD 0
#define MOTOR_REVERSE 1

#define HIGH_SPEED     80
#define MEDIUM_SPEED   50
#define LOW_SPEED      30
#define STOP_SPEED      0

#define HIT 1

#endif
