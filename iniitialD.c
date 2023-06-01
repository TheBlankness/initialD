/*
 *   initialD.C -- RoboKar program with uCOS-II
 *   Written by: Ahmad Hazim
 *   Updated : 6/1/2023
 */

#include ".\inc\kernel.h"   /* Always include these to use uCOS-II      */
#include ".\inc\hal_robo.h" /*   and RoboKar HAL                        */

#define TASK_STK_SZ 128   /* Size of each task's stacks (# of bytes)  */
#define TASK_START_PRIO 1 /* Highest priority                         */
#define TASK_CHKCOLLIDE_PRIO 2
#define TASK_CTRLMOTOR_PRIO 3
#define TASK_NAVIG_PRIO 4 /* Lowest priority                          */

OS_STK TaskStartStk[TASK_STK_SZ];  /* TaskStartTask stack                      */
OS_STK ChkCollideStk[TASK_STK_SZ]; /* Task StopOnCollide stack                 */
OS_STK CtrlmotorStk[TASK_STK_SZ];  /* Task CtrlMotors stack                    */
OS_STK NavigStk[TASK_STK_SZ];      /* Task NavigRobot stack                    */

/* ------ Global shared variable -------*/
/* Ideally, this should be protected by a semaphore etc */
struct robostate
{
    int rspeed;    /* right motor speed  (-100 -- +100)        */
    int lspeed;    /* left motor speed  (-100 -- +100)         */
    char obstacle; /* obstacle? 1 = yes, 0 = no                */
    float kp;      /* proportional gain                        */
    float ki;      /* integral gain                            */
    float kd;      /* derivative gain                          */
} myrobot;

/*------High priority task----------*/
void CheckCollision(void *data)
{
    for (;;)
    {
        if (robo_proxSensor() == 1) /* obstacle?                         */
            myrobot.obstacle = 1;   /* signal obstacle present           */
        else
            myrobot.obstacle = 0; /* signal no obstacle                */

        OSTimeDlyHMSM(0, 0, 0, 100); /* Task period ~ 100 ms              */
    }
}

/* Control robot Motors TASK */
void CntrlMotors(void *data)
{
    int speed_r, speed_l;

    for (;;)
    {
        speed_r = myrobot.rspeed;
        speed_l = myrobot.lspeed;
        robo_motorSpeed(speed_l, speed_r);
        OSTimeDlyHMSM(0, 0, 0, 10); /* Task period ~ 250 ms              */
    }
}

/* --- Task for navigating robot ----
 * Write your own navigation task here
 */

iint getWeightValue(int line)
{
    int weights[] = {-1, 2000, 1000, 1500, 0, -2, 500, 2001};
    return weights[line];
}

#define CENTER_VALUE 1000 // The desired line sensor value for being centered on the line

void Navig(void *data)
{
    int integral = 0;  // Accumulated error
    int prevError = 0; // Previous error

    for (;;)
    {
        int lineSensorValue = getWeightValue(robo_lineSensor()); // Read the line sensor value

        int error = lineSensorValue - CENTER_VALUE; // Calculate the error
        integral += error;                          // Accumulate the error over time

        // Derivative term calculation
        int derivative = error - prevError;
        prevError = error;

        if (lineSensorValue == 1000)
        {
            integral = 0;
        }

        if (myrobot.obstacle == 1)
        {
            // Obstacle detected, stop the robot
            myrobot.rspeed = STOP_SPEED;
            myrobot.lspeed = STOP_SPEED;
        }
        else
        {
            // Adjust motor speeds based on the error and PID gains
            myrobot.rspeed = HIGH_SPEED - myrobot.kp * error - myrobot.ki * integral - myrobot.kd * derivative;
            myrobot.lspeed = HIGH_SPEED + myrobot.kp * error + myrobot.ki * integral + myrobot.kd * derivative;
        }

        OSTimeDlyHMSM(0, 0, 0, 10); /* Task period ~ 250 ms              */
    }
}

int main(void)
{
    OSInit(); /* Initialize uC/OS-II                  */

    /* ------- Create tasks -------*/
    OSTaskCreate(TaskStart, (void *)0, &TaskStartStk[TASK_STK_SZ - 1], TASK_START_PRIO);
    OSTaskCreate(CheckCollision, (void *)0, &ChkCollideStk[TASK_STK_SZ - 1], TASK_CHKCOLLIDE_PRIO);
    OSTaskCreate(CntrlMotors, (void *)0, &CtrlmotorStk[TASK_STK_SZ - 1], TASK_CTRLMOTOR_PRIO);
    OSTaskCreate(Navig, (void *)0, &NavigStk[TASK_STK_SZ - 1], TASK_NAVIG_PRIO);

    OSStart(); /* Start multitasking                   */
    return 0;
}

void TaskStart(void *data)
{
    robo_Setup();                /* initialize the robot                 */
    myrobot.rspeed = HIGH_SPEED; /* set initial motor speed              */
    myrobot.lspeed = HIGH_SPEED;
    myrobot.kp = 0.1; /* initial PID gains                    */
    myrobot.ki = 0.01;
    myrobot.kd = 0.05;

    for (;;)
    {
        // Change robot state here based on desired behavior
        int robotState = state1_following_straight_line; // Example: state1_following_straight_line

        // Update PID gains based on robot state
        switch (robotState)
        {
        case state1_following_straight_line:
            myrobot.kp = 0.1;
            myrobot.ki = 0.01;
            myrobot.kd = 0.05;
            break;
        case state2_turning_right:
            myrobot.kp = 0.2;
            myrobot.ki = 0.02;
            myrobot.kd = 0.1;
            break;
            // Add more cases for other states
        }

        OSTimeDlyHMSM(0, 0, 1, 0); /* Task period ~ 1s               */
    }
}