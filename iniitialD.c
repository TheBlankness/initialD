/*
 *   ROBOSAMPLE.C -- A sample/template for RoboKar program with uCOS-II
 *   Written by: Rosbi Mamat 6/5/2014
 *   Updated : 1/5/2023 Modified to show proximity & light sensor usage
 */

#include ".\inc\kernel.h"                  /* Always include these to use uCOS-II      */
#include ".\inc\hal_robo.h"                /*   and RoboKar HAL                        */

#define TASK_STK_SZ            128          /* Size of each task's stacks (# of bytes)  */
#define TASK_START_PRIO          1          /* Highest priority                         */
#define TASK_CHKCOLLIDE_PRIO     2
#define TASK_CTRLMOTOR_PRIO      3
#define TASK_NAVIG_PRIO          4          /* Lowest priority                          */

OS_STK TaskStartStk[TASK_STK_SZ];           /* TaskStartTask stack                      */
OS_STK ChkCollideStk[TASK_STK_SZ];          /* Task StopOnCollide stack                 */
OS_STK CtrlmotorStk[TASK_STK_SZ];           /* Task CtrlMotors stack                    */
OS_STK NavigStk[TASK_STK_SZ];               /* Task NavigRobot stack                    */

/* ------ Global shared variable -------*/
/* Ideally, this should be protected by a semaphore etc */
struct robostate
{
    int rspeed;                             /* right motor speed  (-100 -- +100)        */
    int lspeed;                             /* leftt motor speed  (-100 -- +100)        */
    char obstacle;                          /* obstacle? 1 = yes, 0 = no                */
} myrobot;

/*------High pririority task----------*/
void CheckCollision (void *data)
{
    for(;;)
    {
        if ( (robo_proxSensor() == 1) )             /* obstacle?                         */
            myrobot.obstacle = 1;                   /* signal obstacle present           */
        else
            myrobot.obstacle = 0;                   /* signal no obstacle                */

		OSTimeDlyHMSM(0, 0, 0, 100);                /* Task period ~ 100 ms              */
    }
}

/* Control robot Motors TASK */
void CntrlMotors (void *data)
{
    int speed_r, speed_l;

    for(;;)
    {
        speed_r = myrobot.rspeed;
        speed_l = myrobot.lspeed;
        robo_motorSpeed(speed_l, speed_r);
        OSTimeDlyHMSM(0, 0, 0, 10);                /* Task period ~ 250 ms              */
    }
}

/* --- Task for navigating robot ----
 * Write you own navigation task here
 */

int getWeightValue(int line) {  
    int mid = 1000; 
    int left = 0; 
    int right = 2000; 
    int value = 0; 
    int active = 0;
 
    switch (line) { 
        case 0:  
            active = 1; 
            value = right; 
            break; 
        case 1:  
            active = 1; 
            value = right; 
            break; 
        case 2:  
            active = 1; 
            value = mid; 
            break; 
        case 3:  
            active = 2; 
            value = mid + right; 
            break; 
        case 4: 
            active = 1;  
            value = left; 
            break; 
        case 6:  
            active = 2; 
            value = mid + left; 
            break; 
        case 7:  
            active = 3; 
            value = mid+left+right; 
            break;
        default:
            value=0;
            active=0;
            break;
    } 
 
    if (active==0)
    {
        return 0;
    }
    
    return value / active; 
}

#define CENTER_VALUE 1000   // The desired line sensor value for being centered on the line
#define KP 0.0075           // Proportional control constant
#define KI 0.0005           // Integral control constant
#define KD 0.001            // Derivative control constant

void Navig(void *data)
{
    int integral = 0;    // Accumulated error
    int prevError = 0;   // Previous error

    for (;;)
    {
        int lineSensorValue = getWeightValue(robo_lineSensor());   // Read the line sensor value

        int error = lineSensorValue - CENTER_VALUE; // Calculate the error
        integral += error;  // Accumulate the error over time

        // Derivative term calculation
        int derivative = error - prevError;
        prevError = error;

        if (lineSensorValue == 1000) {
            integral = 0;
        }

        if (myrobot.obstacle == 1) {
            // Obstacle detected, stop the car
            myrobot.rspeed = 0;
            myrobot.lspeed = 0;
            integral = 0;  // Reset the accumulated error
        }
        else {
            // Adjust motor speeds based on the error, accumulated error, and derivative
            myrobot.lspeed = MEDIUM_SPEED + KP * error + KI * integral + KD * derivative;
            myrobot.rspeed = MEDIUM_SPEED - KP * error - KI * integral - KD * derivative;
        }

        OSTimeDlyHMSM(0, 0, 0, 10);   // Task period ~ 100 ms
    }
}





/*------Highest pririority task----------*/
/* Create all other tasks here           */
void TaskStart( void *data )
{
    OS_ticks_init();                                        /* enable RTOS timer tick        */

    OSTaskCreate(CheckCollision,                            /* Task function                 */
                (void *)0,                                  /* nothing passed to task        */
                (void *)&ChkCollideStk[TASK_STK_SZ - 1],    /* stack allocated to task       */
                TASK_CHKCOLLIDE_PRIO);                      /* priority of task              */

    OSTaskCreate(CntrlMotors,                               /* Task function                 */
                (void *)0,                                  /* nothing passed to task        */
                (void *)&CtrlmotorStk[TASK_STK_SZ - 1],     /* stack allocated to task       */
                TASK_CTRLMOTOR_PRIO);                       /* priority of task              */

    OSTaskCreate(Navig,                                     /* Task function                 */
                (void *)0,                                  /* nothing passed to task        */
                (void *)&NavigStk[TASK_STK_SZ - 1],         /* stack allocated to task       */
                TASK_NAVIG_PRIO);                           /* priority of task              */

    while(1)
    {
        OSTimeDlyHMSM(0, 0, 5, 0);                          /* Task period ~ 5 secs          */
        robo_LED_toggle();                                  /* Show that we are alive        */
    }

}

int main( void )
{
    robo_Setup();                                          /* initialize HAL for RoboKar     */
    OSInit();                                              /* initialize UCOS-II kernel      */

    robo_motorSpeed(STOP_SPEED, STOP_SPEED);               /* Stop the robot                 */
    myrobot.rspeed   = STOP_SPEED;                         /* Initialize myrobot states      */
    myrobot.lspeed   = STOP_SPEED;
    myrobot.obstacle = 0;                                  /*  No collisioin                 */

    OSTaskCreate(TaskStart,                                /* create TaskStart Task          */
                (void *)0,
                (void *)&TaskStartStk[TASK_STK_SZ - 1],
                TASK_START_PRIO);
	robo_Honk(); robo_wait4goPress();                      /* Wait for to GO                 */
    OSStart();                                             /* Start multitasking             */
    while (1);                                             /* die here                       */
}


