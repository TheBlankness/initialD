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
#define TASK_GAME_TIME     1


OS_STK TaskStartStk[TASK_STK_SZ];           /* TaskStartTask stack                      */
OS_STK ChkCollideStk[TASK_STK_SZ];          /* Task StopOnCollide stack                 */
OS_STK GameTimeTsk[TASK_STK_SZ];          /* Task StopOnCollide stack                 */
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

int getWeightValue(int line)
{
    int weights[] = {-1, 2000, 1000, 1500, 0, -2, 500, 1000};
    return weights[line];
}

#define CENTER_VALUE 1000   // The desired line sensor value for being centered on the line
#define KP 0.12             // Proportional control constant    0.002   // dejavu
#define KI 0.000007            // Integral control constant        0.005   // sharp turns
#define KD 0.02              // Derivative control constant      0.009   // smooth

 double integral = 0;    // Accumulated error
    double prevError = 0;   // Previous error
    int flag = 1;

void Navig(void *data)
{
    double base_KI = KI;
    double base_KP = KP;
    int base_speed = 50;
  double gameTime = 0;
   
    for (;;)
    {
        int line = robo_lineSensor();
        int lineSensorValue = getWeightValue(line);   // Read the line sensor value

        int error = lineSensorValue - CENTER_VALUE; // Calculate the error
        if (line == 0){
            if(gameTime<3){
                error = -1000;
            }else {
                error = 1000;
            }
        }

      

        integral += error;  // Accumulate the error over time

        // Derivative term calculation
        int derivative = error - prevError;
        prevError = error;

        // if (lineSensorValue == 1000) {
        //     integral = 0;
        // }

        if (myrobot.obstacle == 1) {
            // Obstacle detected, stop the car
            myrobot.rspeed = 0;
            myrobot.lspeed = 0;
            integral = 0;  // Reset the accumulated error
        }
        else {
            // Adjust motor speeds based on the error, accumulated error, and derivative
            int lspeed = base_speed + base_KP * error + base_KI * integral + KD * derivative;
            int rspeed = base_speed - base_KP * error - base_KI * integral - KD * derivative;

            if(lspeed>100){lspeed=100;}
            if(lspeed<-100){lspeed=-100;}
            
            if(rspeed>100){rspeed=100;}
            if(rspeed<-100){rspeed=-100;}

            myrobot.lspeed = lspeed;
            myrobot.rspeed = rspeed;
        }
gameTime = gameTime + 0.010;
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
                                    /* Show that we are alive        */
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
