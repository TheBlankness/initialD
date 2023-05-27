/* FollowLine task */
void FollowLine(void *data)
{
    for (;;)
    {
        int brightness = robo_lightSensor();  // Read brightness from the light sensor

        if (brightness > 60)  // If the sensor reading is above a threshold, robot turns right
        {
            myrobot.rspeed = -LOW_SPEED;
            myrobot.lspeed = LOW_SPEED;
        }
        else  // If the sensor reading is below the threshold, robot turns left
        {
            myrobot.rspeed = LOW_SPEED;
            myrobot.lspeed = -LOW_SPEED;
        }

        OSTimeDlyHMSM(0, 0, 0, 100);  // Task period ~ 100 ms
    }
}


void Navig(void *data)
{
    for (;;)
    {
        if (myrobot.obstacle == 1)  // If blocked, reverse at a slower speed
        {
            myrobot.rspeed = -LOW_SPEED;
            myrobot.lspeed = -LOW_SPEED;
        }
        else  // If no obstacle, move forward at a higher speed
        {
            myrobot.rspeed = MEDIUM_SPEED;
            myrobot.lspeed = MEDIUM_SPEED;
        }

        if (robo_lightSensor() > 60)  // If it's too bright, turn right at a slower speed
        {
            myrobot.rspeed = -LOW_SPEED;
            myrobot.lspeed = LOW_SPEED;
        }

        OSTimeDlyHMSM(0, 0, 0, 500);  // Task period ~ 500 ms
    }
}

void FollowLine(void *data);

OSTaskCreate(FollowLine,                              /* Task function                 */
            (void *)0,                                /* nothing passed to task        */
            (void *)&FollowLineStk[TASK_STK_SZ - 1],  /* stack allocated to task       */
            TASK_FOLLOWLINE_PRIO);                     /* priority of task              */
