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
