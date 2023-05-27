void Navig(void *data)
{
    double Kp = 1.0;  // Proportional gain
    double Ki = 0.5;  // Integral gain
    double Kd = 0.2;  // Derivative gain
    double error = 0.0;
    double lastError = 0.0;
    double integral = 0.0;
    int desiredvalue = 1000;

    for (;;)
    {
        if (myrobot.obstacle == 1)  // If blocked, then reverse
        {
            myrobot.rspeed = -LOW_SPEED;
            myrobot.lspeed = -LOW_SPEED;
            integral = 0.0;  // Reset integral term
        }
        else  // No obstacle detected
        {
            // Calculate error
            error = ...;  // Calculate the error based on your specific scenario

            // Calculate PID terms
            double P = error;
            integral += error;
            double D = error - lastError;
            lastError = error;

            // Calculate motor speed using PID terms
            int motorspeed = P * Kp + integral * Ki + D * Kd;

            // Set motor speeds
            myrobot.rspeed = motorspeed;
            myrobot.lspeed = motorspeed;
        }

        // Additional code implementation here...

        OSTimeDlyHMSM(0, 0, 0, 10);  // Task period ~ 10 ms
    }
}





////shit


int getWeightValue(int line) {  
    int mid = 1000; 
    int left = 0; 
    int right = 2000; 
    int value = 0; 
    int active = 0;
 
    switch (line) { 
        case 0:  
            active = 0; 
            value = 0; 
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
            value = mid; 
            break; 
    } 
 
    return value / active; 
}

#define CENTER_VALUE 1000   // The desired line sensor value for being centered on the line
#define KP 10           // Proportional gain
#define KI 1           // Integral gain
#define KD 1           // Derivative gain

void Navig(void *data)
{
    int previousError = 0;   // Previous error for calculating the derivative term
    int integral = 0;       // Accumulated error for the integral term

    for (;;)
    {
        int lineSensorValue = getWeightValue(robo_lineSensor());   // Read the line sensor value

        int error = lineSensorValue - CENTER_VALUE; // Calculate the error

        integral += error;   // Accumulate the error for the integral term

        // Calculate the control action using PID formula
        int controlAction = KP * error + KI * integral + KD * (error - previousError);

        previousError = error;   // Update the previous error for the next iteration

        // Adjust motor speeds based on the control action
        if (error > 0) {
            // Error is positive, go right
            myrobot.rspeed = MEDIUM_SPEED;
            myrobot.lspeed = MEDIUM_SPEED - controlAction;
        } else if (error < 0) {
            // Error is negative, go left
            myrobot.rspeed = MEDIUM_SPEED + controlAction;
            myrobot.lspeed = MEDIUM_SPEED;
        } else {
            // Error is zero, stay straight
            myrobot.rspeed = MEDIUM_SPEED;
            myrobot.lspeed = MEDIUM_SPEED;
        }

        if (myrobot.obstacle == 1) {
            // Obstacle detected, stop the car
            myrobot.rspeed = 0;
            myrobot.lspeed = 0;
        }

        OSTimeDlyHMSM(0, 0, 0, 50);   // Task period ~ 50 ms
    }
}