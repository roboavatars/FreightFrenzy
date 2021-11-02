package org.firstinspires.ftc.teamcode.Pathing.Ramsete;

public class VeloPIDController {
    boolean firstLoop = true;
    double lastError;
    double lastTime;
    double integral;

    public double output(double currentVelo, double targetVelo, double Kp, double Ki, double Kd, double time){
        double error = targetVelo - currentVelo;
        if (firstLoop){
            firstLoop = false;
            lastError = 0;
            lastTime = time;
            integral = 0;
        }

        double deltaError = error - lastError;
        double deltaTime = time - lastTime;

        integral += error * deltaTime;

        lastError = error;
        lastTime = time;

        return Kp * error + Ki * integral + Kd * (deltaError/deltaTime);
    }
}
