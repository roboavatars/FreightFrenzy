package org.firstinspires.ftc.teamcode.Pathing.Ramsete;

public class PDController {
    boolean firstLoop = true;
    double lastError;
    double lastTime;

    public double output(double currentVelo, double targetVelo, double Kp, double Kd, double time) {
        double error = targetVelo - currentVelo;
        if (firstLoop){
            firstLoop = false;
            lastError = 0;
            lastTime = time;
        }

        double deltaError = error - lastError;
        double deltaTime = time - lastTime;

        lastError = error;
        lastTime = time;
        return Kp * error + Kd * deltaError/deltaTime;
    }

    public double output(double position, double targetPosition, double velocity, double targetVelo, double Kp, double Kd) {
        return Kp * (targetPosition - position) + Kd * (targetVelo - velocity);
    }
}
