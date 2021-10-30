package org.firstinspires.ftc.teamcode.Pathing.Ramsete;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotClasses.Drivetrain;

public class RamseteController {
    PDController leftPD;
    PDController rightPD;
    Drivetrain drivetrain;
    ElapsedTime time = new ElapsedTime();

    public RamseteController(Drivetrain drivetrain){
        time.reset();
        this.drivetrain = drivetrain;
    }


    public void calculate(double x, double y, double theta, double xTarget, double yTarget, double thetaTarget, double vx, double vy, double w, double vxTarget, double vyTarget, double wTarget, double Kp, double Kd, double b, double zeta) {
        double eX = xTarget - x;
        double eY = yTarget - y;
        double eTheta = thetaTarget - theta;
        double linearVelo = Math.sqrt(Math.pow(vx,2) + Math.pow(vy,2));
        double linearVeloTarget = Math.sqrt(Math.pow(vxTarget,2) + Math.pow(vyTarget,2));

        //constant
        double k = 2.0 * zeta * Math.sqrt(Math.pow(wTarget, 2) + b * Math.pow(linearVeloTarget, 2));

        // set local errors
        double eXLocal = eX * Math.cos(theta) + eY * Math.sin(theta);
        double eYLocal = eY * -Math.sin(theta) + eY * Math.cos(theta);

        double angularVelocityCommand = wTarget + k * eTheta + b * linearVeloTarget * sinc(eTheta) * eYLocal;
        double velocityCommand = linearVeloTarget * Math.cos(eTheta) + k * eXLocal;


        double rightVelocity = velocityCommand + Drivetrain.ODOMETRY_TRACK_WIDTH / 2 * angularVelocityCommand;
        double leftVelocity = velocityCommand - Drivetrain.ODOMETRY_TRACK_WIDTH / 2 * angularVelocityCommand;

        double currRightVelo = linearVelo + Drivetrain.ODOMETRY_TRACK_WIDTH / 2 * w;
        double currLeftVelo = linearVelo - Drivetrain.ODOMETRY_TRACK_WIDTH / 2 * w;

        double rightPower = rightPD.output(currRightVelo, rightVelocity, Kp, Kd, time.seconds());
        double leftPower = leftPD.output(currLeftVelo, leftVelocity, Kp, Kd, time.seconds());
        drivetrain.setRawPower(rightPower, leftPower, rightPower, leftPower);

    }

    private static double sinc(double x) {
        if (Math.abs(x) < 1e-9) {
            return 1.0 - 1.0 / 6.0 * x * x;
        } else {
            return Math.sin(x) / x;
        }
    }
}
