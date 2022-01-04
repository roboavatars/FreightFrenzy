package org.firstinspires.ftc.teamcode.RobotClasses.whitetapedetectionstuff;

import static java.lang.Math.PI;
import static java.lang.Math.sin;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
public class TapeDetector {
    public ColorSensorForTapeDetection left;
    public ColorSensorForTapeDetection right;
    private boolean entering;
    public final double THETA_THRESHOLD = 0.2;
    public final double SENSORS_DIST = 10;
    public final double SENSOR_CENTER_TO_ROBOT_CENTER = 8;
    private boolean resetOdo = false;

    public TapeDetector(LinearOpMode op) {
        left = new ColorSensorForTapeDetection(op, "left");
        right = new ColorSensorForTapeDetection(op, "right");
    }

    public void entering() {
        entering = true;
        right.isNull = true;
        left.isNull = true;
        resetOdo = false;
    }

    public void exiting() {
        entering = false;
        right.isNull = true;
        left.isNull = true;
        resetOdo = false;
    }

    public double[] update(double x, double y, double theta) {
        double[] updatedCoords = new double[2];

        left.update(x, y, theta);
        right.update(x, y, theta);

        if (!resetOdo && !right.isNull && !left.isNull && Math.abs(left.theta - right.theta) < THETA_THRESHOLD) {
            //Calculate robot theta
            updatedCoords[1] = PI/2 + Math.asin((left.y - right.y)/SENSORS_DIST);
            updatedCoords[1] %= 2*PI;
            if (updatedCoords[1]<0) updatedCoords[1] += 2*PI;

            //Calculate robot Y position
            if (entering) {
                updatedCoords[0] = 96 - SENSOR_CENTER_TO_ROBOT_CENTER * sin(updatedCoords[1]);
            } else {
                updatedCoords[0] = 98 - SENSOR_CENTER_TO_ROBOT_CENTER * sin(updatedCoords[1]);
            }
            resetOdo = true;
        } else {
            updatedCoords[0] = y;
            updatedCoords[1] = theta;
        }
        return updatedCoords;
    }
}
