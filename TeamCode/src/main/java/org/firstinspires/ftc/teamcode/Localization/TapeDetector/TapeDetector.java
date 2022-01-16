package org.firstinspires.ftc.teamcode.Localization.TapeDetector;

import static java.lang.Math.PI;
import static java.lang.Math.sin;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotClasses.Constants;

@Config
public class TapeDetector {
    public TapeColorSensor left;
    public TapeColorSensor right;
    private boolean entering;
    private boolean resetOdo = false;

    public TapeDetector(LinearOpMode op) {
        left = new TapeColorSensor(op, "left");
        right = new TapeColorSensor(op, "right");
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
        double[] updatedCoords = new double[2]; //[0] = y; [1] = theta

        left.update(x, y, theta);
        right.update(x, y, theta);

        if (!resetOdo && !right.isNull && !left.isNull && Math.abs(left.theta - right.theta) < Constants.TAPE_THETA_THRESHOLD) {
            //Calculate robot theta
            updatedCoords[1] = PI / 2 + Math.asin(Math.min(Math.max(((left.y - right.y) / Constants.TAPE_SENSORS_DIST), -1), 1)); //asin(x), where x is restricted to [-1,1]
            updatedCoords[1] %= 2 * PI;
            if (updatedCoords[1] < 0) updatedCoords[1] += 2 * PI;

            //Calculate robot Y position
            if (entering) {
                updatedCoords[0] = 96 - Constants.TAPE_SENSOR_CENTER_TO_ROBOT_CENTER * sin(updatedCoords[1]);
            } else {
                updatedCoords[0] = 98 - Constants.TAPE_SENSOR_CENTER_TO_ROBOT_CENTER * sin(updatedCoords[1]);
            }
            resetOdo = true;
        } else {
            updatedCoords[0] = y;
            updatedCoords[1] = theta;
        }
        return updatedCoords;
    }
}
