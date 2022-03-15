package org.firstinspires.ftc.teamcode.Localization.TapeDetector;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static java.lang.Math.PI;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class TapeDetector {
    public TapeColorSensor left;
    private boolean entering;
    private boolean resetOdo = false;
    public static double TAPE_THETA_THRESHOLD = PI / 20;
    public static double TAPE_SENSORS_DIST = 10;
    public static double TAPE_SENSOR_CENTER_TO_ROBOT_CENTER = -4.5;

    public TapeDetector(LinearOpMode op) {
        left = new TapeColorSensor(op, "lineSensor");
    }

    public void entering() {
        entering = true;
        left.isNull = true;
        resetOdo = false;
    }

    public void exiting() {
        entering = false;
        left.isNull = true;
        resetOdo = false;
    }

    public double[] update(double x, double y, double theta) {
        double[] updatedCoords = new double[2]; //[0] = y; [1] = theta

        left.update(x, y, theta);

        if (!resetOdo && !left.isNull) {
            //Calculate robot theta
//            updatedCoords[1] = PI / 2 + Math.asin(Math.min(Math.max(((left.y - right.y) / TAPE_SENSORS_DIST), -1), 1)); //asin(x), where x is restricted to [-1,1]
//            updatedCoords[1] %= 2 * PI;
//            if (updatedCoords[1] < 0) updatedCoords[1] += 2 * PI;
            theta = 0;

            //Calculate robot Y position
            if (entering) {
//                updatedCoords[0] = 100 - TAPE_SENSOR_CENTER_TO_ROBOT_CENTER * sin(updatedCoords[1]);
                updatedCoords[0] = 100 - TAPE_SENSOR_CENTER_TO_ROBOT_CENTER;
            } else {
//                updatedCoords[0] = 101 - TAPE_SENSOR_CENTER_TO_ROBOT_CENTER * sin(updatedCoords[1]);
                updatedCoords[0] = 101 - TAPE_SENSOR_CENTER_TO_ROBOT_CENTER;
            }
            resetOdo = true;

            addPacket("white hit", 0);
        } else {
            updatedCoords[0] = y;
            updatedCoords[1] = theta;
        }
        addPacket("1 left null", left.isNull);

        if (!left.isNull) {
            addPacket("3 left x", left.x);
            addPacket("3 left y", left.y);
            addPacket("3 left theta", left.theta);
        }

        return updatedCoords;
    }
}
