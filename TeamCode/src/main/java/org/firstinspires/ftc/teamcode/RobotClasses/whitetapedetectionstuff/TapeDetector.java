package org.firstinspires.ftc.teamcode.RobotClasses.whitetapedetectionstuff;

import static java.lang.Math.PI;
import static java.lang.Math.asin;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.opencv.core.Mat;

public class TapeDetector {
    private ColorSensorForTapeDetection left;
    private ColorSensorForTapeDetection right;
    private boolean entering;
    public double yOffset;
    public double thetaOffset;
    public double THETA_THRESHOLD = 0.1;
    public double SENSORS_DIST = 10;

    public TapeDetector(LinearOpMode op) {
        left = new ColorSensorForTapeDetection(op, "left");
        right = new ColorSensorForTapeDetection(op, "right");

        yOffset = 0;
        thetaOffset = 0;
    }

    public void entering() {
        entering = true;
        right.isNull = true;
        left.isNull = true;
    }

    public void exiting() {
        entering = false;
        right.isNull = true;
        left.isNull = true;
    }

    public void update(double x, double y, double theta) {
        left.update(x, y, theta);
        right.update(x, y, theta);

        if (!right.isNull && !left.isNull) {
            if (Math.abs(left.theta - right.theta) < THETA_THRESHOLD) {
                double correctTheta = PI/2 + Math.asin((left.y - right.y)/SENSORS_DIST);
                while (Math.abs(correctTheta - theta) > 2*PI) {
                    if ((correctTheta - theta) > 2 * PI) {
                        theta += 2 * PI;
                    } else if ((correctTheta - theta) < -2 * PI) {
                        correctTheta += 2 * PI;
                    }
                }
                thetaOffset += correctTheta - theta;

                if (entering) {
                    yOffset += 96 - (left.y + right.y) / 2;
                } else {
                    yOffset += 98 - (left.y + right.y) / 2;
                }
            }
        }
    }
}
