package org.firstinspires.ftc.teamcode.Localization;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Config
public class IMU {
    private BNO055IMU imu;
    private double theta;
    private double lastHeading;
    private double deltaHeading = 0;

    public static double IMU_DRIFT_COMPENSATION = 1;

    public IMU(double startTheta, LinearOpMode op) {
        this(startTheta, op, BNO055IMU.SensorMode.IMU);
    }
    public IMU(double startTheta, LinearOpMode op, BNO055IMU.SensorMode mode) {
        imu = op.hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = mode;
        imu.initialize(parameters);

        if (mode == BNO055IMU.SensorMode.IMU) {
            while (!op.isStopRequested() && !imu.isGyroCalibrated()) {
                op.idle();
            }
        }

        theta = startTheta;
        lastHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;

        op.telemetry.addData("Status", "IMU Initialized");
    }

    public void updateHeading() {
        double angle = IMU_DRIFT_COMPENSATION * imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        deltaHeading = angle - lastHeading;

        if (deltaHeading < -PI) {
            deltaHeading += 2 * PI;
        } else if (deltaHeading >= PI) {
            deltaHeading -= 2 * PI;
        }

        theta = (theta + deltaHeading) % (2 * PI);
        if (theta < 0) {
            theta += 2 * PI;
        }
        lastHeading = angle;
    }

    public void updateHeadingUncapped() {
        double angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        deltaHeading = angle - lastHeading;

        if (deltaHeading < -PI) {
            deltaHeading += 2 * PI;
        } else if (deltaHeading >= PI) {
            deltaHeading -= 2 * PI;
        }

        theta += deltaHeading;
        lastHeading = angle;
    }

    public void resetHeading(double newTheta) {
        lastHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        theta = newTheta;
    }

    public double getTheta() {
        return theta;
    }

    public double getDeltaHeading() {
        return deltaHeading;
    }

    public Acceleration getAccel () {return imu.getLinearAcceleration();}
}
