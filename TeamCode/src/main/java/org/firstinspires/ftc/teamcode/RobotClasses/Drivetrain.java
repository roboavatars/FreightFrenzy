package org.firstinspires.ftc.teamcode.RobotClasses;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static java.lang.Math.PI;

@SuppressWarnings("FieldCanBeLocal") @Config
public class Drivetrain {

    // Electronics
    private DcMotorEx motorFrontRight;
    private DcMotorEx motorFrontLeft;
    private DcMotorEx motorBackRight;
    private DcMotorEx motorBackLeft;

    // OpMode
    private LinearOpMode op;

    // Tracking X/Y/Theta
    public double x, y, theta, startTheta;
    private double lastRawHeading = 0;
    private double deltaHeading = 0;
    public double commandedW;

    // Odometry
    public double podR = 0;
    public double podL = 0;
    private double lastPodR = 0;
    private double lastPodL = 0;
    private double deltaPodR;
    private double deltaPodL;

    // Motor Caching
    private double lastFRPower = 0;
    private double lastBRPower = 0;
    private double lastFLPower = 0;
    private double lastBLPower = 0;
    private final double motorUpdateTolerance = 0.05;

    // Odometry Constants
    public static double ticksToInchR = 0.005;
    public static double ticksToInchL = 0.005;
    public static double ODOMETRY_HORIZONTAL_OFFSET = 0.15;

    private final double ODOMETRY_HEADING_THRESHOLD = PI/8;

    // PD Controller Constants
    public final static double xKp = 0.6;
    public final static double yKp = 0.55;
    public final static double thetaKp = 3.0;
    public final static double xKd = 0.05;
    public final static double yKd = 0.05;
    public final static double thetaKd = 0.07;

    // Odometry delta 0 counters
    public int zero1, zero2;

    public boolean zeroStrafeCorrection = false;

    public double lastHeading;

    // Constructor
    public Drivetrain(LinearOpMode op, double initialX, double initialY, double initialTheta) {
        this.op = op;
        HardwareMap hardwareMap = op.hardwareMap;

        motorFrontRight = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        motorFrontLeft = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        motorBackRight = hardwareMap.get(DcMotorEx.class, "motorBackRight");
        motorBackLeft = hardwareMap.get(DcMotorEx.class, "motorBackLeft");

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        x = initialX;
        y = initialY;
        startTheta = initialTheta;
        theta = initialTheta;
        lastHeading = theta;
    }

    // reset odometry
    public void resetOdo(double newX, double newY, double newTheta) {
        x = newX;
        y = newY;
        theta = newTheta;
//        imu.resetHeading(newTheta);
    }

    // robot centric movement
    public void setControls(double xdot, double ydot, double w) {
        commandedW = w;

        double FRpower, FLpower, BRpower, BLpower;

        if (!zeroStrafeCorrection) {
            FRpower = ydot + xdot + w;
            FLpower = -ydot + xdot - w;
            BRpower = -ydot + xdot + w;
            BLpower = ydot + xdot - w;
        } else {
            FRpower = xdot + w;
            FLpower = xdot - w;
            BRpower = xdot + w;
            BLpower = xdot - w;
        }

        double maxpower = Math.max(Math.abs(FRpower), Math.max(Math.abs(FLpower), Math.max(Math.abs(BRpower), Math.abs(BLpower))));

        if (maxpower > 1) {
            FRpower /= maxpower;
            FLpower /= maxpower;
            BRpower /= maxpower;
            BLpower /= maxpower;
        }

        if (xdot == 0 && ydot == 0 && w == 0) {
            // Set Motor Powers
            motorFrontRight.setPower(FRpower);
            motorFrontLeft.setPower(FLpower);
            motorBackRight.setPower(BRpower);
            motorBackLeft.setPower(BLpower);

            // Cache New Motor Powers
            lastFRPower = FRpower;
            lastFLPower = FLpower;
            lastBRPower = BRpower;
            lastBLPower = BLpower;

        } else if (Math.abs(FRpower - lastFRPower) > motorUpdateTolerance || Math.abs(FLpower - lastFLPower) > motorUpdateTolerance
                || Math.abs(BRpower - lastBRPower) > motorUpdateTolerance || Math.abs(BLpower - lastBLPower) > motorUpdateTolerance) {

            // Set Motor Powers
            motorFrontRight.setPower(FRpower);
            motorFrontLeft.setPower(FLpower);
            motorBackRight.setPower(BRpower);
            motorBackLeft.setPower(BLpower);

            // Cache New Motor Powers
            lastFRPower = FRpower;
            lastFLPower = FLpower;
            lastBRPower = BRpower;
            lastBLPower = BLpower;
        }
    }

    public void setRawPower(double frontRight, double frontLeft, double backRight, double backLeft) {
        // Set Motor Powers
        motorFrontRight.setPower(frontRight);
        motorFrontLeft.setPower(frontLeft);
        motorBackRight.setPower(backRight);
        motorBackLeft.setPower(backLeft);

        // Cache New Motor Powers
        lastFRPower = frontRight;
        lastFLPower = frontLeft;
        lastBRPower = backRight;
        lastBLPower = backLeft;
    }

    // field centric movement
    public void setGlobalControls(double xvelocity, double yvelocity, double w) {
        double xdot = xvelocity * Math.cos(-theta) - yvelocity * Math.sin(-theta);
        double ydot = yvelocity * Math.cos(-theta) + xvelocity * Math.sin(-theta);
        setControls(xdot, ydot, w);
    }

    // stop drivetrain
    public void stop() {
        setGlobalControls(0, 0, 0);
    }

    // update position from odometry
    public void updatePose() {
        try {
            podR = ticksToInchR * (motorFrontRight.getCurrentPosition() + motorBackRight.getCurrentPosition()) / 2.0;
            podL = ticksToInchL * (motorFrontLeft.getCurrentPosition() + motorBackLeft.getCurrentPosition()) / 2.0;

            deltaPodR = podR - lastPodR;
            deltaPodL = podL - lastPodL;

            if (theta < 0) {
                theta += 2*PI;
            }
            deltaHeading = theta - lastHeading;

//            deltaPod1 = deltaPod2 - deltaHeading * ODOMETRY_TRACK_WIDTH;

//            imu.updateHeading();
//            theta = imu.getTheta() % (2*PI);
//            deltaHeading = imu.getDeltaHeading();
//            deltaPod1 = deltaPod2 - deltaHeading * ODOMETRY_TRACK_WIDTH;

            if (deltaPodR == 0 || deltaPodL == 0) {
                if (deltaPodR == 0) {
                    Log.w("pod-delta-log", "podR delta 0");
                    zero1++;
                }
                if (deltaPodL == 0) {
                    Log.w("pod-delta-log", "podL delta 0");
                    zero2++;
                }
            }

//            deltaHeading = (deltaPod2 - deltaPod1) / ODOMETRY_TRACK_WIDTH;

            double localX = (deltaPodR + deltaPodL) / 2;
            double localY = 0;

//            Robot.log(deltaPod1 + " " + deltaPod2 + " " + deltaPod3 + " " + deltaHeading);

            if (deltaHeading < ODOMETRY_HEADING_THRESHOLD) {
                x += localX * Math.cos(theta) - localY * Math.sin(theta);
                y += localY * Math.cos(theta) + localX * Math.sin(theta);

            } else {
                x += (localX * Math.sin(theta + deltaHeading) + localY * Math.cos(theta + deltaHeading)
                        - localX * Math.sin(theta) - localY * Math.cos(theta)) / deltaHeading;
                y += (localY * Math.sin(theta + deltaHeading) - localX * Math.cos(theta + deltaHeading)
                        - localY * Math.sin(theta) + localX * Math.cos(theta)) / deltaHeading;
            }

//            theta = startTheta + (pod2 - pod1) / ODOMETRY_TRACK_WIDTH;
//            theta = theta % (2*PI);
//            if (theta < 0) theta += 2*PI;

            lastPodR = podR;
            lastPodL = podL;
            lastHeading = theta;
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}