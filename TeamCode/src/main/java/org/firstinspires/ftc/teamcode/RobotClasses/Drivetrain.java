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
    public DcMotorEx motorFrontRight;
    public DcMotorEx motorFrontLeft;
    public DcMotorEx motorBackRight;
    public DcMotorEx motorBackLeft;

    // OpMode
    private LinearOpMode op;

    // Tracking X/Y/Theta
    public double x, y, theta;
    private double deltaHeading = 0;

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
    public static double ticksToInchL = 0.0378367670;
    public static double ticksToInchR = 0.0375245580;
    public static double ODOMETRY_TRACK_WIDTH = 10.420;
    private final double ODOMETRY_HEADING_THRESHOLD = PI/8;

    // PD Controller Constants
    public static double leftFF = 0;
    public static double leftKp = 0.004;
    public static double rightFF = 0;
    public static double rightKp = 0.0049;
    public static double b = 0.035;
    public static double zeta = 0.6;

    // Odometry delta 0 counters
    public int zeroR, zeroL;

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

        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        x = initialX;
        y = initialY;
        theta = initialTheta;


    }

    // reset odometry
    public void resetOdo(double newX, double newY, double newTheta) {
        x = newX;
        y = newY;
        theta = newTheta;
    }

    // robot centric movement
    public void setControls(double xdot, double w) {
        setControls(xdot, 0, w);
    }

    public void setControls(double xdot, double ydot, double w) {
        double FRpower, FLpower, BRpower, BLpower;

        FRpower = ydot + xdot + w;
        FLpower = -ydot + xdot - w;
        BRpower = -ydot + xdot + w;
        BLpower = ydot + xdot - w;

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

    public void tankControls(double leftVelocity, double rightVelocity, double theta, double vx, double vy, double w) {
        double angle = Math.atan2(vy, vx);
        if (angle < 0) {
            angle += 2*PI;
        }

        double v = -Math.hypot(vx, vy);
        if (Math.hypot(vx, vy) < 0.01 || Math.abs(angle - theta) < 0.01) {
            v *= -1;
        }

        double vRight = v + w * ODOMETRY_TRACK_WIDTH / 2;
        double vLeft = v - w * ODOMETRY_TRACK_WIDTH / 2;
        setTankControls(leftFF * leftVelocity + leftKp * (leftVelocity - vLeft), rightFF * rightVelocity + rightKp * (rightVelocity - vRight));
    }

    public void setTankControls(double leftPower, double rightPower) {
        setControls((rightPower + leftPower) / 2, (rightPower - leftPower) / 2);
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
        setControls(0, 0);
    }

    // update position from odometry
    public void updatePose() {
        try {
            podR = ticksToInchR * (motorFrontRight.getCurrentPosition() + motorBackRight.getCurrentPosition()) / 2.0;
            podL = ticksToInchL * (motorFrontLeft.getCurrentPosition() + motorBackLeft.getCurrentPosition()) / 2.0;

            deltaPodR = podR - lastPodR;
            deltaPodL = podL - lastPodL;

            if (deltaPodR == 0 || deltaPodL == 0) {
                if (deltaPodR == 0) {
                    Log.w("pod-delta-log", "podR delta 0");
                    zeroR++;
                }
                if (deltaPodL == 0) {
                    Log.w("pod-delta-log", "podL delta 0");
                    zeroL++;
                }
            }

            deltaHeading = (deltaPodR - deltaPodL) / ODOMETRY_TRACK_WIDTH;

            double localX = (deltaPodR + deltaPodL) / 2;
            double localY = 0;

//            Robot.log(deltaPodL + " " + deltaPodR + " " + deltaHeading);

            if (deltaHeading < ODOMETRY_HEADING_THRESHOLD) {
                x += localX * Math.cos(theta) - localY * Math.sin(theta);
                y += localY * Math.cos(theta) + localX * Math.sin(theta);

            } else {
                x += (localX * Math.sin(theta + deltaHeading) + localY * Math.cos(theta + deltaHeading)
                        - localX * Math.sin(theta) - localY * Math.cos(theta)) / deltaHeading;
                y += (localY * Math.sin(theta + deltaHeading) - localX * Math.cos(theta + deltaHeading)
                        - localY * Math.sin(theta) + localX * Math.cos(theta)) / deltaHeading;
            }

            theta += deltaHeading;
            theta = theta % (2*PI);
            if (theta < 0) theta += 2*PI;

            lastPodR = podR;
            lastPodL = podL;
        } catch (Exception e) {
            e.printStackTrace();
        }
    }


}