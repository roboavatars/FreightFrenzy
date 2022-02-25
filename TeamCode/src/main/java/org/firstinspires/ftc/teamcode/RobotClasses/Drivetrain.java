package org.firstinspires.ftc.teamcode.RobotClasses;

import static java.lang.Math.PI;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@SuppressWarnings("FieldCanBeLocal") @Config
public class Drivetrain {

    // Electronics
    private DcMotorEx motorFrontRight;
    private DcMotorEx motorFrontLeft;
    private DcMotorEx motorBackRight;
    private DcMotorEx motorBackLeft;
    private Servo odoRetractServo;

    // OpMode
    private LinearOpMode op;

    // Tracking X/Y/Theta
    public double x, y, theta, startTheta;
    private double lastRawHeading = 0;
    private double deltaHeading = 0;
    public double commandedW;

    // Odometry
    public double pod1 = 0;
    public double pod2 = 0;
    public double pod3 = 0;
    private double lastPod1 = 0;
    private double lastPod2 = 0;
    private double lastPod3 = 0;
    private double deltaPod1;
    private double deltaPod2;
    private double deltaPod3;

    // Motor Caching
    private double lastFRPower = 0;
    private double lastBRPower = 0;
    private double lastFLPower = 0;
    private double lastBLPower = 0;
    private final double motorUpdateTolerance = 0.05;

    // Odometry constants
    public static double ticksToInch1 = 0.00052639659596868;
    public static double ticksToInch2 = 0.00051562172701052;
    public static double ticksToInch3 = 0.00053046000828844;
    public static double ODOMETRY_TRACK_WIDTH = 9.87211596315280939430;
    public static double ODOMETRY_HORIZONTAL_OFFSET = 0.15;

    private final double ODOMETRY_HEADING_THRESHOLD = PI/8;

    // PD controller constants
    public final static double xKp = 0.5;
    public final static double yKp = 0.4;
    public final static double thetaKp = 2.5;
    public final static double xKd = 0.05;
    public final static double yKd = 0.04;
    public final static double thetaKd = 0.12;

    // Odometry delta 0 counters
    public int zero1, zero2, zero3;

    public boolean zeroStrafeCorrection = false;

    public double lastHeading;

    public boolean constantStrafe = false;
    public double constantStrafeConstant = 0;

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

        odoRetractServo = hardwareMap.get(Servo.class, "odoRetractServo");
        odoRetractServo.setPosition(Constants.ODO_NORMAL_POS);

        x = initialX;
        y = initialY;
        startTheta = initialTheta;
        theta = initialTheta;
        lastHeading = theta;
    }

    // retract horizontal odometry pod
    public void retractOdo() {
        odoRetractServo.setPosition(Constants.ODO_RETRACT_POS);
    }

    // reset odometry
    public void resetOdo(double newX, double newY, double newTheta) {
        x = newX;
        y = newY;
        theta = newTheta;
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
    public void setGlobalControls(double xvelocity, double yvelocity, double w, boolean useStrafeConstant) {
        double xdot = xvelocity * Math.cos(-theta) - yvelocity * Math.sin(-theta);
        double ydot = yvelocity * Math.cos(-theta) + xvelocity * Math.sin(-theta);
        if (useStrafeConstant) ydot += constantStrafeConstant;
        setControls(xdot, ydot, w);
    }

    public void setGlobalControls(double xvelocity, double yvelocity, double w) {
        setGlobalControls(xvelocity, yvelocity, w, true);
    }

    // stop drivetrain
    public void stop() {
        setGlobalControls(0, 0, 0);
    }

    // update position from odometry
    public void updatePose() {
        try {
            pod1 = motorBackRight.getCurrentPosition() * ticksToInch1;
            pod2 = motorFrontLeft.getCurrentPosition() * -ticksToInch2;
            pod3 = motorFrontRight.getCurrentPosition() * -ticksToInch3;

            deltaPod1 = pod1 - lastPod1;
            deltaPod2 = pod2 - lastPod2;
            deltaPod3 = pod3 - lastPod3;

            Log.w("auto", deltaPod1 + " " + deltaPod2 + " " + deltaPod3);
            if (!(deltaPod1 == 0 && deltaPod2 == 0 && deltaPod3 == 0)) {
                if (deltaPod1 == 0) {
                    Log.w("pod-delta-log", "pod1 delta 0");
                    zero1++;
                }
                if (deltaPod2 == 0) {
                    Log.w("pod-delta-log", "pod2 delta 0");
                    zero2++;
                }
                if (deltaPod3 == 0) {
                    Log.w("pod-delta-log", "pod3 delta 0");
                    zero3++;
                }
            }

            deltaHeading = (deltaPod1 - deltaPod2) / ODOMETRY_TRACK_WIDTH;

            double localX = (deltaPod1 + deltaPod2) / 2;
            double localY = deltaPod3 - deltaHeading * ODOMETRY_HORIZONTAL_OFFSET;

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
            theta = theta % (Math.PI * 2);
            if (theta < 0) theta += Math.PI * 2;

            lastPod1 = pod1;
            lastPod2 = pod2;
            lastPod3 = pod3;
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}