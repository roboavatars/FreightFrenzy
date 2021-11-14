package org.firstinspires.ftc.teamcode.RobotClasses;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@SuppressWarnings("FieldCanBeLocal")
public class Deposit {
    private DcMotorEx depositor;
    private Servo depositServo;
    private Servo teamMarkerServo;

    private static double lastTargetPower = 0;
    private static int lastTargetPos = 0;
    private static double lastServoPos = 0;
    private static int offset = 0;

    public enum deposit_height{
        HOME, MID, TOP, CAP, UNDEFINED
    }

    private deposit_height targHeight = deposit_height.HOME;

    public Deposit(LinearOpMode op) {
        depositor = op.hardwareMap.get(DcMotorEx.class, "depositor");
        depositServo = op.hardwareMap.get(Servo.class, "depositServo");
        teamMarkerServo = op.hardwareMap.get(Servo.class, "teamMarkerArm");

//        depositServo.setPosition(Constants.DEPOSIT_HOLD_POS);

        teamMarkerServo.setPosition(Constants.TEAM_MARKER_UP_POS);

        depositor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        depositor.setTargetPosition(0);
        depositor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        depositor.setTargetPosition(0);

        op.telemetry.addData("Status", "Deposit Initialized");
    }

    public void moveSlides(double power, deposit_height deposit_height) {
        depositor.setPower(power);
        if (deposit_height == deposit_height.HOME) {
            depositor.setTargetPosition(Constants.HOME);
            targHeight = deposit_height.HOME;
        } else if (deposit_height == deposit_height.MID) {
            depositor.setTargetPosition(Constants.MID_GOAL);
            targHeight = deposit_height.MID;
        } else if (deposit_height == deposit_height.TOP) {
            depositor.setTargetPosition(Constants.TOP_GOAL);
            targHeight = deposit_height.TOP;
        } else if (deposit_height == deposit_height.CAP) {
            depositor.setTargetPosition(Constants.CAP);
            targHeight = deposit_height.CAP;
        } else {
            depositor.setTargetPosition(0);
        }
    }

    public void moveSlides(float power) {
        depositor.setPower(power);
    }

    public double getSlidesHeight() {
        return depositor.getCurrentPosition();
    }

    public deposit_height getTargHeight() {
        return getTargHeight();
    }

    public boolean slidesMoving() {
        return depositor.isBusy();
    }

    //deposit

    private void depositSetPosition(double pos) {
        if (pos != lastServoPos) {
            depositServo.setPosition(pos);
            lastServoPos = pos;
        }
    }

    public void open() {
        depositSetPosition(Constants.DEPOSIT_OPEN_POS);
    }

    public void hold() {
        depositSetPosition(Constants.DEPOSIT_HOLD_POS);
    }

    public void close() {
        depositSetPosition(Constants.DEPOSIT_CLOSE_POS);
    }

    //team marker

    private void markerSetPosition(double pos) {
        if (pos != lastServoPos) {
            teamMarkerServo.setPosition(pos);
            lastServoPos = pos;
        }
    }

    public void markerArmDown() {
        markerSetPosition(Constants.TEAM_MARKER_DOWN_POS);
    }
    public void markerArmUp() {
        markerSetPosition(Constants.TEAM_MARKER_UP_POS);
    }
}
