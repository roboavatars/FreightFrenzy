package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@SuppressWarnings("FieldCanBeLocal")
public class Deposit {
    private DcMotorEx depositor;
    private Servo depositServo;
    private Servo teamMarkerServo;

    private static double lastTargetPower = 0;
    private static int lastTargetPos = 0;
    private static double lastServoPos = 0;
    private static int offset = 0;

    public enum DepositHeight {
        HOME, LOW, MID, TOP, CAP, UNDEFINED
    }

    public DepositHeight targetHeight = DepositHeight.HOME;

    public Deposit(LinearOpMode op, boolean isAuto) {
        depositor = op.hardwareMap.get(DcMotorEx.class, "depositor");
        depositServo = op.hardwareMap.get(Servo.class, "depositServo");
        teamMarkerServo = op.hardwareMap.get(Servo.class, "teamMarkerArm");

        if (isAuto) {
            hold();
        } else{
            close();
        }
        teamMarkerServo.setPosition(Constants.TEAM_MARKER_HOME_POS);

        depositor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        depositor.setTargetPosition(0);
        depositor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        depositor.setTargetPosition(0);

        op.telemetry.addData("Status", "Deposit Initialized");
    }

    // Slides
    public void moveSlides(double power, DepositHeight depositHeight) {
        depositor.setPower(power);
        if (depositHeight == depositHeight.HOME) {
            depositor.setTargetPosition(Constants.HOME);
            targetHeight = depositHeight.HOME;
        } else if (depositHeight == depositHeight.LOW) {
            depositor.setTargetPosition(Constants.LOW_GOAL);
            targetHeight = depositHeight.LOW;
        } else if (depositHeight == depositHeight.MID) {
            depositor.setTargetPosition(Constants.MID_GOAL);
            targetHeight = depositHeight.MID;
        } else if (depositHeight == depositHeight.TOP) {
            depositor.setTargetPosition(Constants.TOP_GOAL);
            targetHeight = depositHeight.TOP;
        } else if (depositHeight == depositHeight.CAP) {
            depositor.setTargetPosition(Constants.CAP);
            targetHeight = depositHeight.CAP;
        } else {
            depositor.setTargetPosition(0);
        }
    }

    public void moveSlides(float power) {
        depositor.setPower(power);
    }

    public double getSlidesHeight() {
        return depositor.getCurrentPosition() * 0.043;
    }

    // Deposit
    private void depositSetPosition(double pos) {
        if (pos != lastServoPos) {
            depositServo.setPosition(pos);
            lastServoPos = pos;
        }
    }

    public void autoOpen() {
        depositSetPosition(Constants.DEPOSIT_AUTO_OPEN_POS);
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

    // Team marker
    public void markerSetPosition(double pos) {
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
