package org.firstinspires.ftc.teamcode.RobotClasses;

import android.net.wifi.hotspot2.pps.HomeSp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.C;

@Config
@SuppressWarnings("FieldCanBeLocal")
public class Deposit {
    private DcMotorEx depositor;
    private Servo depositServo;

    private static double lastTargetPower = 0;
    private static int lastTargetPos = 0;
    private static double lastServoPos = 0;
    private static int offset = 0;

    public enum deposit_height{
        HOME, MID, TOP, CAP
    }

    public Deposit(LinearOpMode op) {
        depositor = op.hardwareMap.get(DcMotorEx.class, "depositor");
        depositServo = op.hardwareMap.get(Servo.class, "depositServo");

        depositServo.setPosition(0);

        depositor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        depositor.setTargetPosition(0);
        depositor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        depositor.setTargetPosition(0);

        op.telemetry.addData("Status", "Deposit Initialized");
    }

    private void moveSlides(double power, int ticks) {
        if (power != lastTargetPower) {
            depositor.setPower(power);
            lastTargetPower = power;
        }

        if (ticks != lastTargetPos) {
            depositor.setTargetPosition(ticks);
            lastTargetPos = ticks;
            depositor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        }
    }

    public void moveSlides(double power, deposit_height deposit_height) {
        depositor.setPower(power);
        if (deposit_height == deposit_height.HOME) {
            depositor.setTargetPosition(Constants.HOME);
        } else if (deposit_height == deposit_height.MID) {
            depositor.setTargetPosition(Constants.MID_GOAL);
        } else if (deposit_height == deposit_height.TOP) {
            depositor.setTargetPosition(Constants.TOP_GOAL);
        } else if (deposit_height == deposit_height.CAP) {
            depositor.setTargetPosition(Constants.CAP);
        } else {
            depositor.setTargetPosition(0);
        }
    }

    private void setPosition(double pos) {
        if (pos != lastServoPos) {
            depositServo.setPosition(pos);
            lastServoPos = pos;
        }
    }

    public double getPosition() {
        return depositor.getCurrentPosition();
    }

    public void open() {
        setPosition(Constants.DEPOSIT_OPEN_POS);
    }

    public void close() {
        setPosition(Constants.DEPOSIT_CLOSE_POS);
    }
}
