package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@SuppressWarnings("FieldCanBeLocal")
public class Deposit {
    private DcMotorEx depositMotor;
    private Servo dumperServo;

    private double lastTargetPower = 0;
    private int lastTargetPos = 0;
    private double lastServoPos = 0;
    private int offset = 0;

    public Deposit(LinearOpMode op) {
        depositMotor = op.hardwareMap.get(DcMotorEx.class, "depositMotor");
        dumperServo = op.hardwareMap.get(Servo.class, "deposit");

        depositMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        op.telemetry.addData("Status", "Deposit Initialized");
    }

    private void moveSlides(double power, int ticks) {
        if (power != lastTargetPower) {
            depositMotor.setPower(power);
            lastTargetPower = power;
        }

        if (ticks != lastTargetPos) {
            depositMotor.setTargetPosition(ticks);
            lastTargetPos = ticks;
        }
    }


    public void resetAtHomeHeight() {
        offset = depositMotor.getCurrentPosition() - Constants.HOME_TICKS;
    }

    public void resetAtDepositHeight() {
        offset = depositMotor.getCurrentPosition() - Constants.DEPOSIT_TICKS;
    }

    public void resetAtCapHeight() {
        offset = depositMotor.getCurrentPosition() - Constants.CAP_TICKS;
    }

    public void home () {
        if (depositMotor.getCurrentPosition() > Constants.HOME_TICKS) {
            moveSlides(-Constants.DEPOSIT_POWER,Constants.HOME_TICKS + offset);
        } else {
            moveSlides(Constants.DEPOSIT_POWER,Constants.HOME_TICKS + offset);
        }
    }

    public void deposit() {
        if (depositMotor.getCurrentPosition() > Constants.DEPOSIT_TICKS) {
            moveSlides(-Constants.DEPOSIT_POWER,Constants.DEPOSIT_TICKS + offset);
        } else {
            moveSlides(Constants.DEPOSIT_POWER,Constants.DEPOSIT_TICKS + offset);
        }
    }

    public void cap() {
        if (depositMotor.getCurrentPosition() > Constants.CAP_TICKS){
            moveSlides(-Constants.DEPOSIT_POWER,Constants.CAP_TICKS + offset);
        } else {
            moveSlides(Constants.DEPOSIT_POWER,Constants.CAP_TICKS + offset);
        }
    }

    private void setPosition(double pos) {
        if (pos != lastServoPos) {
            dumperServo.setPosition(pos);
            lastServoPos = pos;
        }
    }

    public void open() {
        setPosition(Constants.DEPOSIT_OPEN_POS);
    }

    public void close() {
        setPosition(Constants.DEPOSIT_CLOSE_POS);
    }
}
