package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@SuppressWarnings("FieldCanBeLocal")
public class Intake {
    private DcMotorEx intakeMotor;
    private Servo blockerServo;
    private DistanceSensor intakeDistSensor;

    private double lastIntakePow = 10;
    private double lastBlockerPos = 2;

    public Intake(LinearOpMode op, boolean isAuto) {
        intakeMotor = op.hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        blockerServo = op.hardwareMap.get(Servo.class, "blocker");

        intakeDistSensor = op.hardwareMap.get(DistanceSensor.class, "intake sensor");

        op.telemetry.addData("Status", "Intake initialized");
    }

    // Intake Motors
    public void on() {
        setPower(Constants.INTAKE_POWER);
    }

    public void reverse() {
        setPower(-Constants.INTAKE_POWER);
    }

    public void off() {
        setPower(0);
    }

    public void setPower(double intakePower) {
        if (intakePower != lastIntakePow) {
            intakeMotor.setPower(intakePower);
            lastIntakePow = intakePower;
        }
    }

    // intake blocker

    private void setBlockerPos (double pos){
        if (pos != lastBlockerPos) {
            blockerServo.setPosition(pos);
            lastBlockerPos = pos;
        }
    }

    public void open() {
        setBlockerPos(Constants.INTAKE_BLOCKER_OPEN_POS);
    }

    public void close() {
        setBlockerPos(Constants.INTAKE_BLOCKER_CLOSE_POS);
    }

    //check if freight intaked
    private double getDistance(){
        return intakeDistSensor.getDistance(DistanceUnit.MM);
    }
    public boolean freightIntaked() {
        if (getDistance()<Constants.INTAKE_DISTANCE_THRESHOLD) {
            return true;
        } else {
            return false;
        }
    }

}
