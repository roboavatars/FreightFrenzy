package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@SuppressWarnings("FieldCanBeLocal")
public class Intake {
    private DcMotorEx intakeMotor;
    private Servo blockerServo;
    private DistanceSensor intakeSensor;

    private double lastIntakePow = 0;
    private double lastBlockerPos = 0;

    public Intake(LinearOpMode op) {
        intakeMotor = op.hardwareMap.get(DcMotorEx.class, "intake");
        // blockerServo = op.hardwareMap.get(Servo.class, "blocker");
        // intakeSensor = op.hardwareMap.get(DistanceSensor.class, "intakeSensor");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        op.telemetry.addData("Status", "Intake Initialized");
    }

    // Intake Motor
    public void on() {
        setPower(1);
    }

    public void reverse() {
        setPower(-1);
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

    // Blocker

    private void setBlockerPos (double pos) {
        if (pos != lastBlockerPos) {
            blockerServo.setPosition(pos);
            lastBlockerPos = pos;
        }
    }

    //get power
    public double getLastIntakePow(){
        return lastIntakePow;
    }

    // Distance Sensor
    private double getDistance() {
        return intakeSensor.getDistance(DistanceUnit.MM);
    }

    public boolean intakeFull() {
        return getDistance() < Constants.INTAKE_DISTANCE_THRESHOLD;
    }
}
