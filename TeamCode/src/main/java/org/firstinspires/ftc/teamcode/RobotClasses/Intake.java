package org.firstinspires.ftc.teamcode.RobotClasses;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@SuppressWarnings("FieldCanBeLocal")
public class Intake {
    private DcMotorEx intakeMotor;
    private DcMotorEx slidesMotor;
    private Servo flipServo;
    private DistanceSensor intakeSensor;

    private double lastIntakePow = 0;

    private boolean slidesHome = true;
    public double intakeOffset = 0;
    private double lastSlidesPos = 0;
    private double slidesTargetPos = 0;

    public double INTAKE_SLIDES_SERVO_SPEED = 0.1;
    public double HOME_THRESHOLD = 20;

    private LinearOpMode op;

    public Intake(LinearOpMode op, boolean isAuto) {
        intakeMotor = op.hardwareMap.get(DcMotorEx.class, "intake");

        flipServo = op.hardwareMap.get(Servo.class, "intakeServo");

        slidesMotor = op.hardwareMap.get(DcMotorEx.class, "intakeSlides");
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setTargetPosition(0);
        slidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // Intake Motor
    public void on() {
        setPower(-1);
    }

    public void reverse() {
        setPower(1);
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

    public boolean checkIfStalling() {
        return intakeMotor.getCurrent(CurrentUnit.AMPS) > Constants.STALL_THRESHOLD;
    }

    public double getCurrent() {
        return intakeMotor.getCurrent(CurrentUnit.AMPS);
    }

    // Intake Slides
    public void extend() {
        setSlidesPosition(Constants.INTAKE_SLIDES_EXTEND_TICKS);
    }

    public void home() {
        setSlidesPosition(Constants.INTAKE_SLIDES_HOME_TICKS);
    }

    public void setSlidesPosition(int position) {
        slidesMotor.setTargetPosition(position);
        slidesMotor.setPower(Constants.INTAKE_SLIDES_POWER);
    }

    public double getSlidesPos() {
        return slidesMotor.getCurrentPosition();
    }

    public boolean slidesIsHome() {
        return getSlidesPos() - Constants.INTAKE_SLIDES_HOME_TICKS < HOME_THRESHOLD;
    }

    // Intake Servo
    public void flipUp() {
        flipServo.setPosition(Constants.INTAKE_UP_POS);
    }

    public void flipDown() {
        flipServo.setPosition(Constants.INTAKE_DOWN_POS);
    }

    // Distance Sensor
    public double getDistance() {
        addPacket("0 DISTANCE SENSOR", "> 1000!!!");
        op.telemetry.addData("0 DISTANCE SENSOR", "> 1000!!!");
        return intakeSensor.getDistance(DistanceUnit.MM);
    }

    public boolean intakeFull() {
        return getDistance() < Constants.INTAKE_DISTANCE_THRESHOLD;
    }
}
