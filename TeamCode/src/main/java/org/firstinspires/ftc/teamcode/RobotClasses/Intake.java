package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@SuppressWarnings("FieldCanBeLocal")
public class Intake {
    private DcMotorEx intakeMotor;
    private Servo slidesServo;
    private Servo intakeServo;
    private DistanceSensor intakeSensor;

    private double lastIntakePow = 0;

    private boolean slidesHome = true;
    public double intakeOffset = 0;
    private double lastSlidesPos = 0;
    private double slidesTargetPos = 0;

    public Intake(LinearOpMode op, boolean isAuto) {
        // Intake Motor
        intakeMotor = op.hardwareMap.get(DcMotorEx.class, "intake");

        // Slides Motor
        slidesServo = op.hardwareMap.get(Servo.class, "intakeSlides");
        home();

        // Intake Servo
        intakeServo = op.hardwareMap.get(Servo.class, "intakeServo");
        if (isAuto) {
            flipUp();
        } else {
            flipDown();
        }

        // Intake Distance Sensor
        intakeSensor = op.hardwareMap.get(DistanceSensor.class, "intakeSensor");

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

    public boolean checkIfStalling() {
        return intakeMotor.getCurrent(CurrentUnit.AMPS) > Constants.STALL_THRESHOLD;
    }

    public double getCurrent() {
        return intakeMotor.getCurrent(CurrentUnit.AMPS);
    }

    // Intake Slides
    public void extend() {
        slidesHome = false;
        slidesTargetPos = Constants.INTAKE_EXTEND_POS;
    }

    public void extend(double extendPos) {
        slidesHome = false;
        slidesTargetPos = extendPos;
    }

    public void home() {
        slidesHome = true;
        slidesTargetPos = Constants.INTAKE_HOME_POS;
    }

    public void setSlidesPosition(double position) {
        if (position != lastSlidesPos) {
            slidesServo.setPosition(position);
            lastSlidesPos = position;
        }
    }

    public void update() {
        setSlidesPosition(slidesTargetPos + intakeOffset);
    }

    public boolean slidesIsHome() {
        return slidesHome;
    }

    // Intake Servo
    public void flipUp() {
        intakeServo.setPosition(Constants.INTAKE_UP_POS);
    }

    public void flipDown() {
        intakeServo.setPosition(Constants.INTAKE_DOWN_POS);
    }

    // Distance Sensor
    public double getDistance() {
        return intakeSensor.getDistance(DistanceUnit.MM);
    }

    public boolean intakeFull() {
        return getDistance() < Constants.INTAKE_DISTANCE_THRESHOLD;
    }
}
