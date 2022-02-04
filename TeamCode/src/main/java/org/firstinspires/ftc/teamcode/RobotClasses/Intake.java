package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@SuppressWarnings("FieldCanBeLocal")
public class Intake {
    private DcMotorEx intakeMotor;
    private Servo slidesServo;
    private Servo intakeServo;
    private DistanceSensor intakeSensor;
    private TouchSensor slidesSensor;

    private double lastIntakePow = 0;
    private boolean slidesHome;

    public Intake(LinearOpMode op, boolean isAuto) {
        // Intake Motor
        intakeMotor = op.hardwareMap.get(DcMotorEx.class, "intake");

        // Slides Motor
        slidesServo = op.hardwareMap.get(Servo.class, "intakeSlides");
        if (isAuto) {
            home();
        } else {
            extend();
        }

        // Intake Servo
        intakeServo = op.hardwareMap.get(Servo.class, "intakeServo");
        if (isAuto) {
            flipUp();
        } else {
            flipDown();
        }

        // Intake Distance Sensor
        intakeSensor = op.hardwareMap.get(DistanceSensor.class, "intakeSensor");

        // Intake Slides Limit Switch
        slidesSensor = op.hardwareMap.get(TouchSensor.class, "intakeSlidesLimit");

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

    public void checkIfStalling() {
        if (intakeMotor.getCurrent(CurrentUnit.AMPS) > Constants.STALL_THRESHOLD) {
            off();
        }
    }

    // Intake Slides
    public void extend() {
        slidesHome = false;
        slidesServo.setPosition(Constants.INTAKE_EXTEND_POS);
    }

    public void home() {
        slidesHome = true;
        slidesServo.setPosition(Constants.INTAKE_HOME_POS);
    }

    public boolean slidesIsHome() {
//        return slidesSensor.isPressed();
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
    private double getDistance() {
        return intakeSensor.getDistance(DistanceUnit.MM);
    }

    public boolean intakeFull() {
        return getDistance() < Constants.INTAKE_DISTANCE_THRESHOLD;
    }
}
