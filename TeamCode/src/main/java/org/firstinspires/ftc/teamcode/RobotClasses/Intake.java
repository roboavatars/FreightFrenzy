package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@SuppressWarnings("FieldCanBeLocal")
public class Intake {
    private DcMotorEx intakeMotor;
    private DcMotorEx slidesMotor;
    private Servo intakeServo;
    private DistanceSensor intakeSensor;

    private double lastIntakePow = 0;
    private double lastBlockerPos = 0;

    public Intake(LinearOpMode op, boolean isAuto) {
        //Intake Motor
        intakeMotor = op.hardwareMap.get(DcMotorEx.class, "intake");
        off();

        //Slides Motor
        slidesMotor = op.hardwareMap.get(DcMotorEx.class, "intakeSlides");
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (isAuto){
            slidesMotor.setTargetPosition(Constants.INTAKE_HOME_TICKS);
        } else {
            slidesMotor.setTargetPosition(Constants.INTAKE_EXTEND_TICKS);
        }
        slidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Intake Servo
        intakeServo = op.hardwareMap.get(Servo.class, "intakeServo");
        if (isAuto){
            up();
        } else {
            down();
        }

        //Intake Sensor
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

    //Intake Slides
    public void extend (){
        slidesMotor.setPower(Constants.INTAKE_SLIDES_POWER);
        slidesMotor.setTargetPosition(Constants.INTAKE_EXTEND_TICKS);
    }
    public void retract (){
        slidesMotor.setPower(Constants.INTAKE_SLIDES_POWER);
        slidesMotor.setTargetPosition(Constants.INTAKE_HOME_TICKS);
    }

    //Intake Servo
    public void up (){
        intakeServo.setPosition(Constants.INTAKE_UP_POS);
    }

    public void down (){
        intakeServo.setPosition(Constants.INTAKE_DOWN_POS);
    }

    //Get Power
    public double getLastIntakePow(){
        return lastIntakePow;
    }

    public void checkIfStalling(){
        if (intakeMotor.getCurrent(CurrentUnit.AMPS)>Constants.STALL_THRESHOLD){
            off();
        }
    }

    // Distance Sensor
    private double getDistance() {
        return intakeSensor.getDistance(DistanceUnit.MM);
    }

    public boolean intakeFull() {
        return getDistance() < Constants.INTAKE_DISTANCE_THRESHOLD;
    }
}
