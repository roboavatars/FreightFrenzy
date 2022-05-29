package org.firstinspires.ftc.teamcode.RobotClasses;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@SuppressWarnings("FieldCanBeLocal")
public class Intake {
    private DcMotorEx intakeMotor;
    private DcMotorEx slidesMotor;
    private Servo flipServo;
    private RevColorSensorV3 intakeSensor;

    private double lastIntakePow = 0;
    public static int slidesErrorThreshold = 3;

    public int initialSlidesPos;

    public double INTAKE_SLIDES_SERVO_SPEED = 0.1;
    public double HOME_THRESHOLD = 20;

    public int slidesErrorChange = 0;
    public int slidesError = 0;
    public int slidesTarget = 0;

    public static double slidesKp = 0.05;
    public static double slidesKd = 0.0;
    public static double accelFF =  0;

    private LinearOpMode op;

    public Intake(LinearOpMode op, boolean isAuto) {
        this(op, isAuto, 0);
    }

    public Intake(LinearOpMode op, boolean isAuto, int initialSlidesPos) {
        intakeMotor = op.hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        flipServo = op.hardwareMap.get(Servo.class, "intakeServo");

        slidesMotor = op.hardwareMap.get(DcMotorEx.class, "intakeSlides");
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setTargetPosition(0);
        slidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.initialSlidesPos = initialSlidesPos;

        intakeSensor = op.hardwareMap.get(RevColorSensorV3.class, "intakeSensor");
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
        return getCurrent() > Constants.STALL_THRESHOLD;
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
//        slidesMotor.setTargetPosition(position);
//        slidesMotor.setPower(Constants.INTAKE_SLIDES_POWER);
        slidesTarget = position;
    }

    public void updateSlides(double ay){
        int currentTicks = getSlidesPos();
        slidesErrorChange = slidesTarget - currentTicks - slidesError;
        slidesError = slidesTarget - currentTicks;

        if (Math.abs(slidesError) > slidesErrorThreshold) slidesMotor.setPower(slidesKp * slidesError + slidesKd * slidesErrorChange + accelFF * ay);
        else slidesMotor.setPower(0);
    }

    public int getSlidesPos() {
        return slidesMotor.getCurrentPosition() + initialSlidesPos;
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
        return intakeSensor.getDistance(DistanceUnit.MM);
    }

    public boolean isFull() {
        return getDistance() < Constants.INTAKE_DISTANCE_THRESHOLD;
    }

    public String getElement() {
        String element;
        float [] hsv = {0F, 0F, 0F};
        Color.RGBToHSV(intakeSensor.red(), intakeSensor.green(),  intakeSensor.blue(), hsv);
        if (hsv[0] < Constants.COLOR_SENSOR_THRESHOLD)
            element = "cube";
        else
            element =  "ball";
        return element;
    }
}
