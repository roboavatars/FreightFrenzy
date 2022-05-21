package org.firstinspires.ftc.teamcode.RobotClasses;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;

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

    private boolean slidesHome = true;
    public double intakeOffset = 0;
    private double lastSlidesPos = 0;
    private double slidesTargetPos = 0;

    public double INTAKE_SLIDES_SERVO_SPEED = 0.1;
    public double HOME_THRESHOLD = 20;

    public int slidesErrorChange = 0;
    public int slidesError = 0;
    public int slidesTarget = 0;

    public static double slidesKp = 0.1;
    public static double slidesKd = 0.045;
    public static double accelFF = 0.0008;

    private LinearOpMode op;

    public Intake(LinearOpMode op, boolean isAuto) {
        intakeMotor = op.hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        flipServo = op.hardwareMap.get(Servo.class, "intakeServo");

        slidesMotor = op.hardwareMap.get(DcMotorEx.class, "intakeSlides");
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setTargetPosition(0);
        slidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
//        slidesMotor.setTargetPosition(position);
//        slidesMotor.setPower(Constants.INTAKE_SLIDES_POWER);
        slidesTarget = position;
    }

    public void updateSlides(double ay){
        int currentTicks = getSlidesPos();
        slidesErrorChange = slidesTarget - currentTicks - slidesError;
        slidesError = slidesTarget - currentTicks;

        slidesMotor.setPower(slidesKp * slidesError + slidesKd * slidesErrorChange + accelFF * ay);
        addPacket("ay", ay);
        addPacket("ff", accelFF * ay);
    }

    public int getSlidesPos() {
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
        return intakeSensor.getDistance(DistanceUnit.MM);
    }

    public boolean isFull() {
        return getDistance() < Constants.INTAKE_DISTANCE_THRESHOLD;
    }

    public String getElement() {
        String element;
        if (intakeSensor.blue() < Constants.COLOR_SENSOR_THRESHOLD)
            element = "cube";
        else
            element =  "ball";

        addPacket("element", element);
        return element;
    }
}
