package org.firstinspires.ftc.teamcode.RobotClasses;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
@SuppressWarnings("FieldCanBeLocal")
public class Intake {
    private DcMotorEx intakeMotor;
    private DcMotorEx slidesMotor;
    private Servo flipServo;
    private OpticalDistanceSensor intakeSensor;

    private double lastIntakePow = 0;
    public static int slidesErrorThreshold = 5;

    public double initialSlidesPos = 0;

    public double INTAKE_SLIDES_SERVO_SPEED = 0.1;
    public static double HOME_THRESHOLD = 60;

    public int slidesErrorChange = 0;
    public int slidesError = 0;
    public int slidesTarget = 0;

    //PID constantly

    public static double slidesKp = 0.01;
    public static double slidesKd = 0.01;
    public static double accelFF = 0;
    public static double springFF = 0;

    private LinearOpMode op;
    private boolean isAuto;

    public Intake(LinearOpMode op, boolean isAuto) {
        this(op, isAuto, true);
    }

    //mapping, general intake/slide modes
    public Intake(LinearOpMode op, boolean isAuto, boolean resetEncoder) {
        intakeMotor = op.hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flipServo = op.hardwareMap.get(Servo.class, "intakeServo");

        if (isAuto) flipUp();
        else flipDown();

        slidesMotor = op.hardwareMap.get(DcMotorEx.class, "intakeSlides");
//        slidesMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        if (resetEncoder) slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        slidesMotor.setTargetPosition(0);
        slidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        this.initialSlidesPos = initialSlidesPos;
        this.isAuto = isAuto;

        intakeSensor = op.hardwareMap.get(OpticalDistanceSensor.class, "intakeSensor");
    }

    // Intake Motor
    public void on() {
        on(Robot.DepositTarget.high);
    }

    public void on(Robot.DepositTarget hub) {
        if (hub == Robot.DepositTarget.high) setPower(Constants.INTAKE_POWER);
        else setPower(Constants.INTAKE_DUCK_POWER);
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

    public boolean checkIfStalling() {
        return getCurrent() > Constants.INTAKE_STALL_THRESHOLD;
    }

    public double getCurrent() {
        return intakeMotor.getCurrent(CurrentUnit.AMPS);
    }

    // Intake Slides
    public void extend() {
        setSlidesPosition(Constants.INTAKE_SLIDES_EXTEND_TICKS);
    }

    public void home() {
        home(Robot.DepositTarget.high);
    }

    public void home(Robot.DepositTarget hub) {
        if (hub == Robot.DepositTarget.duck) setSlidesPosition(Constants.INTAKE_SLIDES_DUCK_HOME_TICKS);
        else setSlidesPosition(Constants.INTAKE_SLIDES_HOME_TICKS);
    }

    public void setSlidesPosition(int position) {
//        slidesMotor.setTargetPosition(position);
//        slidesMotor.setPower(Constants.INTAKE_SLIDES_POWER);
        slidesTarget = Math.min(Constants.INTAKE_SLIDES_EXTEND_TICKS, Math.max(position, 0));
    }

    //runs pid for slides
    public void updateSlides(double ay){
        int currentTicks = getSlidesPos();
        slidesErrorChange = slidesTarget - currentTicks - slidesError;
        slidesError = slidesTarget - currentTicks;

        if (Math.abs(slidesError) > slidesErrorThreshold &&
                slidesMotor.getCurrent(CurrentUnit.AMPS) < Constants.INTAKE_SLIDES_STALL_THRESHOLD)
            slidesMotor.setPower(slidesKp * slidesError + slidesKd * slidesErrorChange + accelFF * ay + springFF * (Constants.INTAKE_SLIDES_EXTEND_TICKS - getSlidesPos()));
        else slidesMotor.setPower(0);
        addPacket("intake slides current", slidesMotor.getCurrent(CurrentUnit.AMPS));

    }

    public int getSlidesPos() {
        return slidesMotor.getCurrentPosition() + (int) Math.round(initialSlidesPos);
    }

    public boolean slidesIsHome() {
        return getSlidesPos() - Constants.INTAKE_SLIDES_HOME_TICKS < HOME_THRESHOLD;
    }

    // Intake Servo
    public void flipUp() {
        flipUp(Robot.DepositTarget.high);
    }

    public void flipUp(Robot.DepositTarget hub) {
        if (hub == Robot.DepositTarget.duck) flipServo.setPosition(Constants.INTAKE_UP_DUCK_POS);
        else flipServo.setPosition(Constants.INTAKE_UP_POS);
    }

    public void flipDown() {
        flipDown(Robot.DepositTarget.high);
    }

    public void flipDown(Robot.DepositTarget hub) {
        if (hub == Robot.DepositTarget.duck) flipServo.setPosition(Constants.INTAKE_DOWN_DUCKS_POS);
        else flipServo.setPosition(Constants.INTAKE_DOWN_POS);
    }

    // Distance Sensor
    public double getDistance() {
        return intakeSensor.getLightDetected();
    }

    //checks if block/ball is in intake
    public boolean isFull() {
        return getDistance() > (isAuto? Constants.INTAKE_DISTANCE_THRESHOLD_AUTO : Constants.INTAKE_DISTANCE_THRESHOLD_TELE);
    }

    public boolean transferred() {
        return getDistance() < Constants.INTAKE_SENSOR_TRANSFER_THRESHOLD;
    }

//    public String getElement() {
//        String element;
//        float [] hsv = {0F, 0F, 0F};
//        Color.RGBToHSV(intakeSensor.red(), intakeSensor.green(),  intakeSensor.blue(), hsv);
//        if (hsv[0] < Constants.COLOR_SENSOR_THRESHOLD)
//            element = "cube";
//        else
//            element =  "ball";
//        return element;
//    }
}
