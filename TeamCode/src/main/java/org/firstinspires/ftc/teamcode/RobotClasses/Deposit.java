package org.firstinspires.ftc.teamcode.RobotClasses;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@SuppressWarnings("FieldCanBeLocal")
@Config
public class Deposit {
    private DcMotorEx slidesMotor;
    private Servo armServo1;
    private Servo armServo2;
    private Servo depositServo;

    public static int SLIDES_HOME_THRESHOLD = 20;
    public static int SLIDES_ERROR_THRESHOLD = 50;
    public static double SLIDES_DRIFT_MULTIPLIER = 0.00021;

    // Slides PD
    public int slidesErrorChange = 0;
    public int slidesError = 0;
    public int slidesTarget = 0;
    public int slidesLastTicks = 0;

    //PID constants
    public static double slidesKp = 0.03;
    public static double slidesKd = 0;

    public static double slidesCapKp = 0.02;
    public static double slidesCapKd = 0;

    public static double gravityFF = .3;

    public double midOffset = 0;
    public double highOffset = 0;
    public double sharedOffset = 0;

    public double initialSlidesPos;

    private boolean isAuto;
    private boolean isExtended = false;

    public boolean reset = false;

    public Deposit(LinearOpMode op, boolean isAuto) {
        this(op, isAuto, 0);
    }

    //general mappings, init positions/modes
    public Deposit(LinearOpMode op, boolean isAuto, int initialSlidesPos) {
        this.isAuto = isAuto;

        // Deposit Servo

        depositServo = op.hardwareMap.get(Servo.class, "depositServo");
        armServo1 = op.hardwareMap.get(Servo.class, "arm1");
        armServo2 = op.hardwareMap.get(Servo.class, "arm2");
//        turretServo = op.hardwareMap.get(Servo.class, "turret");

//        turretHome();
        if (isAuto) setArmControls(Constants.ARM_INIT_POS);
        else armHome();

        if (isAuto) hold();
        else open();

        // Arm Motor
        slidesMotor = op.hardwareMap.get(DcMotorEx.class, "depositSlides");
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setTargetPosition(0);
        slidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.initialSlidesPos = initialSlidesPos;

        op.telemetry.addData("Status", "Deposit Initialized");
    }

    public void extendSlides () {extendSlides(Robot.DepositTarget.high);}


    //sets the slide height/position based on deposit level
    public void extendSlides(Robot.DepositTarget hub){
        if (hub == Robot.DepositTarget.high) slidesTarget = Constants.DEPOSIT_SLIDES_HIGH_TICKS + (int) Math.round(highOffset);
        else if (hub == Robot.DepositTarget.mid) slidesTarget = Constants.DEPOSIT_SLIDES_MID_TICKS + (int) Math.round(midOffset);
        else if (hub == Robot.DepositTarget.low || hub == Robot.DepositTarget.shared) slidesTarget = Constants.DEPOSIT_SLIDES_LOW_TICKS;
        else if (hub == Robot.DepositTarget.cap) slidesTarget = Constants.DEPOSIT_SLIDES_CAP_TICKS;

//        slidesTarget = Constants.DEPOSIT_SLIDES_HIGH_TICKS;
    }

    public void retractSlides(){
        slidesTarget = Constants.DEPOSIT_SLIDES_HOME_TICKS;

    }

    public void updateSlides(){
        updateSlides(false);
    }

    //Slide PD
    public void updateSlides(boolean capping){
        double slidesKp = !capping ? this.slidesKp : this.slidesCapKp;
        double slidesKd = !capping ? this.slidesKd : this.slidesCapKd;

        int currentTicks = getSlidesPos();
        initialSlidesPos += Math.abs(currentTicks - slidesLastTicks) * SLIDES_DRIFT_MULTIPLIER;
        slidesLastTicks = currentTicks;

        slidesErrorChange = slidesTarget - currentTicks - slidesError;
        slidesError = slidesTarget - currentTicks;

        slidesMotor.setPower(slidesKp * slidesError + slidesKd * slidesErrorChange + gravityFF);

        addPacket("initialSlidesPos", initialSlidesPos);
    }

    public void armOut() {
        armOut(Robot.DepositTarget.high);
    }

    public void armOut(Robot.DepositTarget hub) {
        if (hub == Robot.DepositTarget.shared) setArmControls(Constants.ARM_SHARED_POS + sharedOffset);
        else setArmControls(Constants.ARM_ALLIANCE_POS);
        isExtended = true;
    }

    public void armHome() {
        setArmControls(Constants.ARM_HOME_POS);
        isExtended = false;
    }

    public boolean isArmOut() {
        return isExtended;
    }

    public void setArmControls(double pos) {
        armServo1.setPosition((Math.max(0, Math.min(1, pos))));
        armServo2.setPosition(Math.max(0, Math.min(1, 1 - pos + Constants.ARM_OFFSET)));
    }

    public int getSlidesPos() {
        return slidesMotor.getCurrentPosition() + (int) Math.round(initialSlidesPos);
    }

    public boolean slidesisHome() {
        return getSlidesPos() - Constants.DEPOSIT_SLIDES_HOME_TICKS < SLIDES_HOME_THRESHOLD;
    }

    public boolean slidesAtPos() {
        return slidesError < SLIDES_ERROR_THRESHOLD;
    }

//    public void turretHome() {
//        turretServo.setPosition(Constants.TURRET_HOME);
//    }
//
//    public void turretLeft() {
//        turretServo.setPosition(Constants.TURRET_HOME - Constants.TURRET_TURN_DIST);
//    }
//
//    public void turretRight() {
//        turretServo.setPosition(Constants.TURRET_HOME + Constants.TURRET_TURN_DIST);
//    }

    //Deposit Servo
    public void hold() {
        setServoPos(Constants.DEPOSIT_HOLD_POS);
    }
    public void release() {
        setServoPos(Constants.DEPOSIT_RELEASE_POS);
    }
    public void open() {
        setServoPos(Constants.DEPOSIT_OPEN_POS);
    }
    public void setServoPos(double pos) {
        depositServo.setPosition(pos);
    }

}
