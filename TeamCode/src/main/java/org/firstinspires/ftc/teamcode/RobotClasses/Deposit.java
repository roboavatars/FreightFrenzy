package org.firstinspires.ftc.teamcode.RobotClasses;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@SuppressWarnings("FieldCanBeLocal")
@Config
public class Deposit {
    private DcMotorEx slidesMotor;
    private DcMotorEx armMotor;
    private Servo depositServo;

    public static int SLIDES_HOME_THRESHOLD = 50;
    public static int SLIDES_ERROR_THRESHOLD = 100;
    public static double SLIDES_DRIFT_MULTIPLIER = 0; //0.002;
    public static double SLIDES_STALL_THRESHOLD = 8; //0.002;
    public static int SLIDES_RESET_THRESHOLD = 100; //0.002;

    public static double ARM_TICKS_TO_RAD = 0.00628;
    public static double ARM_INITIAL_THETA = 0;
    public static int ARM_HOME_THRESHOLD = 30;
    public static int ARM_ERROR_THRESHOLD = 50;
    public static double ARM_MAX_POWER = 0.7;

    // Slides PD
    public int slidesErrorChange = 0;
    public int slidesError = 0;
    public int slidesTarget = 0;
    public int slidesLastTicks = 0;

    //SLides PID constants
    public static double slidesKp = 0.01;
    public static double slidesKd = 0;

    public static double slidesCapKp = 0.002;
    public static double slidesCapKd = 0;

    public static double slidesGravityFF = .1;

    // Arm PD
    public int armErrorChange = 0;
    public int armError = 0;
    public int armTarget = 0;
    public int armLastTicks = 0;

    //Arm PID constants
    public static double armUpKp = 0.01;
    public static double armDownKp = 0.008;
    public static double armUpKd = 0.03;
    public static double armDownKd = 0.02;
    public static double armGravityFF = 0.1;

    public int midOffset = 0;
    public double highOffset = 0;
    public double sharedOffset = 0;

    public double initialSlidesPos = 0;
    public double initialArmPos;

    private boolean isAuto;
    private boolean isExtended = false;
    private boolean slidesReset = false;

    public boolean reset = false;

    public Deposit(LinearOpMode op, boolean isAuto, double armInitPos) {
        this(op, isAuto, true, armInitPos);
    }

    public Deposit(LinearOpMode op, boolean isAuto, boolean resetEncoder, double armInitialPos) {
        this.isAuto = isAuto;
//        if (isAuto)
        this.initialArmPos = armInitialPos;
//        else this.initialArmPos = 0;

        // Deposit Servo

        depositServo = op.hardwareMap.get(Servo.class, "depositServo");

        if (isAuto) setArmControls((int) Math.round(initialArmPos));
        else armHome();

        if (isAuto) hold();
        else open();

        // Arm Motor
        armMotor = op.hardwareMap.get(DcMotorEx.class, "arm");
        if (resetEncoder) armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Slides Motor
        slidesMotor = op.hardwareMap.get(DcMotorEx.class, "depositSlides");
        if (resetEncoder) slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        this.initialSlidesPos = initialSlidesPos;

        op.telemetry.addData("Status", "Deposit Initialized");
    }

    public void update(Robot.DepositTarget hub) {
        updateSlides(hub);
        updateArm();
    }

    public void extendSlides () {extendSlides(Robot.DepositTarget.high);}


    //sets the slide height/position based on deposit level
    public void extendSlides(Robot.DepositTarget hub){
        if (hub == Robot.DepositTarget.high) slidesTarget = Constants.DEPOSIT_SLIDES_HIGH_TICKS + (int) Math.round(highOffset);
        else if (hub == Robot.DepositTarget.mid) slidesTarget = Constants.DEPOSIT_SLIDES_MID_TICKS + midOffset * Constants.DEPOSIT_SLIDES_MID_PRESET;
        else if (hub == Robot.DepositTarget.low) slidesTarget = Constants.DEPOSIT_SLIDES_LOW_TICKS;
        else if (hub == Robot.DepositTarget.shared) slidesTarget = Constants.DEPOSIT_SLIDES_SHARED_TICKS;
        else if (hub == Robot.DepositTarget.cap) slidesTarget = Constants.DEPOSIT_SLIDES_CAP_TICKS;
        else if (hub == Robot.DepositTarget.duck) slidesTarget = Constants.DEPOSIT_SLIDES_HIGH_TICKS + (int) Math.round(highOffset);
        else if (hub == Robot.DepositTarget.fastHigh) slidesTarget = Constants.DEPOSIT_SLIDES_FAST_HIGH_TICKS;

//        slidesTarget = Constants.DEPOSIT_SLIDES_HIGH_TICKS;
    }

    public void retractSlides(){
        slidesTarget = Constants.DEPOSIT_SLIDES_HOME_TICKS;

    }

    public void updateSlides(){
        updateSlides(Robot.DepositTarget.high);
    }

    //Slide PD
    public void updateSlides(Robot.DepositTarget hub){
        if (slidesTarget != Constants.DEPOSIT_SLIDES_HOME_TICKS) slidesReset = false;
        if (slidesTarget == Constants.DEPOSIT_SLIDES_HOME_TICKS && !slidesReset && getSlidesPos() < SLIDES_RESET_THRESHOLD) {
            slidesMotor.setPower(-1);
            if (getSlidesCurrent() > SLIDES_STALL_THRESHOLD) {
                slidesReset = true;
                initialSlidesPos -= getSlidesPos();
            }
        } else {
            double slidesKp = hub != Robot.DepositTarget.duck ? this.slidesKp : this.slidesCapKp;
            double slidesKd = hub != Robot.DepositTarget.duck ? this.slidesKd : this.slidesCapKd;

            int currentTicks = getSlidesPos();
            initialSlidesPos += Math.abs(currentTicks - slidesLastTicks) * SLIDES_DRIFT_MULTIPLIER;
            slidesLastTicks = currentTicks;

            slidesErrorChange = slidesTarget - currentTicks - slidesError;
            slidesError = slidesTarget - currentTicks;

            slidesMotor.setPower(slidesKp * slidesError + slidesKd * slidesErrorChange + slidesGravityFF);
        }

        addPacket("initialSlidesPos", initialSlidesPos);
    }

    public double getSlidesCurrent() {
        return slidesMotor.getCurrent(CurrentUnit.AMPS);
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

    public void updateArm() {
        int currentTicks = getArmPos();

        armErrorChange = armTarget - currentTicks - armError;
        armError = armTarget - currentTicks;

        double armKp = (armTarget == Constants.ARM_HOME_POS) ? armDownKp : armUpKp;
        double armKd = (armTarget == Constants.ARM_HOME_POS) ? armDownKd : armUpKd;

        armMotor.setPower(Math.max(-ARM_MAX_POWER, Math.min(ARM_MAX_POWER, armKp * armError + armKd * armErrorChange + armGravityFF * Math.cos(getArmTheta()))));
    }

    public void armOut() {
        armOut(Robot.DepositTarget.high);
    }

    public void armOut(Robot.DepositTarget hub) {
        if (hub == Robot.DepositTarget.shared) setArmControls(Constants.ARM_SHARED_POS + (int) Math.round(sharedOffset));
        else if (hub == Robot.DepositTarget.duck) setArmControls(Constants.ARM_DUCK_DEPOSIT_POS);
        else if (hub == Robot.DepositTarget.high) setArmControls(Constants.ARM_HIGH_POS);
        else if (hub == Robot.DepositTarget.fastHigh) setArmControls(Constants.ARM_FAST_HIGH_POS);
        else setArmControls(Constants.ARM_ALLIANCE_POS);
        isExtended = true;
    }

    public void armHome() {
        setArmControls(Constants.ARM_HOME_POS);
        isExtended = false;
    }

    public int getArmPos() {
        return armMotor.getCurrentPosition() + (int) Math.round(initialArmPos);
    }

    public double getArmTheta() {
        return ARM_INITIAL_THETA + getArmPos() * ARM_TICKS_TO_RAD;
    }

    public boolean isArmAtPos() {
        return armError < ARM_ERROR_THRESHOLD;
    }

    public boolean isArmHome() {
        return getArmPos() < ARM_HOME_THRESHOLD;
    }

    public boolean isArmOut() {
        return isExtended;
    }

    public void setArmControls(int pos) {
        armTarget = pos;
    }

    //Deposit Servo
    public void hold() {
        hold(Robot.DepositTarget.high);
    }

    public void hold(Robot.DepositTarget hub) {
        if (hub == Robot.DepositTarget.duck) setServoPos(Constants.DEPOSIT_DUCK_HOLD_POS);
        else setServoPos(Constants.DEPOSIT_HOLD_POS);
    }
    public void release() {
        release(Robot.DepositTarget.high);
    }

    public void release(Robot.DepositTarget hub) {
        if (hub == Robot.DepositTarget.shared) setServoPos(Constants.DEPOSIT_RELEASE_POS);
        else setServoPos(Constants.DEPOSIT_FLICK_POS);
    }
    public void open() {
        setServoPos(Constants.DEPOSIT_OPEN_POS);
    }
    public void setServoPos(double pos) {
        depositServo.setPosition(pos);
    }

}
