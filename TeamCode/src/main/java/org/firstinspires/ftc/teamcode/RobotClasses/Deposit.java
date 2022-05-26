package org.firstinspires.ftc.teamcode.RobotClasses;

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
    public static int SLIDES_ERROR_THRESHOLD = 20;

    // Slides PD
    public int slidesErrorChange = 0;
    public int slidesError = 0;
    public int slidesTarget = 0;

    public double slidesKp = 0.03;
    public double slidesKd = 0;
    public double gravityFF = .3;


    public double initialSlidesPos;

    private boolean isAuto;

    public boolean reset = false;

    public Deposit(LinearOpMode op, boolean isAuto) {
        this(op, isAuto, 0);
    }
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

    public void extendSlides(Robot.DepositTarget hub){
        if (hub == Robot.DepositTarget.high) slidesTarget = Constants.DEPOSIT_SLIDES_HIGH_TICKS;
        else if (hub == Robot.DepositTarget.mid) slidesTarget = Constants.DEPOSIT_SLIDES_MID_TICKS;
        else if (hub == Robot.DepositTarget.low) slidesTarget = Constants.DEPOSIT_SLIDES_LOW_TICKS;
        else if (hub == Robot.DepositTarget.cap) slidesTarget = Constants.DEPOSIT_SLIDES_CAP_TICKS;

//        slidesTarget = Constants.DEPOSIT_SLIDES_HIGH_TICKS;
    }

    public void retractSlides(){
        slidesTarget = Constants.DEPOSIT_SLIDES_HOME_TICKS;

    }

    public void updateSlides(){
        int currentTicks = getSlidesPos();
        slidesErrorChange = slidesTarget - currentTicks - slidesError;
        slidesError = slidesTarget - currentTicks;

        slidesMotor.setPower(slidesKp * slidesError + slidesKd * slidesErrorChange + gravityFF);
    }

    public void armOut() {
        setArmControls(Constants.ARM_DEPOSIT_POS);

    }

    public void armHome() {
        setArmControls(Constants.ARM_HOME_POS);
    }

    public void setArmControls(double pos) {
        armServo1.setPosition(pos);
        armServo2.setPosition(1 - pos);
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
