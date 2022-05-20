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
    public static int SLIDES_ERROR_THRESHOLD = 10;

    // Slides PD
    public int slidesErrorChange = 0;
    public int slidesError = 0;
    public int slidesTarget = 0;

    public double slidesKp = 0.03;
    public double slidesKd = 0;
    public double gravityFF = .3;


    public double lastArmPos = 0;

    private boolean isAuto;
    public boolean preload = false;

    public boolean reset = false;

    public int initialArmPos;

    public Deposit(LinearOpMode op, boolean isAuto, int initialArmPos, int initialSlidesPos) {
        this.isAuto = isAuto;

        // Deposit Servo

        depositServo = op.hardwareMap.get(Servo.class, "depositServo");
        armServo1 = op.hardwareMap.get(Servo.class, "arm1");
        armServo2 = op.hardwareMap.get(Servo.class, "arm2");
//        turretServo = op.hardwareMap.get(Servo.class, "turret");

//        turretHome();
        armHome();

        if (isAuto) hold();
        else open();

        // Arm Motor
        slidesMotor = op.hardwareMap.get(DcMotorEx.class, "depositSlides");
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setTargetPosition(0);
        slidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.initialArmPos = initialArmPos;

        op.telemetry.addData("Status", "Deposit Initialized");
    }

    public void extendSlides () {extendSlides(Robot.DepositTarget.high);}

    public void extendSlides(Robot.DepositTarget hub){
        slidesTarget = Constants.DEPOSIT_SLIDES_EXTEND_TICKS;
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
        armServo1.setPosition(Constants.ARM_DEPOSIT_POS);
        armServo2.setPosition(1 - Constants.ARM_DEPOSIT_POS);
    }

    public void armHome() {
        armServo1.setPosition(Constants.ARM_HOME_POS);
        armServo2.setPosition(1 - Constants.ARM_HOME_POS);
    }

    public int getSlidesPos() {
        return slidesMotor.getCurrentPosition();
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
        depositServo.setPosition(Constants.DEPOSIT_HOLD_POS);
    }
    public void release() {
        depositServo.setPosition(Constants.DEPOSIT_RELEASE_POS);
    }
    public void open() {
        depositServo.setPosition(Constants.DEPOSIT_OPEN_POS);
    }

}
