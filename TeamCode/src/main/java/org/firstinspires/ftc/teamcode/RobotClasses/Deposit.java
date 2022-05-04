package org.firstinspires.ftc.teamcode.RobotClasses;

import static java.lang.Math.PI;

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
    private Servo turretServo;
    //private Servo depositServo;

    private double lastServoPos = 0; // deposit servo
    private double lastServoPos2 = 0; // limit servo

    public double ARM_TICKS_PER_RADIAN = 540 / PI;
    public static int SLIDES_HOME_THRESHOLD = 5;
    public static int DEPOSIT_ARM_DEPOSIT_THRESHOLD = 30;

    public int targetArmPos = 0;

    private double initialArmAngle = -PI / 6;

    // Arm PD
    double armErrorChange = 0, armError = 0;
    public static double pArmUp = 0.004;
    public static double dArmUp = 0.045;
    public static double fGravityUp = 0.1;

    public static double pArmDown = 0.002;
    public static double dArmDown = 0.05;
    public static double fGravityDown = 0;

    public static double pArm = pArmUp;
    public static double dArm = dArmUp;
    public static double fGravity = fGravityUp;

    public static double armMaxPower = 1;

    public boolean depositing = false;

    public double lastArmPos = 0;

    private boolean isAuto;
    public boolean preload = false;

    public boolean reset = false;

    public int initialArmPos;

    public Deposit(LinearOpMode op, boolean isAuto, int initialArmPos, int initialSlidesPos) {
        this.isAuto = isAuto;

        // Deposit Servo

        armServo1 = op.hardwareMap.get(Servo.class, "arm1");
        armServo2 = op.hardwareMap.get(Servo.class, "arm2");
        turretServo = op.hardwareMap.get(Servo.class, "turret");
        //depositServo = op.hardwareMap.get(Servo.class, "depositServo");

        turretHome();
        armHome();

        // Arm Motor
        slidesMotor = op.hardwareMap.get(DcMotorEx.class, "depositSlides");
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setTargetPosition(0);
        slidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.initialArmPos = initialArmPos;

        op.telemetry.addData("Status", "Deposit Initialized");
    }

    public void extendSlides(){
        slidesMotor.setTargetPosition(Constants.DEPOSIT_SLIDES_EXTEND_TICKS);
        slidesMotor.setPower(1);
    }

    public void retractSlides(){
        slidesMotor.setTargetPosition(Constants.DEPOSIT_SLIDES_HOME_TICKS);
        slidesMotor.setPower(1);
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
        return getSlidesPos() < SLIDES_HOME_THRESHOLD;
    }

    public void turretHome() {
        turretServo.setPosition(Constants.TURRET_HOME);
    }

    public void turretLeft() {
        turretServo.setPosition(Constants.TURRET_HOME - Constants.TURRET_TURN_DIST);
    }

    public void turretRight() {
        turretServo.setPosition(Constants.TURRET_HOME + Constants.TURRET_TURN_DIST);
    }
}
