package org.firstinspires.ftc.teamcode.RobotClasses;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {

    // Drivetrain
    public static double ODO_NORMAL_POS = 0.45;
    public static double ODO_RETRACT_POS = 0.7;

    // Intake
    public static double INTAKE_DISTANCE_THRESHOLD = 70;
    public static double STALL_THRESHOLD = 6;
    public static double INTAKE_TRANSFER_POWER = -1;

    // Intake Slides
    public static int INTAKE_SLIDES_EXTEND_TICKS = 100;
//    public static double INTAKE_MIDWAY_POS = 0.6;
    public static int INTAKE_SLIDES_HOME_TICKS = 0;
    public static double INTAKE_HOME_INIT_POS = 0.8;
    public static double INTAKE_SLIDES_POWER = 1;

    // Intake Servo
    public static double INTAKE_UP_POS = 0.75;
    public static double INTAKE_DOWN_POS = 0.13;
    public static double INTAKE_UP_INIT_POS = 0.8;

    // Deposit Servo
    public static double DEPOSIT_OPEN_POS = 0.5;
    public static double DEPOSIT_HOLD_POS = 0.76;
    public static double DEPOSIT_RELEASE_POS = 0.5;
    public static double DEPOSIT_CAP_POS = 0.2;

    // Limit Servo
    public static double LIMIT_OPEN = 0.38;
    public static double LIMIT_CLOSE = 0.1;

    // Deposit Arm
    public static int ARM_HIGH_POS = 570;
    public static int ARM_MID_POS = 640;
    public static int ARM_NEUTRAL_POS = 650;
    public static int FAKE_ARM_HOME_POS = 0;
//    public static int ARM_ON_HUB_THRESHOLD = ARM_HIGH_POS - 20;
    public static int ARM_ROTATE_TURRET_THRESHOLD = 600;
    public static int ARM_CAP_DOWN_POS = 725;
    public static int ARM_CAP_UP_POS = 450;
    public static double ARM_HOLD_DEPOSIT_POWER = 0.3;

    public static double ARM_DEPOSIT_POS = 1;
    public static double ARM_HOME_POS = 0.3;

    public static int DEPOSIT_SLIDES_HOME_TICKS = 0;
    public static int DEPOSIT_SLIDES_EXTEND_TICKS = 170;
    public static double DEPOSIT_SLIDES_POWER = 1;

    // Carousel
    public static double CAROUSEL_VELOCITY_SLOW = 300;
    public static double CAROUSEL_VELOCITY_FAST = 1270;
    public static double CAROUSEL_SPEED_UP_THRESHOLD = 1100;

    // Anti-Tip
    public static double TIP_THRESHOLD = 0.1;
    public static double TIP_CONTROLS_GAIN = 5;

    // Tape Detector
    public static double COLOR_SENSOR_THRESHOLD = 400;

    //Turret?
    public static double TURRET_NETRUAL_THETA = PI;

    //Stick Capping
    public static double SERVO_CAP_DOWN = 0.11;
    public static double SERVO_CAP_UP = 0.43;
    public static double SERVO_CAP_HOME = 0.97;

}
