package org.firstinspires.ftc.teamcode.RobotClasses;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {

    // Drivetrain
    public static double ODO_NORMAL_POS = 0.45;
    public static double ODO_RETRACT_POS = 0.7;

    // Intake
    public static double INTAKE_DISTANCE_THRESHOLD = 10;
    public static double STALL_THRESHOLD = 6;
    public static double INTAKE_COLOR_THRESHOLD = 5000;

    // Intake Slides
    public static int INTAKE_SLIDES_EXTEND_TICKS = 100;
//    public static double INTAKE_MIDWAY_POS = 0.6;
    public static int INTAKE_SLIDES_HOME_TICKS = 10;


    // Intake Servo
    public static double INTAKE_UP_POS = 0.75;
    public static double INTAKE_DOWN_POS = 0.08;
    public static double INTAKE_UP_INIT_POS = 0.8;

    // Deposit Servo
    public static double DEPOSIT_OPEN_POS = 0.65;
    public static double DEPOSIT_HOLD_POS = 0;
    public static double DEPOSIT_RELEASE_POS = 1;

    // Deposit Arm
    public static double ARM_DEPOSIT_POS = 0.2;
    public static double ARM_HOME_POS = 0.87;
    public static double ARM_INIT_POS = 0.75;

    public static int DEPOSIT_SLIDES_HOME_TICKS = 0;
    public static int DEPOSIT_SLIDES_HIGH_TICKS = 500;
    public static int DEPOSIT_SLIDES_MID_TICKS = 200;
    public static int DEPOSIT_SLIDES_LOW_TICKS = 0;
    public static double DEPOSIT_SLIDES_POWER = 1;

    // Carousel
    public static double CAROUSEL_VELOCITY_SLOW = 300;
    public static double CAROUSEL_VELOCITY_FAST = 1270;
    public static double CAROUSEL_SPEED_UP_THRESHOLD = 1100;

    // Anti-Tip
    public static double TIP_THRESHOLD = 0.1;
    public static double TIP_CONTROLS_GAIN = 5;

    // Tape Detector
    public static double COLOR_SENSOR_THRESHOLD = 110;

    //Turret?
    public static double TURRET_NETRUAL_THETA = PI;

    public static double TURRET_HOME = .51;
    public static double TURRET_TURN_DIST = 0.35;

    //Stick Capping
    public static double SERVO_CAP_DOWN = 0.11;
    public static double SERVO_CAP_UP = 0.43;
    public static double SERVO_CAP_HOME = 0.97;

}
