package org.firstinspires.ftc.teamcode.RobotClasses;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {

    // Intake
    public static double INTAKE_POWER = 1;
    public static double INTAKE_DISTANCE_THRESHOLD = 80;

    // Intake Stalling
    public static double STALL_THRESHOLD = 7;

    // Intake Slides
    public static double INTAKE_EXTEND_POS = 0.5;
    public static double INTAKE_HOME_POS = 0.2;

    // Intake Servo
    public static double INTAKE_UP_POS = 0.15;
    public static double INTAKE_DOWN_POS = 0.85;

    // Deposit Servo
    public static double DEPOSIT_OPEN_POS = 0.9;
    public static double DEPOSIT_HOLD_POS = 0.65;

    // Deposit Arm
    public static int DEPOSIT_ARM_HOME = 0;
    public static int DEPOSIT_ARM_LOW = 750;
    public static int DEPOSIT_ARM_MID = 650;
    public static int DEPOSIT_ARM_HIGH = 570;
    public static int DEPOSIT_ARM_MIDWAY = 400;

    // Deposit Slides
    public static double SLIDES_DISTANCE_LOW = 19;
    public static double SLIDES_DISTANCE_MID = 19;
    public static double SLIDES_DISTANCE_HIGH = 19;
    public static double SLIDES_DISTANCE_DUCK = 25;

    public static double ARM_DISTANCE = 25;

    //Turret
    public static double TURRET_ALLIANCE_RED_CYCLE_THETA = 0.175;
    public static double TURRET_NEUTRAL_RED_CYCLE_THETA = 0;
    public static double TURRET_DUCK_RED_CYCLE_THETA = 0;

    // Carousel
    public static double CAROUSEL_POWER = 0.55;

    // Anti-Tip
    public static double TIP_THRESHOLD = 0.1;
    public static double TIP_CONTROLS_GAIN = 5;

    // Tape Detector
    public static double COLOR_SENSOR_THRESHOLD = 400;
    public static double TAPE_THETA_THRESHOLD = PI/20;
    public static double TAPE_SENSORS_DIST = 10;
    public static double TAPE_SENSOR_CENTER_TO_ROBOT_CENTER = 8;
}
