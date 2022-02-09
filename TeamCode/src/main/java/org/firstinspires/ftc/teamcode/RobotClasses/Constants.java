package org.firstinspires.ftc.teamcode.RobotClasses;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {

    // Intake
    public static double INTAKE_POWER = 1;
    public static double INTAKE_DISTANCE_THRESHOLD = 100;

    // Deposit Servo
    public static double DEPOSIT_OPEN_POS = 0.9;
    public static double DEPOSIT_CLOSE_POS = 0.3;
    public static double DEPOSIT_HOLD_POS = 0.65;

    // Carousel
    public static double CAROUSEL_POWER = 0.55;

    // Deposit Arm
    public static int DEPOSIT_ARM_HOME = 0;
    public static int DEPOSIT_ARM_LOW = 750;
    public static int DEPOSIT_ARM_MID = 600;
    public static int DEPOSIT_ARM_HIGH = 540;
    public static int DEPOSIT_ARM_MIDWAY = 400;
    public static int DEPOSIT_ARM_OVER_SLIDES_MOTOR = 30;

    // Deposit Slides
    public static double ARM_DISTANCE_LOW = 36;
    public static double ARM_DISTANCE_MID = 33;
    public static double ARM_DISTANCE_HIGH = 24;
    public static double ARM_DISTANCE_CAP = 27;

    // Intake Stalling
    public static double STALL_THRESHOLD = 7;

    // Intake Slides
    public static double INTAKE_EXTEND_POS = 0.2;
    public static double INTAKE_HOME_POS = 0.63;

    // Intake Servo
    public static double INTAKE_UP_POS = 0.15;
    public static double INTAKE_DOWN_POS = 0.85;

    // Anti-Tip
    public static double TIP_THRESHOLD = 0.1;
    public static double TIP_CONTROLS_GAIN = 5;

    // Tape Detector
    public static double COLOR_SENSOR_THRESHOLD = 400;
    public static double TAPE_THETA_THRESHOLD = PI/20;
    public static double TAPE_SENSORS_DIST = 10;
    public static double TAPE_SENSOR_CENTER_TO_ROBOT_CENTER = 8;
}
