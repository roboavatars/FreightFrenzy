package org.firstinspires.ftc.teamcode.RobotClasses;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {

    //Intake
    public static double INTAKE_POWER = 1;
    public static double INTAKE_DISTANCE_THRESHOLD = 100;

    //Deposit Servo
    public static double DEPOSIT_OPEN_POS = 0.9;
    public static double DEPOSIT_CLOSE_POS = 0.3;
    public static double DEPOSIT_HOLD_POS = 0.65;

    //Team Marker
    public static double TEAM_MARKER_HOME_POS = 0.3;
    public static double TEAM_MARKER_UP_POS = 0.7;
    public static double TEAM_MARKER_DOWN_POS = 1;

    //Carousel
    public static double CAROUSEL_POWER = 0.55;

    //Deposit Arm
    public static int DEPOSIT_ARM_HOME = 0;
    public static int DEPOSIT_ARM_LOW = 750;
    public static int DEPOSIT_ARM_MID = 600;
    public static int DEPOSIT_ARM_HIGH = 350;
    public static int DEPOSIT_ARM_TRANSFER = 100;
    public static double DEPOSIT_ARM_MAX_POWER = 0.25;
    public static int DEPOSIT_ARM_ERROR_THRESHOLD = 5;

    //Deposit Slides
    public static double DEPOSIT_SLIDES_TICKS_PER_INCH = 103.6 / (PI * 1.2);
    public static double DEPOSIT_SLIDES_POWER = 1;
    public static double ARM_DISTANCE_LOW = 36;
    public static double ARM_DISTANCE_MID = 33;
    public static double ARM_DISTANCE_HIGH = 30;
    public static double ARM_DISTANCE_CAP = 27;
    public static double DEPOSIT_SLIDES_MIN_TICKS = 0;
    public static double DEPOSIT_SLIDES_MAX_TICKS = 290;
    public static int DEPOSIT_SLIDES_ERROR_THRESHOLD = 5;

    //Intake Stalling
    public static double STALL_THRESHOLD = 7;

    //Intake Slides
    public static double INTAKE_EXTEND_POS = 0.2;
    public static double INTAKE_HOME_POS = 0.65;
    public static double TRANSFER_TIME = 1000;

    //Intake Servo
    public static double INTAKE_UP_POS = 0.15;
    public static double INTAKE_DOWN_POS = 0.85;

    //Anti-Tip
    public static double TIP_THRESHOLD = 0.1;
    public static double TIP_CONTROLS_GAIN = 5;

    //Tape Detector
    public static double COLOR_SENSOR_THRESHOLD = 400;
    public static double TAPE_THETA_THRESHOLD = PI/20;
    public static double TAPE_SENSORS_DIST = 10;
    public static double TAPE_SENSOR_CENTER_TO_ROBOT_CENTER = 8;

    public static double constrainToRange(double value, double min, double max){
        return Math.min(Math.max(value, min), max);
    }
}