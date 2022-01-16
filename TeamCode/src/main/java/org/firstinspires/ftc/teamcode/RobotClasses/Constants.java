package org.firstinspires.ftc.teamcode.RobotClasses;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {

    //Intake
    public static double INTAKE_POWER = 1;
    public static double INTAKE_DISTANCE_THRESHOLD = 100;

    //Deposit Servo
    public static double DEPOSIT_OPEN_POS = 0.05;
    public static double DEPOSIT_CLOSE_POS = 0.4;

    //Team Marker
    public static double TEAM_MARKER_HOME_POS = 0.3;
    public static double TEAM_MARKER_UP_POS = 0.7;
    public static double TEAM_MARKER_DOWN_POS = 1;

    //Carousel
    public static double CAROUSEL_POWER = 0.55;

    //Deposit Turret
    public static double MAX_TURRET_POWER = 1;
    public static double TURRET_HOME_THETA = 0;
    public static double TURRET_MIN_THETA = -PI/2;
    public static double TURRET_MAX_THETA = PI/2;
    public static double TURRET_TICKS_PER_RADIAN = 103.6 * 20 / (2*PI);
    public static double TURRET_CENTER_TO_ROBOT_CENTER_DIST = 2.06066;

    //Deposit Arm
    public static double DEPOSIT_ARM_HOME_TICKS = 0;
    public static double DEPOSIT_ARM_LOW_GOAL_TICKS = 500;
    public static double DEPOSIT_ARM_MID_GOAL_TICKS = 400;
    public static double DEPOSIT_ARM_TOP_GOAL_TICKS = 300;
    public static double DEPOSIT_ARM_OVER_MOTOR = 100;

    //Deposit Slides
    public static double DEPOSIT_SLIDES_TICKS_PER_INCH = 103.6 / (PI * 1.2);
    public static double DEPOSIT_SLIDES_POWER = 1;
    public static double ARM_DIST_PLUS_DIST_FROM_END_OF_SLIDES_TO_ROBOT_CENTER_LOW_GOAL = 36; //So if the center of the robot is Xft away from the shipping hub, the slides don't need to extend Xft, since there is some distance between the end of the slides and the center of the robot, as well as the length of the arm. This is that distance combined (it's different for ever level of the shipping hub since the arm will be extended different amounts for each level)
    public static double ARM_DIST_PLUS_DIST_FROM_END_OF_SLIDES_TO_ROBOT_CENTER_MID_GOAL = 33;
    public static double ARM_DIST_PLUS_DIST_FROM_END_OF_SLIDES_TO_ROBOT_CENTER_TOP_GOAL = 30;
    public static double ARM_DIST_PLUS_DIST_FROM_END_OF_SLIDES_TO_ROBOT_CENTER_CAP_GOAL = 27;
    public static double DEPOSIT_SLIDES_MIN_TICKS = 0;
    public static double DEPOSIT_SLIDES_MAX_TICKS = 290;

    //Intake Stalling
    public static double STALL_THRESHOLD = 7;

    //Intake Slides
    public static double INTAKE_SLIDES_POWER = 1;
    public static int INTAKE_EXTEND_TICKS = 100;
    public static int INTAKE_HOME_TICKS = 0;
    public static int INTAKE_SLIDES_HOME_THRESHOLD = 10;
    public static double TRANSFER_TIME = 1000;

    //Intake Servo
    public static double INTAKE_UP_POS = 0;
    public static double INTAKE_DOWN_POS = 0.8;

    //Anti-Tip
    public static double TIP_THRESHOLD = PI/6;
    public static double TIP_CONTROLS_GAIN = PI/6;

    //Tape Detector
    public static double COLOR_SENSOR_THRESHOLD = 140;
    public static double TAPE_THETA_THRESHOLD = 0.2;
    public static double TAPE_SENSORS_DIST = 10;
    public static double TAPE_SENSOR_CENTER_TO_ROBOT_CENTER = 8;
}