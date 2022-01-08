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

    //Deposit Arm
    public static double DEPOSIT_ARM_HOME = 1;
    public static double DEPOSIT_ARM_LOW_GOAL = 0.01;
    public static double DEPOSIT_ARM_MID_GOAL = 0.15;
    public static double DEPOSIT_ARM_TOP_GOAL = 0.3;
    public static double DEPOSIT_ARM_CAP = 0.35;
    public static double DEPOSIT_ARM_OVER_MOTOR = .9;
    public static double DEPOSIT_ARM_SERVO_OFFSET = 0.05;

    public static double DEPOSIT_SLIDES_TICKS_PER_INCH = 1;
    public static double DEPOSIT_SLIDES_POWER = 1;
    public static double ARM_DIST_PLUS_DIST_FROM_END_OF_SLIDES_TO_ROBOT_CENTER_LOW_GOAL = 36; //So if the center of the robot is Xft away from the shipping hub, the slides don't need to extend Xft, since there is some distance between the end of the slides and the center of the robot, as well as the length of the arm. This is that distance combined (it's different for ever level of the shipping hub since the arm will be extended different amounts for each level)
    public static double ARM_DIST_PLUS_DIST_FROM_END_OF_SLIDES_TO_ROBOT_CENTER_MID_GOAL = 33;
    public static double ARM_DIST_PLUS_DIST_FROM_END_OF_SLIDES_TO_ROBOT_CENTER_TOP_GOAL = 30;
    public static double ARM_DIST_PLUS_DIST_FROM_END_OF_SLIDES_TO_ROBOT_CENTER_CAP_GOAL = 27;

    public static double STALL_THRESHOLD = 7;

    //////////////////////////////////////////////////////////////////////////
    // Old Deposit
    public static int HOME = 0;
    public static int LOW_GOAL = -60;
    public static int MID_GOAL = -250;
    public static int TOP_GOAL = -500;
    public static int CAP = -550;
    public static double DEPOSIT_POWER = 1;

    public static double DEPOSIT_HOLD_POS = 0.28;
    public static double DEPOSIT_AUTO_OPEN_POS = 0.18;
    //////////////////////////////////////////////////////////////////////////////
}