package org.firstinspires.ftc.teamcode.RobotClasses;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {

    public static double INTAKE_POWER = 1;
    public static double INTAKE_DISTANCE_THRESHOLD = 100;

    // Deposit
    public static int HOME = 0;
    public static int LOW_GOAL = -60;
    public static int MID_GOAL = -250;
    public static int TOP_GOAL = -500;
    public static int CAP = -550;

    public static double DEPOSIT_POWER = 1;

    public static double DEPOSIT_AUTO_OPEN_POS = 0.18;
    public static double DEPOSIT_OPEN_POS = 0.05;
    public static double DEPOSIT_HOLD_POS = 0.28;
    public static double DEPOSIT_CLOSE_POS = 0.4;

    public static double TEAM_MARKER_HOME_POS = 0.3;
    public static double TEAM_MARKER_UP_POS = 0.7;
    public static double TEAM_MARKER_DOWN_POS = 1;

    public static double CAROUSEL_POWER = 0.55;

    public static double MAX_TURRET_POWER = 1;
    public static double TURRET_HOME_THETA = 0;

    public static int DEPOSIT_ARM_HOME = 0;
    public static int DEPOSIT_ARM_LOW_GOAL = 60;
    public static int DEPOSIT_ARM_MID_GOAL = 250;
    public static int DEPOSIT_ARM_TOP_GOAL = 500;
    public static int DEPOSIT_ARM_CAP = 550;
    public static int DEPOSIT_ARM_THRESHOLD = 100;
    public static double Deposit_ARM_POWER = 1;

    public static double DEPOSIT_SLIDES_INCHES_TO_TICKS = 1;
    public static double DEPOSIT_SLIDES_POWER = 1;
    public static double SLIDES_EXTENSION_DIST = 5;

}