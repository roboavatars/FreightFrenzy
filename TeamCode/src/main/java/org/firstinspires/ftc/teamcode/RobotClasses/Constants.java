package org.firstinspires.ftc.teamcode.RobotClasses;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {

    // Drivetrain
    public static double ODO_NORMAL_POS = 0.33;
    public static double ODO_RETRACT_POS = 0.6;

    // Intake
    public static double INTAKE_DISTANCE_THRESHOLD = 70;
    public static double STALL_THRESHOLD = 6;

    // Intake Slides
    public static double INTAKE_EXTEND_POS = 0.3;
    public static double INTAKE_MIDWAY_POS = 0.6;
    public static double INTAKE_HOME_POS = 0.67;

    // Intake Servo
    public static double INTAKE_UP_POS = 0.48;
    public static double INTAKE_DOWN_POS = 0.83;

    // Deposit Servo
    public static double DEPOSIT_OPEN_POS = 0.2;
    public static double DEPOSIT_HOLD_POS = 0.45;

    // Deposit Arm
    public static int DEPOSIT_ARM_HOME = 0;
    public static int DEPOSIT_ARM_LOW = 755;
    public static int DEPOSIT_ARM_MID = 645;
    public static int DEPOSIT_ARM_HIGH = 540;
    public static int DEPOSIT_ARM_MIDWAY = 375;
    public static int ARM_ON_HUB_THRESHOLD = 520;

    // Deposit Slides
    public static double SLIDES_DISTANCE_HOME = 0;
    public static double SLIDES_DISTANCE_CLEAR = 2;
    public static double SLIDES_DISTANCE_LOW = 19;
    public static double SLIDES_DISTANCE_MID = 18;
    public static double SLIDES_DISTANCE_HIGH = 19;
    public static double SLIDES_DISTANCE_HIGH_AUTO = 8;
    public static double SLIDES_DISTANCE_DUCK = 21;

    public static double ARM_DISTANCE = 25;

    // Turret
    public static double TURRET_HOME_THETA = 0.53; // changed from 0.54 for quack
    public static double TURRET_ALLIANCE_RED_CYCLE_LOW_THETA = 0.18;
    public static double TURRET_ALLIANCE_RED_CYCLE_MID_THETA = 0.18;
    public static double TURRET_ALLIANCE_RED_CYCLE_HIGH_THETA = 0.18;
    public static double TURRET_NEUTRAL_RED_CYCLE_THETA = 0.8;
    public static double TURRET_DUCK_RED_CYCLE_THETA = 0.19;

    // Carousel
    public static double CAROUSEL_POWER = 0.55;
    public static double CAROUSEL_HOME = 0.9;
    public static double CAROUSEL_OUT = 0.15;

    // Anti-Tip
    public static double TIP_THRESHOLD = 0.1;
    public static double TIP_CONTROLS_GAIN = 5;

    // Tape Detector
    public static double COLOR_SENSOR_THRESHOLD = 400;
}
