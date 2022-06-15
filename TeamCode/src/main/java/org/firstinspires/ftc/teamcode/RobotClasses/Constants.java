package org.firstinspires.ftc.teamcode.RobotClasses;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {

    // Drivetrain
    public static double VERT_ODO_NORMAL_POS = 0.7;
    public static double VERT_ODO_RETRACT_POS = 0;
    public static double LAT_ODO_NORMAL_POS = 0.75;
    public static double LAT_ODO_RETRACT_POS = 0.48;

    // Intake
    public static double INTAKE_DISTANCE_THRESHOLD_AUTO = 30;
    public static double INTAKE_TIME_THRESHOLD_AUTO = 250;
    public static double INTAKE_DISTANCE_THRESHOLD_TELE = 30;
    public static double INTAKE_TIME_THRESHOLD_TELE = 0;
    public static double INTAKE_SENSOR_TRANSFER_THRESHOLD = 30;
    public static double INTAKE_RETRACT_POWER = 0.1;
    public static double INTAKE_POWER = 0.8;
    public static double INTAKE_TRANSFER_POWER = -0.8;
    public static double INTAKE_STALL_THRESHOLD = 6;
    public static double COLOR_SENSOR_THRESHOLD = 110;

    // Intake Slides
    public static int INTAKE_SLIDES_EXTEND_TICKS = 290;
    public static int INTAKE_SLIDES_HOME_TICKS = 50;
    public static int INTAKE_SLIDES_DUCK_HOME_TICKS = 30;
    public static double INTAKE_SLIDES_STALL_THRESHOLD = 1000; //TODO: tune


    // Intake Servo
    public static double INTAKE_UP_POS = 0.8;
    public static double INTAKE_DOWN_POS = 0.1;

    // Deposit Servo
    public static double DEPOSIT_OPEN_POS = 0.7;
    public static double DEPOSIT_HOLD_POS = 0;
    public static double DEPOSIT_RELEASE_POS = 0.9;
    public static double DEPOSIT_DUCK_RELEASE_POS = 1;
    public static double DEPOSIT_SHARED_RELEASE_POS = 1;
    public static double DEPOSIT_CAP_POS = 0.5;

    // Deposit Arm
    public static double ARM_ALLIANCE_POS = 0.2;
    public static double ARM_SHARED_POS = 0.3;
    public static double ARM_HOME_POS = 0.85;
    public static double ARM_INIT_POS = 0.75;
    public static double ARM_OFFSET = -0.0125;

    //Capping Arm
    public static double CAP_OPEN = 1;
    public static double CAP_CLOSE = 0;
    public static double CAP_INIT = 1;
    public static double CAP_HOME = 0.7;
    public static double CAP_UP = 0.5;
    public static double CAP_DOWN = 0.22;

    public static int DEPOSIT_SLIDES_HOME_TICKS = 0;
    public static int DEPOSIT_SLIDES_HIGH_TICKS = 500;
    public static int DEPOSIT_SLIDES_MID_TICKS = 200;
    public static int DEPOSIT_SLIDES_LOW_TICKS = 0;
    public static int DEPOSIT_SLIDES_CAP_TICKS = 500;
    public static double DEPOSIT_SLIDES_POWER = 1;

    // Carousel
    public static double CAROUSEL_VELOCITY_SLOW = 300;
    public static double CAROUSEL_VELOCITY_FAST = 1270;
    public static double CAROUSEL_SPEED_UP_THRESHOLD = 1100;

    public static double CAROUSEL_SPEED = 1;
    public static double CAROUSEL_AUTO_SPEED = 0.15;

    // Anti-Tip
    public static double TIP_THRESHOLD = 0.1;
    public static double TIP_CONTROLS_GAIN = 5;
}
