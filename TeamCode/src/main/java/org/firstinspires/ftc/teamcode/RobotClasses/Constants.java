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
    public static double INTAKE_DISTANCE_THRESHOLD_AUTO = 0.5;
    public static double INTAKE_DISTANCE_THRESHOLD_TELE = 0.45;
    public static double STALL_THRESHOLD = 6;
    public static double COLOR_SENSOR_THRESHOLD = 110;

    // Intake Slides
    public static int INTAKE_SLIDES_EXTEND_TICKS = 100;
    public static int INTAKE_SLIDES_HOME_TICKS = 10;
    public static int INTAKE_SLIDES_DUCK_HOME_TICKS = 5;
    public static int INTAKE_SLIDES_STALL_THRESHOLD = 8;


    // Intake Servo
    public static double INTAKE_UP_POS = 0.75;
    public static double INTAKE_DOWN_POS = 0.08;

    // Deposit Servo
    public static double DEPOSIT_OPEN_POS = 0.65;
    public static double DEPOSIT_HOLD_POS = 0;
    public static double DEPOSIT_RELEASE_POS = 1;
    public static double DEPOSIT_CAP_POS = 0.5;

    // Deposit Arm
    public static double ARM_DEPOSIT_POS = 0.2;
    public static double ARM_HOME_POS = 0.87;
    public static double ARM_CAP_DOWN_POS = 0.15;
    public static double ARM_CAP_UP_POS = 0.15;
    public static double ARM_INIT_POS = 0.75;

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
