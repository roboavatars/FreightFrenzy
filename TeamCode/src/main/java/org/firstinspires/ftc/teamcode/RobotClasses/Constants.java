package org.firstinspires.ftc.teamcode.RobotClasses;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {

    // Drivetrain
    public static double ODO_NORMAL_POS = 0.45;
    public static double ODO_RETRACT_POS = 0.7;

    // Intake
    public static double INTAKE_DISTANCE_THRESHOLD = 70;
    public static double STALL_THRESHOLD = 6;
    public static double INTAKE_TRANSFER_POWER = -1;

    // Intake Slides
    public static double INTAKE_EXTEND_POS = 0.3;
    public static double INTAKE_MIDWAY_POS = 0.6;
    public static double INTAKE_HOME_POS = 0.6;
    public static double INTAKE_HOME_INIT_POS = 0.8;

    // Intake Servo
    public static double INTAKE_UP_POS = 0.8;
    public static double INTAKE_DOWN_POS = 0.15;
    public static double INTAKE_UP_INIT_POS = 0.8;

    // Deposit Servo
    public static double DEPOSIT_OPEN_POS = 0.23;
    public static double DEPOSIT_HOLD_POS = 0.5;

    // Deposit Arm
    public static int ARM_HIGH_POS = 535;
    public static int ARM_NEUTRAL_POS = 650;
    public static int ARM_HOME_POS = 0;
    public static int ARM_ON_HUB_THRESHOLD = ARM_HIGH_POS - 20;
    public static int ARM_ROTATE_TURRET_THRESHOLD = 600;

    // Carousel
    public static double CAROUSEL_VELOCITY = 2400;

    // Anti-Tip
    public static double TIP_THRESHOLD = 0.1;
    public static double TIP_CONTROLS_GAIN = 5;

    // Tape Detector
    public static double COLOR_SENSOR_THRESHOLD = 400;

    //Turret?
    public static double TURRET_NETRUAL_THETA = PI;
}
