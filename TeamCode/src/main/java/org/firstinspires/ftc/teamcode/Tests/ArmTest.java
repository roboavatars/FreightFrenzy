package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp
@Config
public class ArmTest extends LinearOpMode {
    public static double servo1pos = 0;
    public static double offset = 0;

    public static boolean arm1Enable = true;
    public static boolean arm2Enable = true;

    @Override
    public void runOpMode() {
        ServoImplEx arm1 = hardwareMap.get(ServoImplEx.class, "arm1");
        ServoImplEx arm2 = hardwareMap.get(ServoImplEx.class, "arm2");

        waitForStart();

        while (opModeIsActive()) {
            double offset = -.0168224 * servo1pos + -.00285047;
            arm1.setPosition(servo1pos);
            arm2.setPosition(1-servo1pos+offset);

            if(arm1Enable) arm1.setPwmEnable();
            else arm1.setPwmDisable();
            if(arm2Enable) arm2.setPwmEnable();
            else arm2   .setPwmDisable();
        }
    }

}
