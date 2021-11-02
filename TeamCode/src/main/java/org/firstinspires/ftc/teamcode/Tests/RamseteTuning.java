package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotClasses.Drivetrain;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

@Autonomous
@Config
public class RamseteTuning extends LinearOpMode {
    private double b = 2;
    private double zeta = .7;
    private double target_front = 10;

    double target;

    @Override
    public void runOpMode(){
        Robot robot = new Robot(this,0,0,0, true, true);
        ElapsedTime time = new ElapsedTime();

        waitForStart();

        time.reset();
        while(opModeIsActive()){
            if (time.seconds() % 6 < 3){
                target = target_front;
            } else {
                target = 0;
            }

            robot.setTargetPoint(0, target, 0, 0, 0, 0, Drivetrain.Kp, Drivetrain.Ki,Drivetrain.Kd, b, zeta);

            telemetry.addData("current", robot.y);
            telemetry.addData("target", target);

            telemetry.update();
        }


    }
}
