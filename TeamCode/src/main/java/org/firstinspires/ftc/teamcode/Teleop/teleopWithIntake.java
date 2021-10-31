package org.firstinspires.ftc.teamcode.Teleop;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawDrivetrain;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawField;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;
import static java.lang.Math.PI;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotClasses.Drivetrain;
import org.firstinspires.ftc.teamcode.RobotClasses.Intake;

@TeleOp(name = "Intake Test")
public class teleopWithIntake extends LinearOpMode {

    @Override
    public void runOpMode() {
        Drivetrain dt = new Drivetrain(this, 90, 9, PI/2);
        Intake intake = new Intake(this);

        waitForStart();

        while(opModeIsActive()) {
            dt.setControls(-gamepad1.left_stick_y, gamepad1.right_stick_x);


            if (gamepad1.right_trigger>.5){
                intake.on(1);
            } else if (gamepad1.left_trigger > .5){
                intake.reverse(1);
            } else{
                intake.off();
            }
        }
    }
}