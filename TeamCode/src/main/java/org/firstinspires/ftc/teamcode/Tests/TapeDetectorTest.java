package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

@TeleOp(name = "Tape Detector Test")
public class TapeDetectorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this, 0,0, 0, false, true);
        robot.useTapeDetector = true;
        robot.tapeDetector.entering();

        while (opModeIsActive()){
            if (gamepad1.a){
                robot.tapeDetector.entering();
            } else if (gamepad1.b){
                robot.tapeDetector.exiting();
            }

            robot.drivetrain.setControls(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);

            telemetry.addData("leftAlpha", robot.tapeDetector.left.colorSensor.alpha());
            telemetry.addData("rightAlpha", robot.tapeDetector.right.colorSensor.alpha());
            telemetry.addData("x", robot.drivetrain.x);
            telemetry.addData("y", robot.drivetrain.y);
            telemetry.addData("theta", robot.drivetrain.theta);
            telemetry.update();

            addPacket("leftAlpha", robot.tapeDetector.left.colorSensor.alpha());
            addPacket("rightAlpha", robot.tapeDetector.right.colorSensor.alpha());
            sendPacket();
        }
    }
}
