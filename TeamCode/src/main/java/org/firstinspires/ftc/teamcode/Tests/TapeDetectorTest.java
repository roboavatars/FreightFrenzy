package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawField;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawRobot;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.sendPacket;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Localization.TapeDetector.TapeDetector;
import org.firstinspires.ftc.teamcode.RobotClasses.Drivetrain;

@Config
@TeleOp(name = "Tape Detector Color Sensor Test")
public class TapeDetectorTest extends LinearOpMode {
    double x, y, theta;
    public static boolean entering = false;
    public static boolean exiting = false;
    private double lastTime;
    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain drivetrain = new Drivetrain(this, 138, 83, Math.PI/2);
        TapeDetector tapeDetector = new TapeDetector(this);
        tapeDetector.entering();

        waitForStart();
        while (opModeIsActive()){
            drivetrain.setControls(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            drivetrain.updatePose();

            x = drivetrain.x;
            y = drivetrain.y;
            theta = drivetrain.theta;

            if (entering){
                entering = false;
                tapeDetector.entering();
            } else if (exiting){
                exiting = false;
                tapeDetector.exiting();
            }
            double[] resetOdoCoords = tapeDetector.update(x, y, theta);
            drivetrain.resetOdo(drivetrain.x, resetOdoCoords[0], drivetrain.theta);

            telemetry.addData("leftAlpha", tapeDetector.left.colorSensor.alpha());
//            telemetry.addData("rightAlpha", tapeDetector.right.colorSensor.alpha());
            telemetry.addData("x", x);
            telemetry.addData("y", y);
            telemetry.addData("theta", theta);
            telemetry.update();

            addPacket("4 leftAlpha", tapeDetector.left.colorSensor.alpha());
//            addPacket("4 rightAlpha", tapeDetector.right.colorSensor.alpha());
            addPacket("5 reset y", resetOdoCoords[0]);
            addPacket("5 reset theta", resetOdoCoords[1]);
            addPacket("6 x", x);
            addPacket("6 y", y);
            addPacket("6 theta", theta);
            drawField();
            drawRobot(x, y, theta, false, 0, theta, "grey");

            addPacket("7 loop time", 1000/(System.currentTimeMillis() - lastTime));
            lastTime = System.currentTimeMillis();

            sendPacket();
        }
    }
}
