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
@TeleOp(name = "Tape Detector Test")
public class TapeDetectorTest extends LinearOpMode {
    double x, y, theta;
    public static boolean entering = false;
    public static boolean exiting = false;
    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain drivetrain = new Drivetrain(this, 0, 0, 0);
        TapeDetector tapeDetector = new TapeDetector(this);
        tapeDetector.entering();

        waitForStart();
        while (opModeIsActive()){
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

            double[] resetOdoCoords = tapeDetector.update(drivetrain.x, drivetrain.y, drivetrain.theta);
            drivetrain.resetOdo(drivetrain.x, resetOdoCoords[0], resetOdoCoords[1]);

            telemetry.addData("leftAlpha", tapeDetector.left.colorSensor.alpha());
            telemetry.addData("rightAlpha", tapeDetector.right.colorSensor.alpha());
            telemetry.addData("x", x);
            telemetry.addData("y", y);
            telemetry.addData("theta", theta);
            telemetry.update();

            addPacket("leftAlpha", tapeDetector.left.colorSensor.alpha());
            addPacket("rightAlpha", tapeDetector.right.colorSensor.alpha());
            addPacket("x", x);
            addPacket("y", y);
            addPacket("theta", theta);
            drawField();
            drawRobot(x, y, theta, 0, "grey");
            sendPacket();
            sendPacket();
        }
    }
}
