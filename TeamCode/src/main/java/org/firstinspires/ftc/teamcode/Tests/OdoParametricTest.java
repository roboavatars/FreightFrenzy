package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static java.lang.Math.PI;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

@TeleOp(name = "Odo Parametric Test")
@Disabled
public class OdoParametricTest extends LinearOpMode {

    private boolean started;
    private double startTime;

    private boolean constantHeading = false;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, 87, 81, PI/2, false);

        waitForStart();

        while(opModeIsActive()) {

            if (gamepad1.x) {
                robot.resetOdo(87, 81, PI/2);
            }

            if (started) {
                double t = (System.currentTimeMillis() - startTime) / 2000;
                robot.setTargetPoint(87 + 24 * sin(t) * cos(t), 81 + 48 * sin(t), constantHeading ? PI/2 : atan2(2 * cos(t), cos(2*t)));
                if (t > 4*PI) {
                    started = false;
                }
            } else {
                robot.setTargetPoint(87, 81, PI/2);
            }

            if (gamepad1.a) {
                if (!started) {
                    started = true;
                    startTime = System.currentTimeMillis();
                }
            }

            addPacket("podL", robot.drivetrain.podL);
            addPacket("podR", robot.drivetrain.podR);
            robot.update();
        }
    }
}