package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Debug.Dashboard.addPacket;
import static org.firstinspires.ftc.teamcode.Debug.Dashboard.drawPoint;
import static java.lang.Math.PI;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Pathing.Path;
import org.firstinspires.ftc.teamcode.Pathing.Waypoint;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

@TeleOp(name = "Spline Generation / Following Test")
public class SplineTest extends LinearOpMode {
    private boolean followSpline = false;
    private double startTime, driveTime;
    private double pathTime = 4;
    private Path path;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, 90, 9, PI / 2, false, true);

        waitForStart();

        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()) {
            if (followSpline) {
                double curTime = Math.min(timer.seconds(), pathTime);
                robot.setTargetPoint(path.getRobotPose(curTime));

                driveTime = (double) System.currentTimeMillis() / 1000 - startTime;

                if (curTime == pathTime && (robot.notMoving() || robot.isAtPose(60, 50, PI))) {
                    followSpline = false;
                }
            } else {
                robot.drivetrain.setControls(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);

                if (gamepad1.a) {
                    followSpline = true;
                    startTime = (double) System.currentTimeMillis() / 1000;
                    timer.reset();
                }

                if (gamepad1.x) {
                    robot.drivetrain.resetOdo(90, 9, PI / 2);
                }

                Waypoint[] pathWaypoints = new Waypoint[]{
                        new Waypoint(robot.x, robot.y, robot.theta, 40, 40, 0, 0),
                        new Waypoint(90, 60, PI / 2, 40, 10, 0, 2),
                        new Waypoint(60, 70, PI, 30, -5, 0, pathTime)
                };
                path = new Path(pathWaypoints);
            }

            // Force Stop
            if (gamepad1.b) {
                followSpline = false;
                robot.drivetrain.stop();
            }

            // Draw Path
            for (double i = (followSpline ? driveTime : 0); i < pathTime; i += pathTime / 100) {
                try {
                    drawPoint(path.getRobotPose(i).x, path.getRobotPose(i).y, "blue");
                } catch (ArrayIndexOutOfBoundsException ignore) {
                }
            }

            addPacket("followSpline", followSpline);
            robot.update();
        }
    }
}
